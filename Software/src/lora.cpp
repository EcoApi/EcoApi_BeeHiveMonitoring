/************************************************************************************//**
 *
 *	\file		lora.cpp
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes				
/***************************************************************************************/
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <lmic/lmic_util.h>
#include <EEPROM.h>
#include "lora.h"
#include "trace.h"

#if(USE_MY_CREDENTIALS == 1)
    #include "credentials.h"
#else
    #include "credentials_template.h"
#endif

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define MAX_EEPROM_DATA_SIZE E2END
#define LMIC_MAGIC 0x4C4D4943 /* LMIC in ascii */

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef struct t_LmicInfo_ {
  uint32_t magicStart;
  u4_t netid;
  devaddr_t devaddr;
  u1_t nwkKey[16];
  u1_t artKey[16];
  uint32_t magicEnd;
} t_LmicInfo;

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static t_LmicInfo lmicInfo;

static osjob_t initjob ;
static osjob_t sendjob;
static uint8_t payload[50]; //put correct value

static uint32_t userUTCTime; // Seconds since the UTC epoch

const lmic_pinmap lmic_pins = {
  .nss = SPI1_NSS, //LORA_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST,
  .dio = {LORA_DIO0,LORA_DIO1,LORA_DIO2},
  //.rxtx_rx_active = 0,
  //.rssi_cal = 10,
  .spi_freq = 8000000     /* 8 MHz */
};

static fn_lora_sendData fn_sendData_;
static fn_lora_receiveData fn_receiveData_;
static fn_lora_event fn_event_;
static t_RamRet *pt_ramRet_;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static void lora_sendJob(osjob_t* j);
static void lora_initJob(osjob_t* j);
void onEvent(ev_t ev);
static int32_t lora_infoSave(t_LmicInfo *pt_lmicInfo);
static int32_t lora_infoErase(void);
static int32_t lora_infoLoad(t_LmicInfo *pt_lmicInfo);
static void lora_printInfo(t_LmicInfo *pt_lmicInfo);
static void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess);

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16); }

/***************************************************************************************
 *
 *	\fn		int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_receiveData fn_receiveData, fn_lora_event fn_event); 
 *	\brief 
 *
 ***************************************************************************************/
int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_receiveData fn_receiveData, fn_lora_event fn_event) {
  if((pt_ramRet == NULL) || (fn_sendData == NULL) || (fn_event == NULL))
    return ERROR;
  
  fn_sendData_ = fn_sendData;
  fn_receiveData_ = fn_receiveData;
  fn_event_ = fn_event;
  pt_ramRet_ = pt_ramRet;

  memset(&lmicInfo, 0x00, sizeof(t_LmicInfo));
  
  SPI.setMOSI(SPI1_MOSI);      
  SPI.setMISO(SPI1_MISO);
  SPI.setSCLK(SPI1_SCK);

  os_init();

  LMIC_reset(); 
  
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  /*LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band*/

  //LMIC.dn2Dr = DR_SF9;

  //#ifdef TTNSTACKV3
  // essential for TTNv3, otherwise you will get a Scheduled downlink message
  // Rx1 delay: 5 or other number depending on the config
  LMIC.rxDelay = 5;
  //#endif

  // Disable link-check mode and ADR, because ADR tends to complicate testing.
  LMIC_setLinkCheckMode(0);  

  // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
  // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
  LMIC_setDrTxpow(DR_SF7, 14);  
  
  os_setCallback(&initjob, lora_initJob); 

  return OK;  
}

/***************************************************************************************
 *
 *	\fn		int32_t lora_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t lora_suspend(void) {

  return OK;
}

/***************************************************************************************
 *
 *	\fn		void lora_process(void)
 *	\brief 
 *
 ***************************************************************************************/
void lora_process(void) {
   os_runloop_once(); // Readings taken, sent and looped in the do_send() function called from setup
}

/***************************************************************************************
 *
 *	\fn		void lora_schedule(uint32_t time)
 *	\brief 
 *
 ***************************************************************************************/
void lora_schedule(uint32_t time) {
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(time), lora_sendJob);
}

/***************************************************************************************
 *
 *	\fn		static void lora_sendJob(osjob_t* j)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_sendJob(osjob_t* j) {
  uint8_t payloadSize = 0, i;
  uint16_t payloadTmp;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    TRACE_CrLf("[LORA] OP_TXRXPEND, not sending");
  } else {
    if(fn_sendData_ != NULL) {
      int32_t ret = fn_sendData_(payload, sizeof(payload), &payloadSize);

      if(ret == DATA_NOT_CHANGED) {
        if(fn_event_ != NULL)
          fn_event_(e_DATA_NOT_CHANGED, NULL);
      } else if(ret == ERROR) {
        if(fn_event_ != NULL)
          fn_event_(e_SEND_FAILED, NULL);
      } else {
        if(pt_ramRet_->timeUpdated == FALSE)  
          LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);

        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.     
        LMIC_setTxData2(1, payload, payloadSize, 0);
       
        TRACE_CrLf("[LORA] packet queued");
      }
    }
  }
}

/***************************************************************************************
 *
 *	\fn		static void lora_initJob(osjob_t* j)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_initJob(osjob_t* j) {
  if(ramret_isNew() != TRUE) {
    if((pt_ramRet_->forceNewJoining == FALSE) && lora_infoLoad(&lmicInfo) == OK) {
      LMIC_setSession(lmicInfo.netid, lmicInfo.devaddr, lmicInfo.nwkKey, lmicInfo.artKey);

      LMIC.seqnoUp = pt_ramRet_->LORA_seqnoUp;
      LMIC.seqnoDn = pt_ramRet_->LORA_seqnoDown;
    
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), lora_sendJob);
      return; 
    }

    if(pt_ramRet_->forceNewJoining == TRUE) {
      TRACE_CrLf("[LORA] force joining") ;
      pt_ramRet_->forceNewJoining = FALSE;
    }
  } 

  lora_infoErase(); 

  TRACE_CrLf("[LORA] start joining") ;
  LMIC_startJoining() ;

  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(1);
}

/***************************************************************************************
 *
 *	\fn		static void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess)
 *	\brief 
 *
 ***************************************************************************************/
static void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
  uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

  lmic_time_reference_t lmicTimeReference;

  TRACE("[LORA] ");

  if(flagSuccess != 1) {
    TRACE_CrLf("USER CALLBACK: Not a success");
    return;
  }

  flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
  if (flagSuccess != 1) {
    TRACE_CrLf("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed");
    return;
  }

  *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

  // Add the delay between the instant the time was transmitted and
  // the current time

  // Current time, in ticks
  ostime_t ticksNow = os_getTime();
  // Time when the request was sent, in ticks
  ostime_t ticksRequestSent = lmicTimeReference.tLocal;
  uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
  *pUserUTCTime += requestDelaySec;

  if(fn_event_ != NULL)
    fn_event_(e_RX_TIME, (void*) pUserUTCTime);
}

/***************************************************************************************
 *
 *	\fn		void onEvent (ev_t ev)
 *	\brief 
 *
 ***************************************************************************************/
void onEvent (ev_t ev) {
  //Serial.print(os_getTime());
  //Serial.print(": ");

  TRACE("[LORA] ");

  switch(ev) {
    case EV_SCAN_TIMEOUT:
      TRACE_CrLf("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      TRACE_CrLf("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      TRACE_CrLf("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      TRACE_CrLf("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      TRACE_CrLf("EV_JOINING");
      break;
    case EV_JOINED:
      TRACE_CrLf("EV_JOINED");
            
      LMIC_getSessionKeys(&lmicInfo.netid, &lmicInfo.devaddr, lmicInfo.nwkKey, lmicInfo.artKey);

      lora_infoSave(&lmicInfo);
      
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);

      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), lora_sendJob);
      break;
    case EV_JOIN_FAILED:
      TRACE_CrLf("EV_JOIN_FAILED");
      //todo after counter erase lmic info
      break;
    case EV_REJOIN_FAILED:
      TRACE_CrLf("EV_REJOIN_FAILED");
      //todo after counter erase lmic info
      break;
    case EV_TXCOMPLETE:
      TRACE_CrLf("EV_TXCOMPLETE (includes waiting for RX windows)");
      
      if (LMIC.txrxFlags & TXRX_ACK) {
        TRACE_CrLf("[LORA] received ack");
      }

      if (LMIC.dataLen) {
        //TRACE_CrLf("[LORA] received %d bytes of payload", LMIC.dataLen);

        if(fn_receiveData_ != NULL)
          fn_receiveData_(&LMIC.frame[LMIC.dataBeg] , LMIC.dataLen);
      }

      pt_ramRet_->LORA_seqnoUp = LMIC.seqnoUp;
      pt_ramRet_->LORA_seqnoDown = LMIC.seqnoDn;

      if(fn_event_ != NULL)
        fn_event_(e_TX_COMPLETE, NULL);
      break;
    case EV_LOST_TSYNC:
      TRACE_CrLf("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      TRACE_CrLf("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      TRACE_CrLf("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      TRACE_CrLf("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      TRACE_CrLf("EV_LINK_ALIVE");
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    TRACE_CrLf("EV_SCAN_FOUND");
      ||    break;
    */
    case EV_TXSTART:
      TRACE_CrLf("EV_TXSTART");
      break;
    case EV_TXCANCELED:
      TRACE_CrLf("EV_TXCANCELED");
      //todo restart dosend or after counter erase lmic info
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      TRACE_CrLf("EV_JOIN_TXCOMPLETE: no JoinAccept");
      //todo after counter erase lmic info
      break;
       
    default:
      TRACE_CrLf("Unknown event: %d", (uint32_t) ev);
      break;
  }
}

/***************************************************************************************
 *
 *	\fn		static int32_t lora_infoSave(t_LmicInfo *pt_lmicInfo)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t lora_infoSave(t_LmicInfo *pt_lmicInfo) {
  uint16_t i;

  if((pt_lmicInfo == NULL) || (sizeof(t_LmicInfo) > MAX_EEPROM_DATA_SIZE)) {
    TRACE_CrLf("[LORA] save info arg error");
    return ERROR;
  }

  pt_lmicInfo->magicStart = LMIC_MAGIC;
  pt_lmicInfo->magicEnd = LMIC_MAGIC;

  uint8_t *pt_lmicRawData = (uint8_t*) pt_lmicInfo;

  for(i=0;i<sizeof(t_LmicInfo);i++)
    eeprom_buffered_write_byte(i, pt_lmicRawData[i]);
  
  eeprom_buffer_flush();

  TRACE_CrLf("[LORA] save info");
  lora_printInfo(pt_lmicInfo);

  return OK;
}

/***************************************************************************************
 *
 *	\fn		static int32_t lora_infoErase(void)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t lora_infoErase(void) {
  uint16_t i;

  for(i=0;i<sizeof(t_LmicInfo);i++)
    eeprom_buffered_write_byte(i, 0x00);
  
  eeprom_buffer_flush();

  TRACE_CrLf("[LORA] erase info");

  return OK;
}

/***************************************************************************************
 *
 *	\fn		static int32_t lora_infoLoad(t_LmicInfo *pt_lmicInfo)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t lora_infoLoad(t_LmicInfo *pt_lmicInfo) {
  uint16_t i;

  if((pt_lmicInfo == NULL) || (sizeof(t_LmicInfo) > MAX_EEPROM_DATA_SIZE)) {
    TRACE_CrLf("[LORA] load info arg error");
    return ERROR;
  }

  uint8_t *pt_lmicRawData = (uint8_t*) pt_lmicInfo;

  eeprom_buffer_fill();

  for(i=0;i<sizeof(t_LmicInfo);i++)
    pt_lmicRawData[i] = eeprom_buffered_read_byte(i);

  if((pt_lmicInfo->magicStart != LMIC_MAGIC) || (pt_lmicInfo->magicEnd != LMIC_MAGIC)) {
    TRACE_CrLf("[LORA] load info magic error");
    return ERROR;
  }

  TRACE_CrLf("[LORA] load info");
  lora_printInfo(pt_lmicInfo);

  return OK;
}

/***************************************************************************************
 *
 *	\fn		static void lora_printInfo(t_LmicInfo *pt_lmicInfo)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_printInfo(t_LmicInfo *pt_lmicInfo) {
  if(pt_lmicInfo == NULL)
    return;
  
  TRACE_CrLf("[LORA] netid: %d, devaddr %08x", pt_lmicInfo->netid, pt_lmicInfo->devaddr);
  
  TRACE("[LORA] AppSKey:");
  for (size_t i=0; i<sizeof(pt_lmicInfo->artKey); ++i)
    TRACE(" %02x", pt_lmicInfo->artKey[i]);
  TRACE_CrLf("");
  
  TRACE("[LORA] NwkSKey:");
  for (size_t i=0; i<sizeof(pt_lmicInfo->nwkKey); ++i)
    TRACE(" %02x", pt_lmicInfo->nwkKey[i]);
  TRACE_CrLf("");
}

#if LMIC_ENABLE_event_logging
/***************************************************************************************
 *
 *	\fn		void trace_init(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
extern "C" {
    void LMICOS_logEvent(const char *pMessage);
    void LMICOS_logEventUint32(const char *pMessage, uint32_t datum);
}

/***************************************************************************************
 *
 *	\fn		void trace_init(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
void LMICOS_logEvent(const char *pMessage) {
  TRACE_CrLf("%s", pMessage);
}

/***************************************************************************************
 *
 *	\fn		void trace_init(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
void LMICOS_logEventUint32(const char *pMessage, uint32_t datum) {
  TRACE_CrLf("%s %d", pMessage, datum); 
}
#endif // LMIC_ENABLE_event_logging