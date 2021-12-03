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

#define LORA_ADR_ENABLE (1) /* beehise is not mobile */
#define LORA_TRACE_ENABLE (0)

#if (LORA_TRACE_ENABLE == 1)
#define LORA_TRACE_CrLf TRACE_CrLf
#define LORA_TRACE TRACE
#else
#define LORA_TRACE_CrLf
#define LORA_TRACE
#endif

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

static osjob_t initJob;
static osjob_t eventJob;
#if (STANDBY_ENABLE == 0)
  static osjob_t wakeupJob;
#endif
static osjob_t sendJob;
static uint8_t payload[MAX_LEN_PAYLOAD]; //put correct value

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
static fn_lora_event fn_event_;
#if (STANDBY_ENABLE == 0)
  fn_lora_wakeup fn_wakeup_;
#endif
static t_RamRet *pt_ramRet_;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static void lora_setAdr(bool enabled);
#if (STANDBY_ENABLE == 0)
  static void lora_wakeupJob(osjob_t* j);
#endif
static void lora_sendJob(osjob_t* j);
static void lora_initJob(osjob_t* j);
static void lora_requestNetworkTimeCallback(void *pVoidUserUTCTime, int flagSuccess);
#if((LMIC_ENABLE_onEvent == 0) && (LMIC_ENABLE_user_events == 1))  
  static void lora_onReceiveEvent(void *pUserData, uint8_t port, const uint8_t *pMessage, size_t nMessage);
  static void lora_onEvent(void *pUserData, ev_t ev);
#else
  void lora_onEvent(ev_t ev);
#endif
static int32_t lora_infoSave(t_LmicInfo *pt_lmicInfo);
static int32_t lora_infoErase(void);
static int32_t lora_infoLoad(t_LmicInfo *pt_lmicInfo);
static void lora_printInfo(t_LmicInfo *pt_lmicInfo);

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16); }

/***************************************************************************************
 *
 *	\fn		int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_receiveData fn_receiveData, fn_lora_event fn_event); 
 *	\brief 
 *
 ***************************************************************************************/
#if (STANDBY_ENABLE == 1)
int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_event fn_event) {
  if((pt_ramRet == NULL) || (fn_sendData == NULL) || (fn_event == NULL))
    return ERROR;
#else
int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_event fn_event, fn_lora_wakeup fn_wakeup) {
  if((pt_ramRet == NULL) || (fn_sendData == NULL) || (fn_event == NULL) || (fn_wakeup == NULL))
    return ERROR;

  fn_wakeup_ = fn_wakeup;
#endif

  fn_sendData_ = fn_sendData;
  fn_event_ = fn_event;
  pt_ramRet_ = pt_ramRet;

  memset(&lmicInfo, 0x00, sizeof(t_LmicInfo));
  
  SPI.setMOSI(SPI1_MOSI);      
  SPI.setMISO(SPI1_MISO);
  SPI.setSCLK(SPI1_SCK);

  os_init();

  LMIC_reset(); 
  
#if	(LMIC_ENABLE_arbitrary_clock_error == 1)  
  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100); /* if use hsi 16 mHz without pll (SystemClock_Config) */
#endif

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band*/

  /* sf9 for rx2 */
  LMIC.dn2Dr = DR_SF9;

#if 0  
  if(power_isPoweredOn() || pt_ramRet_->loraSettings.waitDownlinkCounter++ > pt_ramRet_->loraSettings.waitDownlinkMaxCycle) {
    LMIC.rxDelay = 5; // TTN V3 Rx1 delay: 5 or other number depending on the config
    pt_ramRet_->loraSettings.waitDownlinkCounter = 0;
  } else {
    LMIC.rxDelay = 0; // must be check
  }
#else
  LMIC.rxDelay = 5; // TTN V3 Rx1 delay: 5 or other number depending on the console config and network
#endif

  // Disable link-check mode and ADR, because ADR tends to complicate testing.
  LMIC_setLinkCheckMode(0);  

  // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
  // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
  LMIC_setDrTxpow(DR_SF7, 14);  

#if((LMIC_ENABLE_onEvent == 0) && (LMIC_ENABLE_user_events == 1))  
  LMIC_registerEventCb(lora_onEvent, (void*) pt_ramRet);
  LMIC_registerRxMessageCb(lora_onReceiveEvent, (void*) pt_ramRet);
#endif

  os_setCallback(&initJob, lora_initJob); 

  return OK;  
}

/***************************************************************************************
 *
 *	\fn		int32_t lora_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t lora_suspend(void) {
  LMIC_shutdown();

  //pinMode(SPI1_MOSI, INPUT);
  //pinMode(SPI1_MISO, INPUT_ANALOG);
  //pinMode(SPI1_SCK, INPUT);
  //pinMode(SPI1_NSS, INPUT);
  
  //pinMode(LORA_RST, INPUT);
  
  //pinMode(LORA_DIO0, INPUT);
  //pinMode(LORA_DIO1, INPUT);
  //pinMode(LORA_DIO2, INPUT);

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
   //os_queryTimeCriticalJobs(ms2osticks(n))
}

/***************************************************************************************
 *
 *	\fn		int32_t lora_getTime(void
 *	\brief 
 *
 ***************************************************************************************/
int32_t lora_getTime(void) {
  return osticks2us(os_getTime());
}

/***************************************************************************************
 *
 *	\fn		static void lora_sendJob(osjob_t* j)
 *	\brief 
 *
 ***************************************************************************************/
//static void lora_eventJob(osjob_t* j) {

//}

#if (STANDBY_ENABLE == 0)
/***************************************************************************************
 *
 *	\fn		void lora_schedule(uint32_t time)
 *	\brief 
 *
 ***************************************************************************************/
void lora_schedule(uint32_t time) {
  LMIC_clrTxData();
  
  os_setTimedCallback(&wakeupJob, os_getTime() + sec2osticks(time), lora_wakeupJob);
}

/***************************************************************************************
 *
 *	\fn		static void lora_setAdr(bool enabled)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_setAdr(bool enabled) {
  LMIC_setAdrMode(enabled);
  LMIC_setLinkCheckMode(!enabled);
}

/***************************************************************************************
 *
 *	\fn		static void lora_wakeupJob(osjob_t* j)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_wakeupJob(osjob_t* j) {
  if(fn_wakeup_ != NULL)
    fn_wakeup_();

  os_setCallback(&sendJob, lora_sendJob);
}
#endif

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
    LORA_TRACE_CrLf("[LORA] OP_TXRXPEND, not sending");
    //todo event 
  } else {
    if(fn_sendData_ != NULL) {
      int32_t ret = fn_sendData_(payload, sizeof(payload), &payloadSize);

      if(ret == DATA_NOT_CHANGED) {
        if(fn_event_ != NULL)
          fn_event_(e_DATA_NOT_CHANGED, NULL, 0);
      } else if(ret == ERROR) {
        if(fn_event_ != NULL)
          fn_event_(e_SEND_FAILED, NULL, 0);
      } else {
        if(pt_ramRet_->timeUpdated == FALSE)  
          LMIC_requestNetworkTime(lora_requestNetworkTimeCallback, &userUTCTime);

        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.     
        LMIC_setTxData2(1, payload, payloadSize, 0);

        LORA_TRACE_CrLf("[LORA] packet queued (%d bytes)", payloadSize);

        if(fn_event_ != NULL)
          fn_event_(e_TX_DATA_QUEUED, NULL, 0);
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

      LMIC.seqnoUp = pt_ramRet_->loraSettings.seqnoUp;
      LMIC.seqnoDn = pt_ramRet_->loraSettings.seqnoDown;

      LMIC_setAdrMode(LORA_ADR_ENABLE);
      LMIC_setLinkCheckMode(!LORA_ADR_ENABLE);

      os_setCallback(&sendJob, lora_sendJob);
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
 *	\fn		static void lora_requestNetworkTimeCallback(void *pVoidUserUTCTime, int flagSuccess)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_requestNetworkTimeCallback(void *pVoidUserUTCTime, int flagSuccess) {
  uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

  lmic_time_reference_t lmicTimeReference;

  if(flagSuccess != 1) {
    LORA_TRACE_CrLf("[LORA] USER CALLBACK: Not a success");
    return;
  }

  flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
  if (flagSuccess != 1) {
    LORA_TRACE_CrLf("[LORA] USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed");
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
    fn_event_(e_RX_TIME, (void*) pUserUTCTime, 0);
}

#if((LMIC_ENABLE_onEvent == 0) && (LMIC_ENABLE_user_events == 1)) 
/***************************************************************************************
 *
 *	\fn		void lora_onReceiveEvent(void *pUserData, uint8_t port, const uint8_t *pMessage, size_t nMessage)
 *	\brief 
 *
 ***************************************************************************************/
void lora_onReceiveEvent(void *pUserData, uint8_t port, const uint8_t *pMessage, size_t nMessage) {
  TRACE_CrLf("[LORA] receive event, size %d bytes", nMessage);
}

/***************************************************************************************
 *
 *	\fn		void lora_onEvent (ev_t ev)
 *	\brief 
 *
 ***************************************************************************************/
void lora_onEvent(void *pUserData, ev_t ev) {
#else
void lora_onEvent(ev_t ev) {
#endif
  //Serial.print(os_getTime());
  //Serial.print(": ");

  switch(ev) {
    case EV_SCAN_TIMEOUT:
      LORA_TRACE_CrLf("[LORA] EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      LORA_TRACE_CrLf("[LORA] EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      LORA_TRACE_CrLf("[LORA] EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      LORA_TRACE_CrLf("[LORA] EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      LORA_TRACE_CrLf("[LORA] EV_JOINING");
      break;
    case EV_JOINED:
      LORA_TRACE_CrLf("[LORA] EV_JOINED");
            
      LMIC_getSessionKeys(&lmicInfo.netid, &lmicInfo.devaddr, lmicInfo.nwkKey, lmicInfo.artKey);

      lora_infoSave(&lmicInfo);
      
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      LMIC_setAdrMode(LORA_ADR_ENABLE);

      os_setCallback(&sendJob, lora_sendJob);
      break;
    case EV_JOIN_FAILED:
      LORA_TRACE_CrLf("[LORA] EV_JOIN_FAILED");
      //todo after counter erase lmic info
      break;
    case EV_REJOIN_FAILED:
      LORA_TRACE_CrLf("[LORA] EV_REJOIN_FAILED");
      //todo after counter erase lmic info
      break;
    case EV_TXCOMPLETE:
      LORA_TRACE_CrLf("[LORA] EV_TXCOMPLETE (includes waiting for RX windows)");
      
      if (LMIC.txrxFlags & TXRX_ACK) {
        if(fn_event_ != NULL)
          fn_event_(e_TX_RX_ACK, NULL, 0);
      }

      if (LMIC.dataLen) {
        if(fn_event_ != NULL)
          fn_event_(e_RX_DATA, (void*) &LMIC.frame[LMIC.dataBeg], LMIC.dataLen);
      }

      pt_ramRet_->loraSettings.seqnoUp = LMIC.seqnoUp;
      pt_ramRet_->loraSettings.seqnoDown = LMIC.seqnoDn;

      if(fn_event_ != NULL)
        fn_event_(e_TX_DONE, NULL, 0);
      break;
    case EV_LOST_TSYNC:
      LORA_TRACE_CrLf("[LORA] EV_LOST_TSYNC");
      break;
    case EV_RESET:
      LORA_TRACE_CrLf("[LORA] EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      LORA_TRACE_CrLf("[LORA] EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      LORA_TRACE_CrLf("[LORA] EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      LORA_TRACE_CrLf("[LORA] EV_LINK_ALIVE");
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
      LORA_TRACE_CrLf("[LORA] EV_TXSTART");
      break;
    case EV_TXCANCELED:
      LORA_TRACE_CrLf("[LORA] EV_TXCANCELED");
      //todo restart dosend or after counter erase lmic info
      break;
    case EV_RXSTART:
      //todo compute rx time for monitoring
      
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      LORA_TRACE_CrLf("[LORA] EV_JOIN_TXCOMPLETE: no JoinAccept");
      //todo after counter erase lmic info
      break;
       
    default:
      LORA_TRACE_CrLf("[LORA] Unknown event: %d", (uint32_t) ev);
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