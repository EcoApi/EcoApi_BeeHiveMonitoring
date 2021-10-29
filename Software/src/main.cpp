#include "board.h"
#include <Wire.h>
#include <EEPROM.h>
#include "trace.h"
#include "power.h"
#include "system.h"
#include "rtc.h"
#include "ramret.h"

#define LORA_ENABLE (1) 
#define BMP180_ENABLE (1)
#define ONEWIRE_ENABLE (1)
#define ANALOG_ENABLE (1)
#define HX711_ENABLE (1)
#define MAX9815_ENABLE (0)
#define TILT_ENABLE (0)
#define WDG_ENABLE (0)

#define STM32_ID	((u1_t *) 0x1FFFF7E8)

static void actionButtonHandler(void);

bool b_actionButtonPressed = false;

#define MAX_TEMPERATURE_1WIRE 5

typedef struct t_1WireTemperature_ {
  byte address[8];
  float temperature;
} t_1WireTemperature;

typedef struct t_telemetryData_ {
  //1wire
  t_1WireTemperature temperature1Wire[MAX_TEMPERATURE_1WIRE];
  uint8_t nTemperature1Wire;

  //bmp180
  double temperature;
  double pressure;
  double pressureCompensated;
  double altitude;

  //hx711
  float weight;
  float weightAverage;
  long currentOffset;
  float calibration_factor; 
  uint8_t scaleType;

  //analog
  int32_t anaRing;
  int32_t vbattExt;
  float vbattExtPercent;
  int32_t vref;
  int32_t vbatt;
  int32_t temperatureInternal;
} t_telemetryData;

t_telemetryData telemetryData;
t_RamRet t_ramRet;


#if (ANALOG_ENABLE == 1)
#include "adc.h"
static int32_t getExternalAnaRing(int32_t vref_mvolt);
static int32_t getExternalVbatt(int32_t vref_mvolt);
static int32_t getInternalTemp(int32_t vref_mvolt); 
static int32_t getInternalVbatt(int32_t vref_mvolt);
static int32_t getInternalVrefInternal(int32_t vref_mvolt);
static int32_t getInternalVref(void);
static void Analog_getData(t_telemetryData *pt_telemetryData);
#endif

#if (HX711_ENABLE == 1)
#include "HX711.h"

#define LB2KG  0.45352
#define CALWEIGHT 2.00 //Kg
#define DEFAULT_CALIFACTOR -7050

HX711 scale;

static void HX711_setup(void);
static void HX711_calibrate(t_telemetryData *pt_telemetryData);
static void HX711_getData(t_telemetryData *pt_telemetryData); 
#endif

#if (BMP180_ENABLE == 1)
#include <SFE_BMP180.h>

#define ALTITUDE 51.0 // Altitude of EcoApi pont de l'arche France 

SFE_BMP180 bmp180;

static void BMP180_setup(void);
static void BMP180_getData(t_telemetryData *pt_telemetryData);
#endif

#if (ONEWIRE_ENABLE == 1)
#include <OneWire.h>

OneWire ow(ONEWIRE_DATA);  

static void OneWire_getData(t_telemetryData *pt_telemetryData);
#endif

#if (LORA_ENABLE == 1)
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <lmic/lmic_util.h>

#if(USE_MY_CREDENTIALS == 1)
    #include "credentials.h"
#else
    #include "credentials_template.h"
#endif

#define MAX_EEPROM_DATA_SIZE E2END
#define LMIC_MAGIC 0x4C4D4943 /* LMIC in ascii */

typedef struct t_LmicInfo_ {
  uint32_t magicStart;
  u4_t netid;
  devaddr_t devaddr;
  u1_t nwkKey[16];
  u1_t artKey[16];
  uint32_t magicEnd;
} t_LmicInfo;

t_LmicInfo lmicInfo;

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16); }

static void LMIC_setup(void);
static void do_send(osjob_t* j);
static void initfunc(osjob_t* j);
void onEvent(ev_t ev);
static int32_t LMIC_infoSave(t_LmicInfo *pt_lmicInfo);
static int32_t LMIC_infoErase(void);
static int32_t LMIC_infoLoad(t_LmicInfo *pt_lmicInfo);
void LMIC_printInfo(t_LmicInfo *pt_lmicInfo);
void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess);

static osjob_t initjob ;
static osjob_t sendjob;
static uint8_t payload[50]; //put correct value

float humidity_ = 0;
uint32_t userUTCTime; // Seconds since the UTC epoch

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const uint32_t TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
  .nss = SPI1_NSS, //LORA_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST,
  .dio = {LORA_DIO0,LORA_DIO1,LORA_DIO2},
  //.rxtx_rx_active = 0,
  //.rssi_cal = 10,
  .spi_freq = 8000000     /* 8 MHz */
};
#endif

void setup(void) {
  system_clockConfig_16MHz(); // reduce cpu consumption

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SCALE_TYPE, INPUT);
  pinMode(BTN_TARE, INPUT);
  pinMode(BTN_ACT, INPUT);
  pinMode(ANA_VBATT, INPUT_ANALOG);
  pinMode(RING, INPUT);
  pinMode(LED_INFO, OUTPUT);

  //clock debug with mco pin
  //HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

  int32_t powerInit = power_init();
  int32_t rtcInit = rtc_init();
  int32_t rtcStart = rtc_start(0);
  int32_t ramretInit = ramret_init(&t_ramRet);

  t_ramRet.boot++;

  trace_init(&t_ramRet);

  { //todo move in trace init function
#if 1
    if(power_isPoweredOn()) {
#else
    if(ramretInit) {
#endif
      uint32_t traceActivationTimeout = 3000;
    
      attachInterrupt(digitalPinToInterrupt(BTN_ACT), actionButtonHandler, FALLING);
    
      while(traceActivationTimeout) {
        digitalWrite(LED_INFO, HIGH);
        delay(50);
        digitalWrite(LED_INFO, LOW);
        delay(50);
        traceActivationTimeout -= 100;
      };
      digitalWrite(LED_INFO, LOW);

      detachInterrupt(digitalPinToInterrupt(BTN_ACT));

      trace_setState(b_actionButtonPressed);
    }

    Serial.setTx(UART1_TX);
    Serial.setRx(UART1_RX);

    Serial.begin(115200); // Sets the speed needed to read from the serial port when connected
    while (!Serial); // Loop that only continues once the serial port is active (true)
  }

  memset(&telemetryData, 0x00, sizeof(t_telemetryData)); // todo ram retention

  TRACE_CrLf("################");
  TRACE_CrLf("[BOOT] EcoApi BeeHive Monitor");

  //https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog
  
  if(powerInit == STANDBY_RESUMED) {
    TRACE_CrLf("[POWER] resume standby");
  } else if(powerInit != OK) {
    TRACE_CrLf("[POWER] init ko");
  }

  power_traceState();

  if(rtcInit != OK) {
    TRACE_CrLf("[RTC] init ko");
  } else
    TRACE_CrLf("[RTC] init ok");

  if(rtcStart != OK) {
    TRACE_CrLf("[RTC] start ko");
  } else
    TRACE_CrLf("[RTC] start ok | timestamp: %d", rtc_read());

  if(ramretInit) { 
    TRACE_CrLf("[RRAM] initialized");
  } else
    TRACE_CrLf("[RRAM] restored");

  Wire.setSCL(I2C1_SCL);
  Wire.setSDA(I2C1_SDA);

#if (BMP180_ENABLE == 1)
  BMP180_setup();
#endif

#if (HX711_ENABLE == 1)
  HX711_setup();
#endif

#if (LORA_ENABLE == 1)
  LMIC_setup();
#endif

  //attachInterrupt(digitalPinToInterrupt(BTN_TARE), tareButtonHandler, FALLING);


#if (RAMRET_ENABLE ==1)
  ramret_save(&t_ramRet);
#endif

  pinMode(LED_INFO, INPUT_ANALOG); 
}

void loop(void) {
#if (LORA_ENABLE == 1)
  os_runloop_once(); // Readings taken, sent and looped in the do_send() function called from setup
#else

#if (RTC_ENABLE == 1)
  TRACE_CrLf("[RTC] timestamp: %d", rtc_read());
#endif
#if (ONEWIRE_ENABLE == 1)    
  OneWire_getData(&telemetryData);
#endif
#if (BMP180_ENABLE == 1)
  BMP180_getData(&telemetryData);
#endif
#if (HX711_ENABLE == 1)
  HX711_getData(&telemetryData);
#endif
#if (ANALOG_ENABLE == 1)
  Analog_getData(&telemetryData);
#endif

  delay(1000);
#endif
}

static void actionButtonHandler(void) {
  b_actionButtonPressed = true;
}

#if (LORA_ENABLE == 1)
static void LMIC_setup(void) {
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
  
  os_setCallback(&initjob, initfunc) ;   
}  

#if LMIC_ENABLE_event_logging
extern "C" {
    void LMICOS_logEvent(const char *pMessage);
    void LMICOS_logEventUint32(const char *pMessage, uint32_t datum);
}

void LMICOS_logEvent(const char *pMessage) {
  TRACE_CrLf("%s", pMessage);
}

void LMICOS_logEventUint32(const char *pMessage, uint32_t datum) {
  TRACE_CrLf("%s %d", pMessage, datum); 
}
#endif // LMIC_ENABLE_event_logging



static void do_send(osjob_t* j) {
  uint8_t payloadSize = 0, i;
  uint16_t payloadTmp;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    TRACE_CrLf("[LMIC] OP_TXRXPEND, not sending");
  } else {
    if(t_ramRet.timeUpdated == FALSE)  
      LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);

    //digitalWrite(LED_BUILTIN, LOW); 

#if (ONEWIRE_ENABLE == 1)    
    OneWire_getData(&telemetryData);
#endif
#if (BMP180_ENABLE == 1)
    BMP180_getData(&telemetryData);
#endif
#if (HX711_ENABLE == 1)
    HX711_getData(&telemetryData);
#endif
#if (ANALOG_ENABLE == 1)
    Analog_getData(&telemetryData);
#endif

    if(humidity_++ > 100)
      humidity_ = 0;

    //temperature
    payloadTmp = telemetryData.temperature * 100;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);

    //humidity
    payloadTmp = humidity_ * 100;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);
    
    //pressure
    payloadTmp = telemetryData.pressureCompensated; //* 100;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);
    
    //weight    
    payloadTmp = telemetryData.weightAverage; //* 100;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);

    //vbatt
    payloadTmp = telemetryData.vbattExt;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);

    //vbatt percent
    payloadTmp = telemetryData.vbattExtPercent * 100;
    payload[payloadSize++] = highByte(payloadTmp);
    payload[payloadSize++] = lowByte(payloadTmp);

    //scale type
    payload[payloadSize++] = telemetryData.scaleType;

    //temerature ow
    payload[payloadSize++] = telemetryData.nTemperature1Wire;
    for(i=0;i<telemetryData.nTemperature1Wire;i++) {
      payloadTmp = telemetryData.temperature1Wire[i].temperature * 100;
      payload[payloadSize++] = highByte(payloadTmp);
      payload[payloadSize++] = lowByte(payloadTmp);
    }

    // prepare upstream data transmission at the next possible time.
    // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.     
    LMIC_setTxData2(1, payload, payloadSize, 0);
       
    TRACE_CrLf("[LMIC] packet queued");

    //digitalWrite(LED_BUILTIN, HIGH); 
  }
}

static void initfunc (osjob_t* j) {
  if(ramret_isNew() != TRUE) {
    if(LMIC_infoLoad(&lmicInfo) == OK) {
      LMIC_setSession(lmicInfo.netid, lmicInfo.devaddr, lmicInfo.nwkKey, lmicInfo.artKey);

      LMIC.seqnoUp = t_ramRet.LORA_seqnoUp;
      LMIC.seqnoDn = t_ramRet.LORA_seqnoDown;
    
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1 /*TX_INTERVAL*/), do_send);
      return; 
    }
  } else
    LMIC_infoErase(); 

  TRACE_CrLf("[LMIC] start joining") ;
  LMIC_startJoining() ;

  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(1);
}

void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
  uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

  lmic_time_reference_t lmicTimeReference;

  TRACE("[LMIC] ");

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

  struct tm time_info;
  _rtc_localtime(*pUserUTCTime, &time_info, RTC_4_YEAR_LEAP_YEAR_SUPPORT);

  TRACE_CrLf("utc time is: %02d:%02d:%02d %02d/%02d/%02d", time_info.tm_hour,
                                                      time_info.tm_min,
                                                      time_info.tm_sec,
                                                      time_info.tm_mday,
                                                      time_info.tm_mon + 1,
                                                      time_info.tm_year - 100);

  rtc_write(*pUserUTCTime);
  t_ramRet.timeUpdated = TRUE;
}

void onEvent (ev_t ev) {
  //Serial.print(os_getTime());
  //Serial.print(": ");

  TRACE("[LMIC] ");

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

      LMIC_infoSave(&lmicInfo);
      
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);

      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1 /*TX_INTERVAL*/), do_send);
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
      if (LMIC.txrxFlags & TXRX_ACK)
        TRACE_CrLf("[LMIC] received ack");

      if (LMIC.dataLen) {
        TRACE_CrLf("[LMIC] received %d bytes of payload", LMIC.dataLen);
      }

#if 0            
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
      TRACE_CrLf("[RTC] before standby | timestamp: %d", rtc_read());

      t_ramRet.LORA_seqnoUp = LMIC.seqnoUp;
      t_ramRet.LORA_seqnoDown = LMIC.seqnoDn;
      ramret_save(&t_ramRet);

      //set lowpower

      power_sleep(e_SLEEP_MODE_STANDBY, e_WAKEUP_TYPE_BOTH, TX_INTERVAL * 1000 /*ms*/, WAKEUP_PIN);
#endif 
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

int32_t LMIC_infoSave(t_LmicInfo *pt_lmicInfo) {
  uint16_t i;

  if((pt_lmicInfo == NULL) || (sizeof(t_LmicInfo) > MAX_EEPROM_DATA_SIZE)) {
    TRACE_CrLf("[LMIC] save info arg error");
    return ERROR;
  }

  pt_lmicInfo->magicStart = LMIC_MAGIC;
  pt_lmicInfo->magicEnd = LMIC_MAGIC;

  uint8_t *pt_lmicRawData = (uint8_t*) pt_lmicInfo;

  for(i=0;i<sizeof(t_LmicInfo);i++)
    eeprom_buffered_write_byte(i, pt_lmicRawData[i]);
  
  eeprom_buffer_flush();

  TRACE_CrLf("[LMIC] save info");
  LMIC_printInfo(pt_lmicInfo);

  return OK;
}

int32_t LMIC_infoErase(void) {
  uint16_t i;

  for(i=0;i<sizeof(t_LmicInfo);i++)
    eeprom_buffered_write_byte(i, 0x00);
  
  eeprom_buffer_flush();

  TRACE_CrLf("[LMIC] erase info");

  return OK;
}

int32_t LMIC_infoLoad(t_LmicInfo *pt_lmicInfo) {
  uint16_t i;

  if((pt_lmicInfo == NULL) || (sizeof(t_LmicInfo) > MAX_EEPROM_DATA_SIZE)) {
    TRACE_CrLf("[LMIC] load info arg error");
    return ERROR;
  }

  uint8_t *pt_lmicRawData = (uint8_t*) pt_lmicInfo;

  eeprom_buffer_fill();

  for(i=0;i<sizeof(t_LmicInfo);i++)
    pt_lmicRawData[i] = eeprom_buffered_read_byte(i);

  if((pt_lmicInfo->magicStart != LMIC_MAGIC) || (pt_lmicInfo->magicEnd != LMIC_MAGIC)) {
    TRACE_CrLf("[LMIC] load info magic error");
    return ERROR;
  }

  TRACE_CrLf("[LMIC] load info");
  LMIC_printInfo(pt_lmicInfo);

  return OK;
}

void LMIC_printInfo(t_LmicInfo *pt_lmicInfo) {
  if(pt_lmicInfo == NULL)
    return;
  
  TRACE_CrLf("[LMIC] netid: %d | devaddr %08x", pt_lmicInfo->netid, pt_lmicInfo->devaddr);
  
  TRACE("[LMIC] AppSKey:");
  for (size_t i=0; i<sizeof(pt_lmicInfo->artKey); ++i)
    TRACE(" %02x", pt_lmicInfo->artKey[i]);
  TRACE_CrLf("");
  
  TRACE("[LMIC] NwkSKey:");
  for (size_t i=0; i<sizeof(pt_lmicInfo->nwkKey); ++i)
    TRACE(" %02x", pt_lmicInfo->nwkKey[i]);
  TRACE_CrLf("");
}
#endif  

#if (ONEWIRE_ENABLE == 1)
static void OneWire_getData(t_telemetryData *pt_telemetryData) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  
  if(pt_telemetryData == NULL)
    return;

  TRACE("[1WIRE]");
  
  ow.reset_search();

  if ( !ow.search(pt_telemetryData->temperature1Wire[0].address)) {
    TRACE_CrLf(" No more addresses.");
    ow.reset_search();
    return;
  }
  
  TRACE(" addr: ");
  for(i=0;i<8;i++)
    TRACE("%02x", pt_telemetryData->temperature1Wire[0].address[i]);

  if (OneWire::crc8(pt_telemetryData->temperature1Wire[0].address, 7) != pt_telemetryData->temperature1Wire[0].address[7]) {
      TRACE_CrLf(" | bad crc");
      return;
  }
 
  switch (pt_telemetryData->temperature1Wire[0].address[0]) {
    case 0x10:
      TRACE(" | DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      TRACE(" | DS18B20");
      type_s = 0;
      break;
    case 0x22:
      TRACE(" | DS1822");
      type_s = 0;
      break;
    default:
      TRACE(" | unknown");
      return;
  } 

  ow.reset();
  ow.select(pt_telemetryData->temperature1Wire[0].address);
  ow.write(0x44, 1); // start conversion, with parasite power on at the end
  
  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ow.reset();
  ow.select(pt_telemetryData->temperature1Wire[0].address);    
  ow.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ow.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  pt_telemetryData->nTemperature1Wire = 1;
  pt_telemetryData->temperature1Wire[0].temperature = (float)raw / 16.0;

  TRACE_CrLf(" | temp: %0.2f C", pt_telemetryData->temperature1Wire[0].temperature);

  return;
}
#endif

#if (BMP180_ENABLE == 1)
static void BMP180_setup(void) {
  if (bmp180.begin())
    TRACE_CrLf("[BMP180] init ok");
  else {
    TRACE_CrLf("[BMP180] init ko");
  }
}

static void BMP180_getData(t_telemetryData *pt_telemetryData) {
  char status;
  
  if(pt_telemetryData == NULL)
    return;
    
  TRACE("[BMP180] provided altitude: %0.0f m", ALTITUDE);
  
  status = bmp180.startTemperature();
  if(status != 0) {
    delay(status);

    status = bmp180.getTemperature(pt_telemetryData->temperature);
    if (status != 0)
      TRACE(" | temp: %0.2f C", pt_telemetryData->temperature);
  }  

  status = bmp180.startPressure(3);
  if(status != 0) {
    delay(status);

    status = bmp180.getPressure(pt_telemetryData->pressure, pt_telemetryData->temperature);
    if(status != 0) {
      pt_telemetryData->pressureCompensated = bmp180.sealevel(pt_telemetryData->pressure, ALTITUDE); 
      pt_telemetryData->altitude = bmp180.altitude(pt_telemetryData->pressure, pt_telemetryData->pressureCompensated);
    
      TRACE(" | absolute pressure: %0.2f mb | relative (sea-level) pressure: %0.2f mb | computed altitude: %0.2f m", pt_telemetryData->pressure,
                                                                                                                     pt_telemetryData->pressureCompensated,
                                                                                                                     pt_telemetryData->altitude);
    }                                                                                                                 
  }

  TRACE_CrLf(""); 
}
#endif

#if (HX711_ENABLE == 1)
static void HX711_setup(void) {
  scale.begin(HX711_DT, HX711_CK);

  if(t_ramRet.hx711_calibrated == FALSE) {  
    TRACE_CrLf("[HX711] not initialized");
 
    HX711_calibrate(&telemetryData);
  } else {
    telemetryData.currentOffset = t_ramRet.hx711_offset;
    telemetryData.calibration_factor = t_ramRet.hx711_calibrationFactor;  

    scale.set_scale(telemetryData.calibration_factor / LB2KG);
    scale.set_offset(telemetryData.currentOffset);

    TRACE_CrLf("[HX711] restore calibration | currentOffset: %d | calibration_factor: %0.0f", telemetryData.currentOffset,
                                                                                              telemetryData.calibration_factor);
  }
}

static void HX711_calibrate(t_telemetryData *pt_telemetryData) {
  if(pt_telemetryData == NULL)
    return;
  
  pt_telemetryData->currentOffset = 0;
  pt_telemetryData->calibration_factor = DEFAULT_CALIFACTOR;  
  
  while (digitalRead(BTN_TARE)) {
    TRACE_CrLf("[HX711] start calibration | remove all weight and press <TARE> button");
    digitalWrite(LED_INFO, HIGH);
    delay(150);
    digitalWrite(LED_INFO, LOW);
    delay(150);
  };
  digitalWrite(LED_INFO, HIGH);

  TRACE_CrLf("[HX711] calibration_factor: %0.0f", pt_telemetryData->calibration_factor);
  scale.set_scale(pt_telemetryData->calibration_factor / LB2KG);

  if (!digitalRead(BTN_TARE)) {
    TRACE_CrLf("[HX711] clear scale");
    
    while (!digitalRead(BTN_TARE)); // wait leaves the button
    
    delay(200); //short delay
    
    scale.tare();
    pt_telemetryData->currentOffset = scale.get_offset();
    TRACE_CrLf("[HX711] current offset: %d", pt_telemetryData->currentOffset);

    while (digitalRead(BTN_TARE)) {
      TRACE_CrLf("[HX711] place %0.2f kg and press <TARE> button", CALWEIGHT);
      digitalWrite(LED_INFO, HIGH);
      delay(50);
      digitalWrite(LED_INFO, LOW);
      delay(50);
    };
    digitalWrite(LED_INFO, HIGH);

    TRACE_CrLf("[HX711] please wait calibration");
    
    uint8_t done = false;
    uint8_t flipDirCount = 0;
    int8_t direction = 1;
    uint8_t dirScale = 100;
    double data = abs(scale.get_units());
    double prevData = data;
  
    while (!done) {
      data = abs(scale.get_units());
      TRACE_CrLf("[HX711] data: %0.2f | abs: %0.4f | calibration_factor: %0.0f", data, abs(data - CALWEIGHT), pt_telemetryData->calibration_factor);
      
      if (abs(data - CALWEIGHT) >= 0.01) {
        if (abs(data - CALWEIGHT) < abs(prevData - CALWEIGHT) && direction != 1 && data < CALWEIGHT) {
          direction = 1;
          flipDirCount++;
        } else if (abs(data - CALWEIGHT) >= abs(prevData - CALWEIGHT) && direction != -1 && data > CALWEIGHT) {
          direction = -1;
          flipDirCount++;
        }

        if (flipDirCount > 2) {
          if (dirScale != 1) {
            dirScale = dirScale / 10;
            flipDirCount = 0;
            TRACE_CrLf("[HX711] dirScale: %d", dirScale);
          }
        }
        
        pt_telemetryData->calibration_factor += direction * dirScale;
        scale.set_scale(pt_telemetryData->calibration_factor / LB2KG);
        
        delay(5); //short delay
         
        prevData = data; // keep old data
      } else {
        t_ramRet.hx711_offset = pt_telemetryData->currentOffset;
        t_ramRet.hx711_calibrationFactor = pt_telemetryData->calibration_factor;  
        t_ramRet.hx711_calibrated = TRUE;
        done = true;

        TRACE_CrLf("[HX711] stop calibration | new currentOffset: %d | new calibration_factor: %0.0f", pt_telemetryData->currentOffset,
                                                                                                       pt_telemetryData->calibration_factor);

        digitalWrite(LED_INFO, LOW);
        delay(250);
        digitalWrite(LED_INFO, HIGH);
        delay(250);
        digitalWrite(LED_INFO, LOW);                                                                                            
      }
    } 
  } 

  scale.set_offset(pt_telemetryData->currentOffset);
}  

static void HX711_getData(t_telemetryData *pt_telemetryData) {
  if(pt_telemetryData == NULL)
    return;
  
  //scale.power_up();

  pt_telemetryData->weight = abs(scale.get_units());
  // issue with abs missing at the 4th digit after the dot (bug!!)
  if (0.0000 - pt_telemetryData->weight > 0.0001) 
    pt_telemetryData->weight = 0.00; //reset to zero

  pt_telemetryData->weightAverage = abs(scale.get_units(10));
  // issue with abs missing at the 4th digit after the dot (bug!!)
  if (0.0000 - pt_telemetryData->weightAverage > 0.0001) 
    pt_telemetryData->weightAverage = 0.00; //reset to zero
  
  pt_telemetryData->scaleType = digitalRead(SCALE_TYPE);

  TRACE_CrLf("[HX711] weight oneshot: %0.1f kg | weight average: %0.1f kg | scale type: %d", pt_telemetryData->weight,
                                                                                             pt_telemetryData->weightAverage,
                                                                                             pt_telemetryData->scaleType);

  //scale.power_down();
}
#endif

#if (ANALOG_ENABLE == 1)
static int32_t getExternalAnaRing(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_RING), LL_ADC_RESOLUTION_12B);
}

static int32_t getExternalVbatt(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_VBAT), LL_ADC_RESOLUTION_12B);
}

static int32_t getInternalTemp(int32_t vref_mvolt) { 
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_TEMPERATURE(vref_mvolt, AnalogRead(INTERNAL_TEMP), LL_ADC_RESOLUTION_12B);
}

static int32_t getInternalVbatt(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(INTERNAL_VBAT), LL_ADC_RESOLUTION_12B) * VBAT_RATIO;
}

static int32_t getInternalVrefInternal(int32_t vref_mvolt) {
#if 0
  uint16_t vref_12b;
  int32_t vref_mvolt, vrefint_mvolt;

  
  /* Optionally, for this example purpose, calculate analog reference       */
  /* voltage (Vref+) from ADC conversion of internal voltage reference      */
  /* VrefInt.                                                               */
  /* This voltage should correspond to value of literal "VDDA_APPLI".       */
  /* Note: This calculation can be performed when value of voltage Vref+    */
  /* is unknown in the application.                                         */
  vrefint_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, vref_12b, LL_ADC_RESOLUTION_12B);

  return vrefint_mvolt;
#else
  return 0;
#endif
}

static int32_t getInternalVref(void) {
  uint16_t vref_12b;
  int32_t vref_mvolt, vrefint_mvolt;

  __HAL_RCC_ADC1_CLK_ENABLE();

  vref_12b = AnalogRead(INTERNAL_VREF); 

  vref_mvolt = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_12b, LL_ADC_RESOLUTION_12B);
  //vrefint_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, vref_12b, LL_ADC_RESOLUTION_12B);

  return vref_mvolt;
  //return vrefint_mvolt;
}

static void Analog_getData(t_telemetryData *pt_telemetryData) {
  if(pt_telemetryData == NULL)
    return;
  
  pt_telemetryData->vref = getInternalVref();
  //static int32_t getInternalVrefInternal(int32_t vref_mvolt);
  pt_telemetryData->temperatureInternal = getInternalTemp(pt_telemetryData->vref); 
  pt_telemetryData->vbatt = getInternalVbatt(pt_telemetryData->vref);
  pt_telemetryData->anaRing = getExternalAnaRing(pt_telemetryData->vref);
  pt_telemetryData->vbattExt = getExternalVbatt(pt_telemetryData->vref);

  pt_telemetryData->vbattExt = (float) pt_telemetryData->vbattExt / ((float)R2/((float)R1+(float)R2)); 

  /* 18650 battery percent */
  const float battery_max = 4100; //maximum voltage of battery
  const float battery_min = 3500;  //minimum voltage of battery before shutdown

  pt_telemetryData->vbattExtPercent = ((pt_telemetryData->vbattExt - battery_min) / (battery_max - battery_min)) * 100;
  if (pt_telemetryData->vbattExtPercent > 100)
    pt_telemetryData->vbattExtPercent = 100.0f;

  TRACE_CrLf("[ANALOG] vbattInt: %d mv | vbattExtPercent: %0.2f | vbattExt: %d mv | ring: %d | temp: %d  C", pt_telemetryData->vbatt,
                                                                                                             pt_telemetryData->vbattExtPercent,
                                                                                                             pt_telemetryData->vbattExt,
                                                                                                             pt_telemetryData->anaRing,
                                                                                                             pt_telemetryData->temperatureInternal);
} 
#endif

void Power_setLowConsumption(void) {
#if (ONEWIRE_ENABLE == 1)    
    //OneWire_getData(&telemetryData);
#endif
#if (BMP180_ENABLE == 1)
    //BMP180_getData(&telemetryData);
#endif
#if (HX711_ENABLE == 1)
    //HX711_getData(&telemetryData);
#endif
#if (ANALOG_ENABLE == 1)
    //Analog_getData(&telemetryData);

  //adc1 off

#endif

  //rfm95w

  //gpio
  //input analog
}