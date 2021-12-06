/************************************************************************************//**
 *
 *	\file		main.cpp
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
#include "board.h"
#include <Wire.h>
#include "trace.h"
#include "power.h"
#include "system.h"
#include "rtc.h"
#include "ramret.h"
#include "sensor.h"
#include "lora.h"
#include "wdg.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define HW_VERSION 1

#define LORA_ENABLE (1) 
#define WDG_ENABLE (0)

//#define STM32_ID	((u1_t *) 0x1FFFF7E8) //F1
#define STM32_UUID	((uint32_t *) 0x1FFF7A10) //F4

#define MIN_TO_SEC(min) (min*60)

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
typedef enum e_DATA_STATE {
  e_dataStateDefault = 0,
  e_dataStateNotChange,
  e_dataStateQueued,
  e_dataStateSended,
  e_dataStateSendFailed,
  e_dataStateStandby,
};

static e_DATA_STATE e_dataState = e_dataStateDefault;

static bool b_actionButtonPressed = false;
static t_RamRet t_ramRet;
static int32_t startTime = 0;
static uint32_t startDataQueuedTime = 0;

//IWatchdogClass watchdog = IWatchdogClass();

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static int32_t lora_sendDataCallback(uint8_t *payload, uint8_t payloadSize, uint8_t *p_sendSize);
static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data, uint32_t size);
#if (STANDBY_ENABLE == 0)
  static void lora_wakeupCallback(void);
#endif  
static void gotoSleep(void);
static void setLowConsumption(void);
static void actionButtonHandler(void);

/***************************************************************************************
 *
 *	\fn		void setup(void)
 *	\brief 
 *
 ***************************************************************************************/
void setup(void) {
  //system_setBorLevel(OB_BOR_LEVEL1);

  //pinMode(LED_BUILTIN, OUTPUT);
 
  //pinMode(RING, INPUT);

  //digitalWrite(LED_INFO, LOW);
  //pinMode(LED_INFO, OUTPUT);


  //clock debug with mco pin
  //HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

  int32_t powerInit = power_init();
  int32_t rtcInit = rtc_init();
  int32_t rtcStart = rtc_start(0);
  int32_t ramretInit = ramret_init(&t_ramRet);

  trace_init(&t_ramRet);

  { //todo move in trace init function
#if (TRACE_ENABLE == 0)
#if 1
    if(power_isPoweredOn()) {
#else
    if(ramretInit) {
#endif
      uint32_t traceActivationTimeout = 3000;
    
      pinMode(BTN_ACT, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(BTN_ACT), actionButtonHandler, FALLING);
    
      pinMode(LED_INFO, OUTPUT);
      while(traceActivationTimeout) {
        digitalWrite(LED_INFO, HIGH);
        delay(50);
        digitalWrite(LED_INFO, LOW);
        delay(50);
        traceActivationTimeout -= 100;
      };
      digitalWrite(LED_INFO, LOW);
      pinMode(LED_INFO, INPUT);

      detachInterrupt(digitalPinToInterrupt(BTN_ACT));
      pinMode(BTN_ACT, INPUT);


    }
#else
    b_actionButtonPressed = TRUE;
#endif

    if(b_actionButtonPressed) {
      Serial.setTx(UART1_TX);
      Serial.setRx(UART1_RX);

      Serial.begin(115200); // Sets the speed needed to read from the serial port when connected
      while (!Serial); // Loop that only continues once the serial port is active (true)
    } 

    trace_setState(b_actionButtonPressed);
  }

  TRACE_CrLf("################");
  TRACE_CrLf("[BOOT] EcoApi BeeHive Monitor [hw %d, sw %d]", HW_VERSION, SW_REVISION);
  TRACE_CrLf("[BOOT] tx freq %d, cpu uuid %08X%08X%08X", t_ramRet.sendFrequency, STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]);

  /*if(!watchdog.isEnabled()) {
    TRACE_CrLf("[WDG] init ko");
  } else {
    TRACE_CrLf("[WDG] init ok");
  } */

  if(powerInit == STANDBY_RESUMED) {
    TRACE_CrLf("[POWER] resume standby");
  } else if(powerInit != OK) {
    TRACE_CrLf("[POWER] init ko");
  }

  //if(power_isWatchdogReset())
  //  t_ramRet.boot++;

  power_traceState();

  if(rtcInit != OK) {
    TRACE_CrLf("[RTC] init ko");
  } else
    TRACE_CrLf("[RTC] init ok");

  if(rtcStart != OK) {
    TRACE_CrLf("[RTC] start ko");
  } else {
    startTime = rtc_read();
    TRACE_CrLf("[RTC] start ok, timestamp: %d, sleep time %d", startTime, startTime - t_ramRet.lastSendTime);
  }

#if 0 /* test rtc */ 
#if (LORA_ENABLE == 1)
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_receiveDataCallback, &lora_eventCallback);

#if (STANDBY_ENABLE == 0)
  t_ramRet.lastSendTime = 0;
#endif  
#endif

  pinMode(LED_INFO, INPUT_ANALOG);

  int i = 10;
  while(i--) {
    delay(1000);
    TRACE_CrLf("[RTC] timestamp: %d", rtc_read());
  };

  Serial.end();
  pinMode(UART1_TX, INPUT);
  pinMode(UART1_RX, INPUT);

  lora_suspend();

  power_sleep(e_SLEEP_MODE_STANDBY, e_WAKEUP_TYPE_BOTH, 10000 /*ms*/, WAKEUP_PIN);
#endif

  if(ramretInit) { 
    TRACE_CrLf("[RRAM] initialized to default");
  } else
    TRACE_CrLf("[RRAM] restored");

  TRACE_CrLf("[RRAM] size %d, %d x %d bytes", sizeof(t_RamRet), (sizeof(t_RamRet) % 4) ? sizeof(t_RamRet) / sizeof(void*) + 1 : sizeof(t_RamRet) / sizeof(void*), sizeof(void*));  

 

#if (LORA_ENABLE == 1)
  if(t_ramRet.audioSettings.sendData == FALSE) {
    if(sensor_setup(&t_ramRet, FLAG_ALL_WITHOUT_AUDIO) != OK) {
      //todo qos
    } 
  } else {
    if(sensor_setup(&t_ramRet, FLAG_AUDIO_INFO) != OK) {
      //todo qos
     }
  }  

#if (STANDBY_ENABLE == 1)
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_eventCallback);
#else
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_eventCallback, &lora_wakeupCallback);

  t_ramRet.lastSendTime = 0;
#endif
#else
  if(t_ramRet.audioSettings.sendData == FALSE) {
    if(sensor_setup(&t_ramRet, /*FLAG_ALL_WITHOUT_AUDIO | */FLAG_AUDIO_INFO) != OK) {
      //todo qos
    } 
  } 
#endif

  ramret_save(&t_ramRet);
}

/***************************************************************************************
 *
 *	\fn		void loop(void)
 *	\brief 
 *
 ***************************************************************************************/
void loop(void) {
#if (LORA_ENABLE == 1)
  lora_process();

  if((e_dataState == e_dataStateSended) ||
     (e_dataState == e_dataStateSendFailed) || 
     (e_dataState == e_dataStateNotChange)) {

    e_dataState = e_dataStateStandby;

    gotoSleep();
  }

  if(e_dataState == e_dataStateQueued) {
    if((startDataQueuedTime != 0) && (millis() > (startDataQueuedTime + 20000))) {
      //todo qos
      startDataQueuedTime = 0;
      e_dataState = e_dataStateSended; /* force go to sleep state */
    } 
  }
#else
  sensor_getData();

  //delay(1000);

  sensor_suspend();
  delay(5000);
  ramret_save(&t_ramRet);
  system_reset();
#endif
}

#if (LORA_ENABLE == 1)
/***************************************************************************************
 *
 *	\fn		static int32_t lora_sendDataCallback(uint8_t *payload, uint8_t payloadSize, uint8_t *p_sendSize)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t lora_sendDataCallback(uint8_t *payload, uint8_t payloadSize, uint8_t *p_sendSize) {
  uint16_t payloadTmp;
  int32_t ret;
  uint8_t i, sendSize = 0;

  if((payload == NULL) || !payloadSize || (p_sendSize == NULL))
    return ERROR;

  ret = sensor_getData();

  if(ret != OK)
    return ret;

  //add content info first
  payload[sendSize++] = t_ramRet.telemetryData.contentInfo.data[0];
  payload[sendSize++] = t_ramRet.telemetryData.contentInfo.data[1];

  if(t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
    // hardware id : hardware_id (12 bytes)
    memcpy(&payload[sendSize], (uint8_t*) STM32_UUID, 3 * sizeof(uint32_t));
    sendSize += (3 * sizeof(uint32_t));

    // tx interval : measurement_interval_min (1 byte)
    payload[sendSize++] = t_ramRet.sendFrequency;

    // hw version : hardware_version (1 bytes)
    payload[sendSize++] = (uint8_t) HW_VERSION;

    // fw version : firmware_version (2 bytes)
    payload[sendSize++] = highByte((uint16_t) SW_REVISION);
    payload[sendSize++] = lowByte((uint16_t) SW_REVISION);

    // time device : time_device
    //payload[sendSize++] = 
    //payload[sendSize++] = 
    //payload[sendSize++] = 
    //payload[sendSize++] = 

    // total byte 17 + 4 (if time)
  }

  if(t_ramRet.telemetryData.contentInfo.details.boot == TRUE) {
    payload[sendSize++] = t_ramRet.boot;
  }

  if(t_ramRet.telemetryData.contentInfo.details.temperatureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.temperatureOutside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.humidityOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityOutside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.pressureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.pressureOutside; /* 100 exceed 16 bits encoding */
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.weight == TRUE) {
    payloadTmp = t_ramRet.telemetryData.weight * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.vbatt == TRUE) {
    payloadTmp = t_ramRet.telemetryData.vbatt;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);

    payloadTmp = analog_getVBattPercent(t_ramRet.telemetryData.vbatt) * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.humidityInside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityInside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.temperatureInside) {
    for(i=0;i<t_ramRet.telemetryData.contentInfo.details.temperatureInsideCount;i++) {
      payloadTmp = t_ramRet.telemetryData.temperatureInside[i] * 100;
      payload[sendSize++] = highByte(payloadTmp);
      payload[sendSize++] = lowByte(payloadTmp);
    }
  }

  if(t_ramRet.telemetryData.contentInfo.details.custom == TRUE) {

  } 

  if(t_ramRet.telemetryData.contentInfo.details.audio == TRUE) {
    uint8_t size = sensor_getAudioData(&payload[sendSize], payloadSize - sendSize);
    
    if(!size) {
      /*payload[sendSize++] = 0x00; // binOffset
      payload[sendSize++] = 0x00; // binOffset
      payload[sendSize++] = 0x00; // binSize
      payload[sendSize++] = 0x00; // binCount*/
      return ERROR;
    } else
      sendSize += size;
  }

  *p_sendSize = sendSize;

  return OK;
}  

/***************************************************************************************
 *
 *	\fn		static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data, uint32_t size) {
  switch(e_event) {
    case e_TX_RX_ACK:
      TRACE_CrLf("[LORA] received ack tx / rx");
      break;

    case e_RX_DATA: {
      uint8_t *pu8_data = (uint8_t*) pv_data;
      uint32_t i;
  
      if((pu8_data == NULL) || !size)
          break;

      TRACE("[LORA] received %d bytes :", size);
      for(i=0;i<size;i++) {
        TRACE(" %02x", pu8_data[i]);
      }
      TRACE_CrLf("");

      /* todo get audio config and store in flash (count, start,stop,gain,sampling frequency) */

      break;
    }

    case e_TX_DATA_QUEUED:
      TRACE_CrLf("[LORA] data queued (%d bytes)", size);

      startDataQueuedTime = millis();

      e_dataState = e_dataStateQueued;
      break;

    case e_TX_DONE:
      TRACE_CrLf("[LORA] data sended");
      
      if(t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
        t_ramRet.baseInfoSended = TRUE;
        TRACE_CrLf("[LORA] base info sended");
      }  

      if(e_dataState != e_dataStateStandby)
        e_dataState = e_dataStateSended;
      break;
    
    case e_DATA_NOT_CHANGED:
      TRACE_CrLf("[LORA] data not changed");

      e_dataState = e_dataStateNotChange;
      break;
    
    case e_SEND_FAILED:
      TRACE_CrLf("[LORA] send failed");
#if 0
      uint32_t now = rtc_read();

      //uint32_t lapse = now - t_ramRet.lastSendTime;

      if(lapse < (MIN_TO_SEC(t_ramRet.sendFrequency) - 1)) {
        TRACE_CrLf("[ERROR] send time lapse %d, timestamp %d, run %d", lapse, now, runTime);

        t_ramRet.forceNewJoining = TRUE;
        ramret_save(&t_ramRet);

        system_reset();
      } 
#endif

      e_dataState = e_dataStateSendFailed;
      break;

    case e_RX_TIME: {
      if(pv_data == NULL)
        break;
      
      struct tm time_info;
      uint32_t *p_time = (uint32_t*) pv_data;
      
      _rtc_localtime(*p_time, &time_info, RTC_4_YEAR_LEAP_YEAR_SUPPORT);

      TRACE_CrLf("[LORA] utc time is: %02d:%02d:%02d %02d/%02d/%02d", time_info.tm_hour,
                                                      time_info.tm_min,
                                                      time_info.tm_sec,
                                                      time_info.tm_mday,
                                                      time_info.tm_mon + 1,
                                                      time_info.tm_year - 100);

      rtc_write(*p_time);
      t_ramRet.timeUpdated = TRUE;
      break;
    }    
  }
}

#if (STANDBY_ENABLE == 0)
/***************************************************************************************
 *
 *	\fn		static void lora_wakeupCallback(void)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_wakeupCallback(void) {
  e_dataState = e_dataStateDefault;
  
  startTime = rtc_read();

  if(power_isPoweredOn() == FALSE)
    TRACE_CrLf("################");

  TRACE_CrLf("[RTC] send data, timestamp: %d, sleep time %d", startTime, startTime - t_ramRet.lastSendTime);

  if(t_ramRet.audioSettings.sendData == FALSE) {
    if(sensor_setup(&t_ramRet, FLAG_ALL_WITHOUT_AUDIO) != OK) {
      //todo qos
    } 
  } else {
    if(sensor_setup(&t_ramRet, FLAG_AUDIO_INFO) != OK) {
      //todo qos
     }
  } 
}
#endif

/***************************************************************************************
 *
 *	\fn		static void gotoSleep(void)
 *	\brief 
 *
 ***************************************************************************************/
static void gotoSleep(void) {
  uint32_t now = rtc_read();
  uint32_t runTime = now - startTime;
  uint32_t sleepTime;

  if(t_ramRet.audioSettings.sendData == FALSE) { // if not send audio data
    if(++t_ramRet.audioSettings.sendDataCounter >= t_ramRet.audioSettings.sendDataMaxCycle) { // if time to send audio (next cycle)
      t_ramRet.audioSettings.sendData = TRUE;
      sleepTime = MIN_TO_SEC(AUDIO_SEND_DATA_OFFSET); //short suspend for sending audio data
      
      TRACE_CrLf("[RTC] send audio data %d / %d next wakeup", t_ramRet.audioSettings.sendDataCounter, t_ramRet.audioSettings.sendDataMaxCycle);
    } else { 
      sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency) - runTime; // compute real suspend time 
      
      TRACE_CrLf("[RTC] send audio data %d / %d", t_ramRet.audioSettings.sendDataCounter, t_ramRet.audioSettings.sendDataMaxCycle);
    }  
  } else { // just send audio data
    t_ramRet.audioSettings.sendData = FALSE;
    t_ramRet.audioSettings.sendDataCounter = 0;

    sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency) - runTime - MIN_TO_SEC(AUDIO_SEND_DATA_OFFSET); // compute real suspend time with offset send audio data adjustment
  }

  if(sleepTime > MIN_TO_SEC(t_ramRet.sendFrequency))
    sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency);

  t_ramRet.lastSendTime = now;
  
  ramret_save(&t_ramRet);

  TRACE_CrLf("[RTC] before standby, timestamp: %d, sleep time %d, runtime %d", t_ramRet.lastSendTime, sleepTime, runTime);

#if (STANDBY_ENABLE == 1) 
  setLowConsumption();

  power_sleep(e_SLEEP_MODE_STANDBY, e_WAKEUP_TYPE_BOTH, sleepTime * 1000 /*ms*/, WAKEUP_PIN);
#else
  lora_schedule(sleepTime);
#endif 
}
#endif

/***************************************************************************************
 *
 *	\fn		static void setLowConsumption(void)
 *	\brief 
 *
 ***************************************************************************************/
static void setLowConsumption(void) {
  if(sensor_suspend() != OK) {

  }

  pinMode(LED_INFO, INPUT);
  pinMode(BTN_ACT, INPUT);

  Serial.end();
  pinMode(UART1_TX, INPUT);
  pinMode(UART1_RX, INPUT);

#if (LORA_ENABLE == 1)
  if(lora_suspend() != OK) {

  }
#endif
}

/***************************************************************************************
 *
 *	\fn		static void actionButtonHandler(void)
 *	\brief 
 *
 ***************************************************************************************/
static void actionButtonHandler(void) {
  b_actionButtonPressed = true;
}

/***************************************************************************************
 *
 *	\fn		void _Error_Handler(const char *msg, int val)
 *	\brief 
 *
 ***************************************************************************************/
void _Error_Handler(const char *msg, int val) {
  /* User can add his own implementation to report the HAL error return state */
  //core_debug("Error: %s (%i)\n", msg, val);
  TRACE_CrLf("Error: %s (%i)\n", msg, val);
  while (1) {
  }
}