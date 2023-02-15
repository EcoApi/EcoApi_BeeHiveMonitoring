/************************************************************************************/ /**
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
#if (USE_EEPROM == 1)
#include "eeprom.h"
#endif
#include "ramret.h"
#include "sensor.h"
#include "lora.h"
#include "wdg.h"
#include "utils.h"

/***************************************************************************************/
/*	Defines
/***************************************************************************************/
#define HW_VERSION 2

#define TEST_AUDIO (0) 
#define TEST_HARDWARE (0) 
#if (TEST_HARDWARE == 1) || (TEST_AUDIO == 1)
#define LORA_ENABLE (0)
#else
#define LORA_ENABLE (1)
#endif
#define WDG_ENABLE (0)

#define STM32_UUID ((uint32_t *)0x1FFF7A10) // F4

#define MIN_TO_SEC(min) (min * 60)

/***************************************************************************************/
/*	Local variables
/***************************************************************************************/
typedef enum e_DATA_STATE
{
  e_dataStateDefault = 0,
  e_dataStateNotChange,
  e_dataStateQueued,
  e_dataStateSended,
  e_dataStateSendFailed,
  e_dataStateStandby,
};

static e_DATA_STATE e_dataState = e_dataStateDefault;

#if (USE_EEPROM == 1)
static t_Eeprom t_eeprom;
#endif

static t_RamRet t_ramRet;
static time_t startTime = 0;
static uint32_t startDataQueuedTime = 0;

// IWatchdogClass watchdog = IWatchdogClass();

/***************************************************************************************/
/*	Local Functions prototypes
/***************************************************************************************/
static int32_t lora_sendDataCallback(uint8_t *payload, uint8_t payloadSize, uint8_t *p_sendSize);
static void lora_eventCallback(e_LORA_EVENT e_event, void *pv_data, uint32_t size);
#if (STANDBY_ENABLE == 0)
static void lora_wakeupCallback(void);
#endif
static void gotoSleep(void);
static void setLowConsumption(void);
#if (TEST_HARDWARE == 1)
static void test_hardware(void);
#endif

/***************************************************************************************
 *
 *	\fn		void setup(void)
 *	\brief
 *
 ***************************************************************************************/
void setup(void) {
  // system_setBorLevel(OB_BOR_LEVEL1);

  digitalWrite(LED_INFO, LOW);
  pinMode(LED_INFO, OUTPUT);

  pinMode(MIC_ANA, INPUT_ANALOG);

  // clock debug with mco pin
  // HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

  SET_3V3_EXTERNAL(true);

  int32_t powerInit = power_init();

  trace_init(&Serial);

  if(power_isPoweredOn())
    delay(1000);

  Wire.setSCL(I2C1_SCL);
  Wire.setSDA(I2C1_SDA);

  Wire.setClock(400000);
  Wire.begin();

#if (USE_EEPROM == 1)
  int32_t eepromInit = eeprom_init(&t_eeprom);
#endif

  int32_t rtcInit = rtc_init(&startTime);

  int32_t ramRetInit = ramret_init(&t_ramRet, rtc_isLostPower());

  TRACE_CrLf("[BOOT] EcoApi BeeHive Monitor [hw %d, sw %d]", HW_VERSION, SW_REVISION);
#if (USE_EEPROM == 1)
  TRACE_CrLf("[BOOT] tx freq %d, cpu uuid %08X%08X%08X", t_eeprom.sendFrequency, STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]);
#else
  TRACE_CrLf("[BOOT] tx freq %d, cpu uuid %08X%08X%08X", t_ramRet.sendFrequency, STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]);
#endif

  /*if(!watchdog.isEnabled()) {
    TRACE_CrLf("[WDG] init ko");
  } else {
    TRACE_CrLf("[WDG] init ok");
  } */

  if (powerInit == STANDBY_RESUMED) {
    TRACE_CrLf("[POWER] resume standby");
  } else if (powerInit != OK) {
    TRACE_CrLf("[POWER] init ko");
  }

  // if(power_isWatchdogReset())
  //   t_ramRet.boot++;

  power_traceState();

#if (USE_EEPROM == 1)
  if (OK != eepromInit) {
    TRACE_CrLf("[EEPROM] error");
    delay(1000);
    system_reset();
  } else {
    if (eeprom_isNew())
      TRACE_CrLf("[EEPROM] initialized to default");
    else
      TRACE_CrLf("[EEPROM] restored");
  }
#endif

  if (OK != rtcInit) {
    TRACE_CrLf("[RTC] error %d", rtcInit);
    delay(1000);
    system_reset();
  } else {
    if (rtc_isLostPower())
      TRACE_CrLf("[RTC] init");
    else {
      TRACE_CrLf("[RTC] ok, timestamp: %d, sleep time %d, last send time %d, last update time %d", (uint32_t) startTime,
                                                                                                   (uint32_t) startTime - t_ramRet.lastSendTime,
                                                                                                   t_ramRet.lastSendTime,
                                                                                                   t_ramRet.lastUpdateTime);

      /*TRACE_CrLf("rtc is wkup %d, rtc is en %d, is p rst %d, is sw rst %d, is pwr on %d, pwr sb %d", rtc_isWakeUpRtc(),
                                                                                       rtc_isEnabledWakeUpRtc(),
                                                                                       power_isPinReset(),
                                                                                       power_isSoftwareReset(),
                                                                                       power_isPoweredOn(),
                                                                                       powerInit == STANDBY_RESUMED);*/

      if((FALSE == rtc_isWakeUpRtc()) &&
         (TRUE == rtc_isEnabledWakeUpRtc()) &&
         (TRUE == power_isPoweredOn()) || (TRUE == power_isSoftwareReset()) || (powerInit == STANDBY_RESUMED))
        t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn = TRUE;
      else
        t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn = FALSE;

      if(t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn) {
        TRACE_CrLf("[POWER] motion or power on detected, last time %d", t_ramRet.lastSendMotionOrPowerTime);

        if((t_ramRet.lastSendMotionOrPowerTime + DEFAULT_MOTION_DETECT_PERIOD) > startTime) {
#if (TEST_HARDWARE == 0) && (TEST_AUDIO == 0)
          TRACE_CrLf("[POWER] go standby");
         
          //lora_suspend();

          pinMode(LED_INFO, INPUT_ANALOG);

          pinMode(MIC_ANA, INPUT_ANALOG);

          SET_3V3_EXTERNAL(false);

          trace_setState(false);  

          Wire.end();

          power_sleep(e_SLEEP_MODE_STANDBY, e_WAKEUP_TYPE_RTC, DEFAULT_STANDBY_MOTION_DETECT_TIMEOUT, WAKEUP_PIN);
#endif          
        }  
      } 
    }    
  }

  if (OK != ramRetInit) {
    TRACE_CrLf("[RRAM] error");
    delay(1000);
    system_reset();
  } else {
    if (ramret_isNew())
      TRACE_CrLf("[RRAM] initialized to default");
    else {
      if((t_ramRet.lastUpdateTime + DEFAULT_UPDATE_TIME_PERIOD) < startTime) {
        t_ramRet.lastUpdateTime = startTime;
        t_ramRet.timeUpdated = FALSE;

        TRACE_CrLf("[RRAM] restored, ask update time");
      } else
        TRACE_CrLf("[RRAM] restored");
    }  
  }

#if (TEST_AUDIO == 1)
  uint32_t vRef = analog_getInternalVref();

  audio_setup(&t_ramRet, vRef);
#elif (TEST_HARDWARE == 1)
  test_hardware();
#else
#if (LORA_ENABLE == 1)
  if (t_ramRet.audioSettings.sendData == FALSE) {
    if (sensor_setup(&t_ramRet, &t_ramRet, FLAG_ALL_WITHOUT_AUDIO) != OK) {
      // todo qos
    }
  } else {
    if (sensor_setup(&t_ramRet, &t_ramRet, FLAG_AUDIO_INFO) != OK) {
      // todo qos
    }
  }

#if (STANDBY_ENABLE == 1)
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_eventCallback);
#else
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_eventCallback, &lora_wakeupCallback);

  t_ramRet.lastSendTime = 0;
#endif
#else
  // if(t_ramRet.audioSettings.sendData == FALSE) {
#if (USE_EEPROM == 1)
  if (sensor_setup(&t_eeprom, &t_ramRet, FLAG_ALL_WITHOUT_AUDIO /*| FLAG_AUDIO_INFO*/) != OK) {
#else
  if (sensor_setup(&t_ramRet, &t_ramRet, FLAG_ALL_WITHOUT_AUDIO /*| FLAG_AUDIO_INFO*/) != OK) {
#endif

    // todo qos
  }
  //}
#endif

  if (OK != ramret_save(&t_ramRet)) {
    delay(1000);
    system_reset();
  }
#endif

  //display_mallinfo();
}

/***************************************************************************************
 *
 *	\fn		void loop(void)
 *	\brief
 *
 ***************************************************************************************/
void loop(void) {
#if (TEST_AUDIO == 1)
  t_telemetryData telemetryData; 

  audio_getData(&telemetryData);

  while(true);
#else
#if (LORA_ENABLE == 1)
  lora_process();

  if ((e_dataState == e_dataStateSended) ||
      (e_dataState == e_dataStateSendFailed) ||
      (e_dataState == e_dataStateNotChange)) {
    e_dataState = e_dataStateStandby;

    gotoSleep();
  }

  if (e_dataState == e_dataStateQueued) {
    if ((startDataQueuedTime != 0) && (millis() > (startDataQueuedTime + 20000))) {
      // todo qos
      startDataQueuedTime = 0;
      e_dataState = e_dataStateSended; /* force go to sleep state */
    }
  }
#else
  sensor_getData();

  // delay(1000);

  // sensor_suspend();
  // delay(5000);

  if(t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn)
    t_ramRet.lastSendMotionOrPowerTime = rtc_read();

  ramret_save(&t_ramRet);

  power_sleep(e_SLEEP_MODE_OFF, e_WAKEUP_TYPE_RTC, 120 /*seconde*/, WAKEUP_PIN);
#endif
#endif

  //display_mallinfo();
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

  if ((payload == NULL) || !payloadSize || (p_sendSize == NULL))
    return ERROR;

  ret = sensor_getData();

  if (ret != OK)
    return ret;

  // add content info first
  payload[sendSize++] = t_ramRet.telemetryData.contentInfo.data[0];
  payload[sendSize++] = t_ramRet.telemetryData.contentInfo.data[1];

  if (t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
    // hardware id : hardware_id (12 bytes)
    memcpy(&payload[sendSize], (uint8_t *)STM32_UUID, 3 * sizeof(uint32_t));
    sendSize += (3 * sizeof(uint32_t));

    // tx interval : measurement_interval_min (1 byte)
    payload[sendSize++] = t_ramRet.sendFrequency;

    // hw version : hardware_version (1 bytes)
    payload[sendSize++] = (uint8_t)HW_VERSION;

    // fw version : firmware_version (2 bytes)
    payload[sendSize++] = highByte((uint16_t)SW_REVISION);
    payload[sendSize++] = lowByte((uint16_t)SW_REVISION);

    // time device : time_device
    // payload[sendSize++] =
    // payload[sendSize++] =
    // payload[sendSize++] =
    // payload[sendSize++] =

    // total byte 17 + 4 (if time)
  }

  if (t_ramRet.telemetryData.contentInfo.details.boot == TRUE) {
    payload[sendSize++] = t_ramRet.boot;
  }

  if (t_ramRet.telemetryData.contentInfo.details.temperatureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.temperatureOutside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.humidityOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityOutside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.pressureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.pressureOutside; /* 100 exceed 16 bits encoding */
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.weight == TRUE) {
    payloadTmp = t_ramRet.telemetryData.weight * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.vbatt == TRUE) {
    payloadTmp = t_ramRet.telemetryData.vbatt;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);

    payloadTmp = analog_getVBattPercent(t_ramRet.telemetryData.vbatt) * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.humidityInside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityInside * 100;
    payload[sendSize++] = highByte(payloadTmp);
    payload[sendSize++] = lowByte(payloadTmp);
  }

  if (t_ramRet.telemetryData.contentInfo.details.temperatureInside) {
    for (i = 0; i < t_ramRet.telemetryData.contentInfo.details.temperatureInsideCount; i++) {
      payloadTmp = t_ramRet.telemetryData.temperatureInside[i] * 100;
      payload[sendSize++] = highByte(payloadTmp);
      payload[sendSize++] = lowByte(payloadTmp);
    }
  }

  if (t_ramRet.telemetryData.contentInfo.details.custom == TRUE) {
  }

  if (t_ramRet.telemetryData.contentInfo.details.audio == TRUE) {
    uint8_t size = sensor_getAudioData(&payload[sendSize], payloadSize - sendSize);

    if (!size) {
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
static void lora_eventCallback(e_LORA_EVENT e_event, void *pv_data, uint32_t size) {
  switch (e_event) {
  case e_TX_RX_ACK:
    TRACE_CrLf("[LORA] received ack tx / rx");
    break;

  case e_RX_DATA: {
    uint8_t *pu8_data = (uint8_t *)pv_data;
    uint32_t i;

    if ((pu8_data == NULL) || !size)
      break;

    TRACE("[LORA] received %d bytes :", size);
    for (i = 0; i < size; i++) {
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

    if (t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
      t_ramRet.baseInfoSended = TRUE;
      TRACE_CrLf("[LORA] base info sended");
    }

    if (e_dataState != e_dataStateStandby)
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
    if (pv_data == NULL)
      break;

    struct tm time_info;
    uint32_t *p_time = (uint32_t *)pv_data;

    _rtc_localtime(*p_time, &time_info, RTC_4_YEAR_LEAP_YEAR_SUPPORT);

    TRACE_CrLf("[LORA] utc time is: %02d:%02d:%02d %02d/%02d/%02d, ts %d", time_info.tm_hour,
               time_info.tm_min,
               time_info.tm_sec,
               time_info.tm_mday,
               time_info.tm_mon + 1,
               time_info.tm_year - 100,
               *p_time);

    if(OK == rtc_write(*p_time)) {
      t_ramRet.timeUpdated = TRUE;
      t_ramRet.lastUpdateTime = *p_time;
    }
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

  if (power_isPoweredOn() == FALSE)
    TRACE_CrLf("################");

  TRACE_CrLf("[RTC] send data, timestamp: %d, sleep time %d", startTime, startTime - t_ramRet.lastSendTime);

  if (t_ramRet.audioSettings.sendData == FALSE) {
    if (sensor_setup(&t_ramRet, &t_ramRet, FLAG_ALL_WITHOUT_AUDIO) != OK) {
      // todo qos
    }
  } else {
    if (sensor_setup(&t_ramRet, &t_ramRet, FLAG_AUDIO_INFO) != OK) {
      // todo qos
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
  uint32_t runTime = constrain(now - startTime, 0, 0xffffffff);
  uint32_t sleepTime = 60;

  if (t_ramRet.audioSettings.sendData == FALSE) { // if not send audio data
    if (++t_ramRet.audioSettings.sendDataCounter >= t_ramRet.audioSettings.sendDataMaxCycle) { // if time to send audio (next cycle)
      t_ramRet.audioSettings.sendData = TRUE;
      sleepTime = MIN_TO_SEC(AUDIO_SEND_DATA_OFFSET); // short suspend for sending audio data

      TRACE_CrLf("[RTC] send audio data %d / %d next wakeup", t_ramRet.audioSettings.sendDataCounter, t_ramRet.audioSettings.sendDataMaxCycle);
    } else {
      sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency) /*- runTime*/; // compute real suspend time

      TRACE_CrLf("[RTC] send audio data %d / %d", t_ramRet.audioSettings.sendDataCounter, t_ramRet.audioSettings.sendDataMaxCycle);
    }
  } else { // just send audio data
    t_ramRet.audioSettings.sendData = FALSE;
    t_ramRet.audioSettings.sendDataCounter = 0;

    sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency) /*- runTime*/ - MIN_TO_SEC(AUDIO_SEND_DATA_OFFSET); // compute real suspend time with offset send audio data adjustment
  }

  if (sleepTime > MIN_TO_SEC(t_ramRet.sendFrequency))
    sleepTime = MIN_TO_SEC(t_ramRet.sendFrequency);

  t_ramRet.lastSendTime = now;

  if(t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn)
    t_ramRet.lastSendMotionOrPowerTime = now;

  sleepTime = constrain(sleepTime, MIN_SLEEP_PERIOD, MAX_SLEEP_PERIOD);

  TRACE_CrLf("[RTC] before standby, timestamp: %d, sleep time %d s, runtime %d s", t_ramRet.lastSendTime, sleepTime, runTime);

  ramret_save(&t_ramRet);

#if (STANDBY_ENABLE == 1)
  //setLowConsumption();

  power_sleep(e_SLEEP_MODE_OFF, e_WAKEUP_TYPE_RTC, sleepTime /*min*/, WAKEUP_PIN);
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
  if (sensor_suspend() != OK) {
  }

  pinMode(LED_INFO, INPUT);

  Serial.end();
  pinMode(UART1_TX, INPUT);
  pinMode(UART1_RX, INPUT);

#if (LORA_ENABLE == 1)
  if (lora_suspend() != OK) {
  }
#endif
}

/***************************************************************************************
 *
 *	\fn		void _Error_Handler(const char *msg, int val)
 *	\brief
 *
 ***************************************************************************************/
void _Error_Handler(const char *msg, int val) {
  /* User can add his own implementation to report the HAL error return state */
  // core_debug("Error: %s (%i)\n", msg, val);
  TRACE_CrLf("Error: %s (%i)\n", msg, val);
  while (1) { }
}

#if (TEST_HARDWARE == 1)
/***************************************************************************************
 *
 *	\fn		static void test_hardware(void)
 *	\brief
 *
 ***************************************************************************************/
static void test_hardware(void) {
  t_telemetryData telemetryData;
  int i;

#if 0
  i = 10;
  while(i--) {
    delay(1000);
    TRACE_CrLf("[RTC] timestamp: %d", rtc_read());
    
    digitalToggle(LED_INFO);
  };
#endif

#if 0
  i = 3;
  while(i--) {
 // if(rtc_isLostPower()) {
    time_t time_write = 1676325389; //13/02/2023 22:56:29
    time_t time_read;

    struct tm time_info_write;
    struct tm time_info_read;

    if( 0 == _rtc_localtime(time_write, &time_info_write, RTC_4_YEAR_LEAP_YEAR_SUPPORT)) 
      TRACE_CrLf("[RTC] error local time");

    TRACE_CrLf("[RTC] utc time write is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_write.tm_hour,
                time_info_write.tm_min,
                time_info_write.tm_sec,
                time_info_write.tm_mday,
                time_info_write.tm_mon + 1,
                time_info_write.tm_year - 100,
                time_write);

    if(OK != rtc_write(time_write))
      TRACE_CrLf("[RTC] error write");

    delay(1000);

    time_read = rtc_read();

    if(0 == _rtc_localtime(time_read, &time_info_read, RTC_FULL_LEAP_YEAR_SUPPORT))
      TRACE_CrLf("[RTC] error make time");
    
    TRACE_CrLf("[RTC] utc time read is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_read.tm_hour,
                time_info_read.tm_min,
                time_info_read.tm_sec,
                time_info_read.tm_mday,
                time_info_read.tm_mon + 1,
                time_info_read.tm_year - 100,
                time_read);

    delay(1000);

    time_read = rtc_read();

    if(0 == _rtc_localtime(time_read, &time_info_read, RTC_FULL_LEAP_YEAR_SUPPORT))
      TRACE_CrLf("[RTC] error make time");
    
    TRACE_CrLf("[RTC] utc time read is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_write.tm_hour,
                time_info_read.tm_min,
                time_info_read.tm_sec,
                time_info_read.tm_mday,
                time_info_read.tm_mon + 1,
                time_info_read.tm_year - 100,
                time_read); 

    delay(1000);                       
  //}
  };

  i = 3;
  while(i--) {
 // if(rtc_isLostPower()) {
    time_t time_write = 1676359293; //14/02/2023 08:21:33
    time_t time_read;

    struct tm time_info_write;
    struct tm time_info_read;
  
    if( 0 == _rtc_localtime(time_write, &time_info_write, RTC_4_YEAR_LEAP_YEAR_SUPPORT)) 
      TRACE_CrLf("[RTC] error local time");

    TRACE_CrLf("[RTC] utc time write is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_write.tm_hour,
                time_info_write.tm_min,
                time_info_write.tm_sec,
                time_info_write.tm_mday,
                time_info_write.tm_mon + 1,
                time_info_write.tm_year - 100,
                time_write);

    if(OK != rtc_write(time_write))
      TRACE_CrLf("[RTC] error write");

    delay(1000);

    time_read = rtc_read();

    if(0 == _rtc_localtime(time_read, &time_info_read, RTC_FULL_LEAP_YEAR_SUPPORT))
      TRACE_CrLf("[RTC] error make time");
    
    TRACE_CrLf("[RTC] utc time read is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_read.tm_hour,
                time_info_read.tm_min,
                time_info_read.tm_sec,
                time_info_read.tm_mday,
                time_info_read.tm_mon + 1,
                time_info_read.tm_year - 100,
                time_read);

    delay(1000);

    time_read = rtc_read();

    if(0 == _rtc_localtime(time_read, &time_info_read, RTC_FULL_LEAP_YEAR_SUPPORT))
      TRACE_CrLf("[RTC] error make time");
    
    TRACE_CrLf("[RTC] utc time read is: %02d:%02d:%02d %02d/%02d/%02d from ts %d", time_info_read.tm_hour,
                time_info_read.tm_min,
                time_info_read.tm_sec,
                time_info_read.tm_mday,
                time_info_read.tm_mon + 1,
                time_info_read.tm_year - 100,
                time_read);  

    delay(1000);                      
  //}
  };
#endif

  #if 1 /* test vbatt */
  pinMode(WAKEUP, INPUT_FLOATING);
  //rtc_disableWakeUpTimer();

  analog_setup(&t_ramRet, 0);

  i = 3;
  while(i--) {
    delay(500);
    analog_getData(&telemetryData);
    //TRACE_CrLf("WAKEUP pin %d", digitalRead(WAKEUP));
  };
  
  //power_sleep(e_SLEEP_MODE_OFF, e_WAKEUP_TYPE_RTC, 180 /*minute*/, WAKEUP_PIN);

  //NVIC_SystemReset();
#endif

#if 1 /* test rtc */
#if (LORA_ENABLE == 1)
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_eventCallback);

#if (STANDBY_ENABLE == 0)
  t_ramRet.lastSendTime = 0;
#endif
#endif
 
  //delay(2000);

  //power_sleep(e_SLEEP_MODE_OFF, e_WAKEUP_TYPE_RTC, 60 /*minute*/, WAKEUP_PIN);

#if (USE_EEPROM == 1)
  hx711_setup(&t_eeprom, &t_ramRet);
#else
  hx711_setup(&t_ramRet, &t_ramRet);
#endif

  sht3x_setup(&t_ramRet);

  bmp180_setup(&t_ramRet);

#if 1
  i = 100;
  while(i--) {
    delay(500);
    
    hx711_getData(&telemetryData);
    
    digitalToggle(LED_INFO);
  };
#endif

  sht3x_getData(&telemetryData);

  bmp180_getData(&telemetryData);

  

  sht3x_suspend();

  bmp180_suspend();

  //hx711_suspend();

#if (LORA_ENABLE == 1)
  lora_suspend();
#endif

  if(t_ramRet.telemetryData.contentInfo.details.motionDetectionOrPowerOn) {
    t_ramRet.lastSendMotionOrPowerTime = rtc_read();
    TRACE_CrLf("[POWER] last motion %d", t_ramRet.lastSendMotionOrPowerTime);
  }

  // SET_3V3_EXTERNAL(false);

  ramret_save(&t_ramRet);

  // Wire.end();

  // digitalWrite(PWR_OFF, HIGH);
  // pinMode(PWR_OFF, OUTPUT);

  // while(TRUE) {};

  power_sleep(e_SLEEP_MODE_OFF, e_WAKEUP_TYPE_RTC, 60 /*minute*/, WAKEUP_PIN);

  //system_reset();
  while(TRUE) {};
#endif
}
#endif