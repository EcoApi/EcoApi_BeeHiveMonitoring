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
#include "trace.h"
#include "power.h"
#include "system.h"
#include "rtc.h"
#include "ramret.h"
#include "sensor.h"
#include "lora.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define HW_VERSION 1

#define LORA_ENABLE (1) 
#define WDG_ENABLE (0)

//#define STM32_ID	((u1_t *) 0x1FFFF7E8) //F1
#define STM32_UUID	((uint32_t *) 0x1FFF7A10) //F4

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static bool b_actionButtonPressed = false;
static const uint32_t TX_INTERVAL = 60;
static t_RamRet t_ramRet;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static int32_t lora_sendDataCallback(uint8_t *payload, uint8_t payloadSize, uint8_t *p_sendSize);
static int32_t lora_receiveDataCallback(uint8_t *payload, uint8_t payloadSize);
static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data);
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

  TRACE_CrLf("################");
  TRACE_CrLf("[BOOT] EcoApi BeeHive Monitor [hw %d, sw %d]", HW_VERSION, SW_REVISION);
  TRACE_CrLf("[BOOT] tx freq %d, cpu uuid %08X%08X%08X", TX_INTERVAL, STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]);

  //https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog
  
  if(powerInit == STANDBY_RESUMED) {
    TRACE_CrLf("[POWER] resume standby");
  } else if(powerInit != OK) {
    TRACE_CrLf("[POWER] init ko");
  }

  //if wdg
  t_ramRet.boot++;

  power_traceState();

  if(rtcInit != OK) {
    TRACE_CrLf("[RTC] init ko");
  } else
    TRACE_CrLf("[RTC] init ok");

  if(rtcStart != OK) {
    TRACE_CrLf("[RTC] start ko");
  } else {
    uint32_t now = rtc_read();
    TRACE_CrLf("[RTC] start ok, timestamp: %d, lapse %d", now, now - t_ramRet.LORA_lastSendTime);
  }

  if(ramretInit) { 
    TRACE_CrLf("[RRAM] initialized to default");
  } else
    TRACE_CrLf("[RRAM] restored");

  if(sensor_setup(&t_ramRet) != OK) {

  }  

#if (LORA_ENABLE == 1)
  lora_setup(&t_ramRet, &lora_sendDataCallback, &lora_receiveDataCallback, &lora_eventCallback);

#if (STANDBY_ENABLE == 0)
  t_ramRet.LORA_lastSendTime = 0;
#endif  
#endif

  ramret_save(&t_ramRet);

  pinMode(LED_INFO, INPUT_ANALOG); 
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
#else
  if(sensor_getData() != OK) {

  }
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
  uint8_t i;

  if((payload == NULL) || !payloadSize || (p_sendSize == NULL))
    return ERROR;

  ret = sensor_getData();

  if(ret != OK)
    return ret;

  //add content info first
  payload[*p_sendSize++] = t_ramRet.telemetryData.contentInfo.data[0];
  payload[*p_sendSize++] = t_ramRet.telemetryData.contentInfo.data[1];

  if(t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
    // hardware id : hardware_id (12 bytes)
    memcpy(&payload[*p_sendSize], (uint8_t*) STM32_UUID, 3 * sizeof(uint32_t));
    *p_sendSize += (3 * sizeof(uint32_t));

    // tx interval : measurement_interval_min (1 byte)
    payload[*p_sendSize++] = TX_INTERVAL / 60;

    // hw version : hardware_version (1 bytes)
    payload[*p_sendSize++] = (uint8_t) HW_VERSION;

    // fw version : firmware_version (2 bytes)
    payload[*p_sendSize++] = highByte((uint16_t) SW_REVISION);
    payload[*p_sendSize++] = lowByte((uint16_t) SW_REVISION);

    // time device : time_device
    //payload[*p_sendSize++] = 
    //payload[*p_sendSize++] = 
    //payload[*p_sendSize++] = 
    //payload[*p_sendSize++] = 

    // total byte 17 + 4 (if time)
  }

  if(t_ramRet.telemetryData.contentInfo.details.boot == TRUE) {
    payload[*p_sendSize++] = t_ramRet.boot;
  }

  if(t_ramRet.telemetryData.contentInfo.details.temperatureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.temperatureOutside * 100;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.humidityOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityOutside * 100;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.pressureOutside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.pressureOutside; /* 100 exceed 16 bits encoding */
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.weight == TRUE) {
    payloadTmp = t_ramRet.telemetryData.weight * 100;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.vbatt == TRUE) {
    payloadTmp = t_ramRet.telemetryData.vbatt;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);

    payloadTmp = analog_getVBattPercent(t_ramRet.telemetryData.vbatt) * 100;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }

  if(t_ramRet.telemetryData.contentInfo.details.humidityInside == TRUE) {
    payloadTmp = t_ramRet.telemetryData.humidityInside * 100;
    payload[*p_sendSize++] = highByte(payloadTmp);
    payload[*p_sendSize++] = lowByte(payloadTmp);
  }  

  if(t_ramRet.telemetryData.contentInfo.details.frequency == TRUE) {

  }  

  if(t_ramRet.telemetryData.contentInfo.details.gas == TRUE) {

  }  

  if(t_ramRet.telemetryData.contentInfo.details.temperatureInside) {
    for(i=0;i<t_ramRet.telemetryData.contentInfo.details.temperatureInsideCount;i++) {
      payloadTmp = t_ramRet.telemetryData.temperatureInside[i] * 100;
      payload[*p_sendSize++] = highByte(payloadTmp);
      payload[*p_sendSize++] = lowByte(payloadTmp);
    }

  }

  return OK;
}  

/***************************************************************************************
 *
 *	\fn		static int32_t lora_receiveDataCallback(uint8_t *payload, uint8_t payloadSize)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t lora_receiveDataCallback(uint8_t *payload, uint8_t payloadSize) {
  int i;
  
  if((payload == NULL) || !payloadSize)
    return ERROR;

  TRACE("[LORA] received %d bytes :", payloadSize);
  for(i=0;i<payloadSize;i++) {
    TRACE(" %02x", payload[i]);
  }
  TRACE_CrLf("");

  return OK;
}

/***************************************************************************************
 *
 *	\fn		static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data)
 *	\brief 
 *
 ***************************************************************************************/
static void lora_eventCallback(e_LORA_EVENT e_event, void* pv_data) {
  switch(e_event) {
    case e_TX_COMPLETE:
      if(t_ramRet.telemetryData.contentInfo.details.baseInfo == TRUE) {
        t_ramRet.baseInfoSended = TRUE;
        TRACE_CrLf("[LORA] base info sended");
      }  

      gotoSleep();
      break;
    
    case e_DATA_NOT_CHANGED:
      gotoSleep();
      break;
    
    case e_SEND_FAILED:

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

/***************************************************************************************
 *
 *	\fn		static void gotoSleep(void)
 *	\brief 
 *
 ***************************************************************************************/
static void gotoSleep(void) {
  uint32_t now = rtc_read();

  uint32_t lapse = now - t_ramRet.LORA_lastSendTime;

  if(lapse < (TX_INTERVAL - 1)) {
    TRACE_CrLf("[ERROR] send time lapse %d, timestamp %d", lapse, now);

    t_ramRet.forceNewJoining = TRUE;
    ramret_save(&t_ramRet);

    system_reset();
  } 

  t_ramRet.LORA_lastSendTime = now;
  ramret_save(&t_ramRet);

  TRACE_CrLf("[RTC] before standby, timestamp: %d", t_ramRet.LORA_lastSendTime);

#if (STANDBY_ENABLE == 1) 
  setLowConsumption();

  power_sleep(e_SLEEP_MODE_STANDBY, e_WAKEUP_TYPE_BOTH, TX_INTERVAL * 1000 /*ms*/, WAKEUP_PIN);

#else
  lora_schedule(TX_INTERVAL);
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

#if (LORA_ENABLE == 1)
  if(lora_suspend() != OK) {

  }
#endif

  //rfm95w

  //gpio
  //input analog
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