/************************************************************************************//**
 *
 *	\file		rtc_external.cpp
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
#include "rtc_external.h"
#include <PCF8563.h>
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
PCF8563 rtc;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		int32_t rtc_external_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_init(time_t *p_startTime) {
  if (!rtc.begin(p_startTime)) {
    TRACE_CrLf("[RTC] not found");
    return ERROR;
  } 

  TRACE_CrLf("[RTC] reg 00 = 0x%02X, reg 01 = 0x%02X", rtc.getStatus1(), rtc.getStatus2());

  //rtc.setRegister(0x00, 0x00);
  //rtc.setRegister(0x01, 0x08);
  //rtc.setRegister(0x0e, 0x00);

  if (rtc.lostPower()) {
    TRACE_CrLf("[RTC] lost power");

    if(true != rtc.set_timestamp(0))
      TRACE_CrLf("[RTC] error set default ts");

    if(true != rtc.clearAlarm())
      TRACE_CrLf("[RTC] error clear alarm");
  }
  
  //rtc.clearAlarm();

  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_external_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_start(uint32_t u32_lsFrequency) {
  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_external_deinit(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_deinit(void) {

  return OK;
}

/************************************************************************************
 *
 *	\fn		time_t rtc_external_read(void) 
 *	\brief 
 *
 ***************************************************************************************/
time_t rtc_external_read(void) {
  time_t now = rtc.now();

  if(now == -1) {
    TRACE_CrLf("[RTC] now error");

    now = rtc.now();
  }

  return now;
}

/************************************************************************************
 *
 *	\fn		void rtc_external_write(time_t timestamp)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_write(time_t timestamp) {

  if(true != rtc.set_timestamp(timestamp))
    return ERROR;  

  return OK;
}
 
/************************************************************************************
 *
 *	\fn		void rtc_external_enableWakeUpRtc(uint32_t u32_sleepTime)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_enableWakeUpRtc(uint32_t u32_sleepTime /* second */) {
  time_t now;
  struct tm time_alarm;
  uint8_t i;

  // modulo 60 check because min alarm is 1 minute 

  i = 5;
  while(true) {
    now = rtc_external_read();

    if(ERROR != now)
      break;

    TRACE_CrLf("[RTC] retry rtc_external_read");

    if(i-- == 0)
      return ERROR;  
  }

  i = 5;
  while(true) {
    if(true == rtc.clearAlarm())
      break;

    TRACE_CrLf("[RTC] retry clearAlarm");

    if(i-- == 0)
      return ERROR;  
  }

  i = 5;
  while(true) {
    if(true == rtc.set_timestampAlarm(now + u32_sleepTime))
      break;
    
    TRACE_CrLf("[RTC] retry set_timestampAlarm");

    if(i-- == 0)
      return ERROR;  
  }

  i = 5;
  while(true) {
    if(true == rtc.get_timeAlarm(&time_alarm))
      break;
    
    TRACE_CrLf("[RTC] retry get_timeAlarm");

    if(i-- == 0)
      return ERROR;  
  }

  TRACE_CrLf("[RTC] alarm, now %d, %02d:%02d day %02d wday %02d, sleep %d s", (uint32_t) now, 
                                                                                    time_alarm.tm_hour,
                                                                                    time_alarm.tm_min,
                                                                                    time_alarm.tm_mday,
                                                                                    time_alarm.tm_wday,
                                                                                    u32_sleepTime);

  return OK;  
}

/************************************************************************************
 *
 *	\fn		void rtc_external_isEnabledWakeUpRtc()
 *	\brief 
 *
 ***************************************************************************************/
bool rtc_external_isEnabledWakeUpRtc(void) {
#if 0
  struct tm timeAlarm;
  rtc.get_timeAlarm(&timeAlarm);
  Serial.printf("Alarm next : %02d:%02d, day %02d, day n%d\r\n", timeAlarm.tm_hour, timeAlarm.tm_min, timeAlarm.tm_mday, timeAlarm.tm_wday);

  Serial.println(rtc.alarmEnabled() ? "Alarm enabled" : "Alarm disabled"); 
  Serial.println(rtc.alarmActive() ? "Alarm active" : "Alarm not active");
#endif

  return rtc.alarmEnabled();  
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_external_isWakeUpRtc(void)
 *	\brief 
 *
 ***************************************************************************************/
bool rtc_external_isWakeUpRtc(void) {
#if 0
  struct tm timeAlarm;
  rtc.get_timeAlarm(&timeAlarm);
  Serial.printf("Alarm next : %02d:%02d, day %02d, day n%d\r\n", timeAlarm.tm_hour, timeAlarm.tm_min, timeAlarm.tm_mday, timeAlarm.tm_wday);

  Serial.println(rtc.alarmEnabled() ? "Alarm enabled" : "Alarm disabled"); 
  Serial.println(rtc.alarmActive() ? "Alarm active" : "Alarm not active");
#endif

  return rtc.alarmActive();  
}

/************************************************************************************
 *
 *	\fn		void rtc_external_desableWakeUpTimer(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_external_disableWakeUpTimer(void) {
  rtc.clearAlarm();

  return OK;
}

/************************************************************************************
 *
 *	\fn		bool rtc_external_isLostPower(void)
 *	\brief 
 *
 ***************************************************************************************/
bool rtc_external_isLostPower(void) {
  return rtc.lostPower();
}

