/************************************************************************************//**
 *
 *	\file		rtc.cpp
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
#include "rtc_ext.h"
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
 *	\fn		int32_t rtc_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_init(time_t *p_startTime) {
  if (!rtc.begin(p_startTime)) {
    TRACE_CrLf("[RTC] not found");
    return ERROR;
  } 


  //rtc.setRegister(0x00, 0x00);
  //rtc.setRegister(0x01, 0x00);
  //rtc.setRegister(0x0e, 0x00);

  if (rtc.lostPower()) {
    TRACE_CrLf("[RTC] lost power");

    rtc.set_timestamp(0);  
    rtc.clearAlarm();
  }
  
  //rtc.clearAlarm();

  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_start(uint32_t u32_lsFrequency) {
  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_deinit(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_deinit(void) {

  return OK;
}

/************************************************************************************
 *
 *	\fn		time_t rtc_read(void) 
 *	\brief 
 *
 ***************************************************************************************/
//#include <mktime.h>
time_t rtc_read(void) {
#if 0
  struct tm now_;
  time_t seconds;

  rtc.get_time(&now_);

  TRACE_CrLf("[RTC] sec %d", now_.tm_sec);
  TRACE_CrLf("[RTC] min %d", now_.tm_min);
  TRACE_CrLf("[RTC] hour %d", now_.tm_hour);
  TRACE_CrLf("[RTC] mday %d", now_.tm_mday);
  TRACE_CrLf("[RTC] wday %d", now_.tm_wday);
  TRACE_CrLf("[RTC] mon %d", now_.tm_mon);
  TRACE_CrLf("[RTC] year %d", now_.tm_year);
  TRACE_CrLf("[RTC] yday %d", now_.tm_yday);
  TRACE_CrLf("[RTC] isdst %d", now_.tm_isdst);
#endif  
  
  time_t now = rtc.now();

  if(now == -1) {
    TRACE_CrLf("[RTC] now error");

    now = rtc.now();
  }

  return now;
}

/************************************************************************************
 *
 *	\fn		void rtc_write(time_t timestamp)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_write(time_t timestamp) {

  rtc.set_timestamp(0);  

  return OK;
}
 
/************************************************************************************
 *
 *	\fn		int32_t rtc_isEnabled(void) 
 *	\brief 
 *
 ***************************************************************************************/ 
int32_t rtc_isEnabled(void) {
  return TRUE;
}

/************************************************************************************
 *
 *	\fn		void rtc_enableWakeUpRtc(uint32_t u32_sleepTime)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_enableWakeUpRtc(uint32_t u32_sleepTime /* second */) {
  time_t now = rtc_read();

  rtc.clearAlarm();
  
  rtc.set_timestampAlarm(now + u32_sleepTime);

  TRACE_CrLf("[RTC] alarm, now %d, sleep %d", now, u32_sleepTime);

  return OK;  
}

/************************************************************************************
 *
 *	\fn		void rtc_isEnabledWakeUpRtc()
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_isEnabledWakeUpRtc(void) {
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
 *	\fn		void rtc_desableWakeUpTimer(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_disableWakeUpTimer(void) {
  rtc.clearAlarm();

  return OK;
}

/************************************************************************************
 *
 *	\fn		bool rtc_isLostPower(void)
 *	\brief 
 *
 ***************************************************************************************/
bool rtc_isLostPower(void) {

  return rtc.lostPower();
}

