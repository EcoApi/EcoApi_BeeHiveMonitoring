/************************************************************************************//**
 *
 *	\file		rtc.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __RTC_H
#define __RTC_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include "mktime.h"
#include "rtc_external.h"
#include "rtc_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
int32_t rtc_init(time_t *p_startTime);
int32_t rtc_deinit(void);
time_t rtc_read(void);
int32_t rtc_write(time_t timestamp);
int32_t rtc_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */);
bool rtc_isEnabledWakeUpRtc();
bool rtc_isWakeUpRtc(void);
int32_t rtc_disableWakeUpTimer(void);
bool rtc_isLostPower(void);

#ifdef __cplusplus
}
#endif
#endif