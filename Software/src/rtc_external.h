/************************************************************************************//**
 *
 *	\file		rtc_external.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __RTC_EXTERNAL_H
#define __RTC_EXTERNAL_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include "mktime.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
int32_t rtc_external_init(time_t *p_startTime);
int32_t rtc_external_start(uint32_t u32_lsFrequency);
int32_t rtc_external_deinit(void);
time_t rtc_external_read(void);
int32_t rtc_external_write(time_t timestamp);
int32_t rtc_external_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */);
bool rtc_external_isEnabledWakeUpRtc();
bool rtc_external_isWakeUpRtc(void);
int32_t rtc_external_disableWakeUpTimer(void);
bool rtc_external_isLostPower(void);

#ifdef __cplusplus
}
#endif
#endif