/************************************************************************************//**
 *
 *	\file		rtc_internal.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __RTC_INTERNAL_H
#define __RTC_INTERNAL_H

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
#define RTC_BACKUP_MAX_INDEX      20

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
int32_t rtc_internal_init(void);
int32_t rtc_internal_start(uint32_t u32_lsFrequency);
int32_t rtc_internal_deinit(void);
uint32_t rtc_internal_read(void);
int32_t rtc_internal_write(uint32_t timestamp);
int32_t rtc_internal_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */);
int32_t rtc_internal_isEnabledWakeUpRtc();
int32_t rtc_internal_disableWakeUpTimer(void);
uint32_t rtc_internal_backupRead( uint32_t u32_index );
int32_t rtc_internal_backupWrite( uint32_t u32_index, uint32_t u32_value );

#ifdef __cplusplus
}
#endif
#endif