/************************************************************************************//**
 *
 *	\file		power.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __POWER_H
#define __POWER_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
#define STANDBY_RESUMED (OK+1)
#define STANDBY_NOT_RESUMED (ERROR-1)

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef enum {
  e_WAKEUP_TYPE_RTC = 0,
  e_WAKEUP_TYPE_INPUT,
  e_WAKEUP_TYPE_BOTH,

} e_WAKEUP_TYPE;

typedef enum {
  e_SLEEP_MODE_STANDBY = 0,
  e_SLEEP_MODE_SLEEP,
  e_SLEEP_MODE_STOP,

} e_SLEEP_MODE;

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t power_sleep(e_SLEEP_MODE e_mode, e_WAKEUP_TYPE e_wakeupType, uint32_t u32_sleepTime, uint32_t u32_wakeupPin);
int32_t power_resumeSleep(e_SLEEP_MODE e_mode, e_WAKEUP_TYPE e_wakeupType, uint32_t u32_wakeupPin);
int32_t power_init(void);
int32_t power_getLastState(void);
uint8_t power_isPoweredOn(void);
uint8_t power_isWatchdogReset(void);
void power_traceState(void);

#ifdef __cplusplus
}
#endif
#endif