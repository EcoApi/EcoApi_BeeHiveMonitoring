/************************************************************************************//**
 *
 *	\file		ramret.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __RAMRET_H
#define __RAMRET_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <stdint.h>
#include "board.h"
#include "settings_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef struct t_RamRet_ {
  uint32_t u32_magicStart; 

#if (USE_EEPROM == 0)  
  uint8_t sendFrequency; // minutes
  
  t_hx711Settings hx711Settings;
  t_audioSettings audioSettings;
  t_SensorPresence sensorPresence; 
#endif
  
  t_loraSettings loraSettings;

  t_telemetryData telemetryData; 

  t_dataCommon dataCommon;

  uint32_t u32_magicEnd; 
} t_RamRet; 

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t ramret_init(t_RamRet *pt_ramRet, bool init);
int32_t ramret_save(t_RamRet *pt_ramRet);
uint8_t ramret_isNew(void);
void ramret_clean(t_RamRet *pt_ramRet);

#ifdef __cplusplus
}
#endif
#endif