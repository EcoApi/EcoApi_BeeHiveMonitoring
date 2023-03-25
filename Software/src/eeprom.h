/************************************************************************************//**
 *
 *	\file		eeprom.h
 *
 *	\brief
 *
 *	\date		21 jan. 2022
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __EEPROM_H
#define __EEPROM_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <stdint.h>
#include "board.h"
#include "settings_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (USE_EEPROM == 1)

/***************************************************************************************/
/* Define
/***************************************************************************************/

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef struct t_Eeprom_ {
  uint32_t magicStart;

  uint8_t sendFrequency; // minutes

  t_hx711Settings hx711Settings;
  t_audioSettings audioSettings;

  t_SensorPresence sensorPresence;

  uint32_t magicEnd;
} t_Eeprom;

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t eeprom_init(t_Eeprom *pt_eeprom);
int32_t eeprom_save(t_Eeprom *pt_eeprom);
bool eeprom_isNew(void);
#endif

#ifdef __cplusplus
}
#endif
#endif