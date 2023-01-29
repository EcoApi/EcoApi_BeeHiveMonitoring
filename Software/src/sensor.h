/************************************************************************************//**
 *
 *	\file		sensor.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __SENSOR_H_
#define __SENSOR_H_

/***************************************************************************************/
/*	Includes																		                                      
/***************************************************************************************/
#include "board.h"
#if (USE_EEPROM == 1)
#include "eeprom.h"
#endif
#include "ramret.h"
#include "analog.h"
#include "hx711.h"
#include "bmp180.h"
#include "sht3x.h"
#include "audio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
#define FLAG_NONE           (0x0000)
#define FLAG_BASE_INFO      (0x0001)
#define FLAG_BOOT_INFO      (0x0002)  
#define FLAG_OUT_INFO       (0x0004)
#define FLAG_WEIGHT_INFO    (0x0010)
#define FLAG_BATT_INFO      (0x0020)
#define FLAG_IN_INFO        (0x0040)
#define FLAG_AUDIO_INFO     (0x0080)
#define FLAG_CUSTOM_INFO    (0x0100) /* gas or other i2c slave with my own registers definition */

#define FLAG_ALL_WITHOUT_AUDIO (FLAG_BASE_INFO | FLAG_BOOT_INFO | FLAG_OUT_INFO | FLAG_WEIGHT_INFO | FLAG_BATT_INFO | FLAG_IN_INFO /* | FLAG_CUSTOM_INFO*/)


/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
#if (USE_EEPROM == 1)
int32_t sensor_setup(t_Eeprom *pt_eeprom, t_RamRet *pt_ramRet, uint16_t dataFlag);
#else
int32_t sensor_setup(t_RamRet *pt_eeprom, t_RamRet *pt_ramRet, uint16_t dataFlag);
#endif
int32_t sensor_getData(void);
int8_t sensor_getAudioData(uint8_t *p_data, uint8_t dataSize);
int32_t sensor_suspend(void);

#ifdef __cplusplus
}
#endif

#endif