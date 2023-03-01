/************************************************************************************//**
 *
 *	\file		audio_i2s.h
 *
 *	\brief
 *
 *	\date		21 feb. 2023
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __AUDIO_I2S_H_
#define __AUDIO_I2S_H_

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include "ramret.h"
#include "eeprom.h"  
#include "audio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
// define macro to pack structures correctly with both GCC and MSVC compilers

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
#if (USE_EEPROM == 1)
int32_t audio_i2s_setup(t_Eeprom *pt_eeprom);
#else
int32_t audio_i2s_setup(t_RamRet *pt_eeprom);
#endif
int32_t audio_i2s_getData(t_telemetryData *pt_telemetryData);
int32_t audio_i2s_suspend(void);
uint8_t audio_i2s_getResultCount(void);
int32_t audio_i2s_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex);

void I2S_DMA2_Stream0_IRQHandler(void);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);

void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s); 

#ifdef __cplusplus
}
#endif
#endif