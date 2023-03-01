/************************************************************************************//**
 *
 *	\file		audio.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __AUDIO_H_
#define __AUDIO_H_

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
int32_t audio_setup(t_Eeprom *pt_eeprom, int32_t vref);
#else
int32_t audio_setup(t_RamRet *pt_eeprom, int32_t vref);
#endif
int32_t audio_getData(t_telemetryData *pt_telemetryData);
int32_t audio_suspend(void);
uint8_t audio_getResultCount(void);
int32_t audio_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex);

void _DMA2_Stream0_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif