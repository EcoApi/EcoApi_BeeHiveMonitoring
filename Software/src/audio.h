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
typedef enum e_AUDIO_TYPE_ {
    e_AUDIO_TYPE_NONE = 0,
    e_AUDIO_TYPE_I2S,
    e_AUDIO_TYPE_ADC,
} e_AUDIO_TYPE;

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
e_AUDIO_TYPE audio_detectType(void);
#if (USE_EEPROM == 1)
int32_t audio_setup(t_Eeprom *pt_eeprom, int32_t vref, e_AUDIO_TYPE e_audioType);
#else
int32_t audio_setup(t_RamRet *pt_eeprom, int32_t vref, e_AUDIO_TYPE e_audioType);
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