/************************************************************************************//**
 *
 *	\file		audio_adc.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __AUDIO_ADC_H_
#define __AUDIO_ADC_H_

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
int32_t audio_adc_setup(t_Eeprom *pt_eeprom, int32_t vref);
#else
int32_t audio_adc_setup(t_RamRet *pt_eeprom, int32_t vref);
#endif
int32_t audio_adc_getData(t_telemetryData *pt_telemetryData);
int32_t audio_adc_suspend(void);
uint8_t audio_adc_getResultCount(void);
int32_t audio_adc_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex);

void ADC_DMA2_Stream0_IRQHandler(void);
void _TIM2_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif