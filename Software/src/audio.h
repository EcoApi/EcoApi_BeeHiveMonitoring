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
PACK(struct FFT_RESULTS_ {
  uint16_t binOffset; /* start bin */
  uint8_t binSize; /* bin count in one outputbin */
  uint8_t binCount; /* bin output count */
  uint16_t values[AUDIO_MAX_BINS];
});
typedef FFT_RESULTS_ FFT_RESULTS;

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
int32_t audio_setup(t_RamRet *pt_ramRet, int32_t vref);
int32_t audio_getData(t_telemetryData *pt_telemetryData);
int32_t audio_suspend(void);
uint8_t audio_getResultCount(void);
int32_t audio_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex);

void _DMA2_Stream0_IRQHandler(void);
void _TIM2_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif