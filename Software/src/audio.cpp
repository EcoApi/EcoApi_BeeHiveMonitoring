/************************************************************************************//**
 *
 *	\file		audio_adc.cpp
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes				
/***************************************************************************************/
#include <string.h>
#include <stdio.h>

#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include <math.h>

#include "audio.h"
#include "audio_i2s.h"
#include "audio_adc.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/


/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
#if (USE_EEPROM == 1)
static t_Eeprom *pt_eeprom_ = NULL;
#else
static t_RamRet *pt_eeprom_ = NULL;
#endif

static int32_t vRef_ = 3300; /* mV */

bool i2s_used = TRUE;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void _DMA2_Stream0_IRQHandler(void) {
  if(i2s_used)
    I2S_DMA2_Stream0_IRQHandler();
  else
    ADC_DMA2_Stream0_IRQHandler();
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_setup(t_Eeprom *pt_eeprom)
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
int32_t audio_setup(t_Eeprom *pt_eeprom, int32_t vRef) {
#else
int32_t audio_setup(t_RamRet *pt_eeprom, int32_t vRef) {
#endif  
  int32_t ret;
  
  if(pt_eeprom == NULL)
    return ERROR;

  pt_eeprom_ = pt_eeprom;
  vRef_ = vRef;

  //detected i2s or not and store 

  if(i2s_used)
    ret = audio_i2s_setup(pt_eeprom);
  else  
    ret = audio_adc_setup(pt_eeprom, vRef);
  
  if(ret != OK)
    TRACE_CrLf("[AUDIO] init ko"); 

  return ret;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_getData(t_telemetryData *pt_telemetryData) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_getData(t_telemetryData *pt_telemetryData) {
  if(i2s_used)
    return audio_i2s_getData(pt_telemetryData);
  else
    return audio_adc_getData(pt_telemetryData); 

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_suspend(void) {
  if(i2s_used)
    return audio_i2s_suspend();
  else
    return audio_adc_suspend();

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t audio_getResultCount(void) {
  if(i2s_used)
    return audio_i2s_getResultCount();
  else  
    return audio_adc_getResultCount();

  return 0;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex) {
  if(p_fftResult == NULL)
    return ERROR;

  if(i2s_used)
    return audio_i2s_getResult(p_fftResult, resultIndex);
  else
    return audio_adc_getResult(p_fftResult, resultIndex);

  return OK;
}
