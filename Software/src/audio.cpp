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
#include "adc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define DETECT_MAX_VALUE (10)
#define DETECT_THRESHOLD_FOR_I2S (20)
#define DETECT_THRESHOLD_FOR_ADC (1000)
#define DETECT_THRESHOLD_MIN_FOR_NONE (700)
#define DETECT_THRESHOLD_MAX_FOR_NONE (850)

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
 *	\fn		e_AUDIO_TYPE audio_detectType(void)
 *	\brief 
 *
 ***************************************************************************************/
e_AUDIO_TYPE audio_detectType(void) {
  uint16_t vref_12b;
  int32_t vref_mvolt;
  int32_t mic_mvolt;
  uint32_t mic_mvolt_average = 0;

  pinMode(MIC_ANA, INPUT_ANALOG);

  __HAL_RCC_ADC1_CLK_ENABLE();

  vref_12b = AnalogRead(INTERNAL_VREF); 
  vref_mvolt = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_12b, LL_ADC_RESOLUTION_12B);

  for(uint8_t i=0;i<DETECT_MAX_VALUE;i++) {
    mic_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_MIC), LL_ADC_RESOLUTION_12B);

    mic_mvolt_average += mic_mvolt;
  }

  mic_mvolt_average = mic_mvolt_average / DETECT_MAX_VALUE;

  if(mic_mvolt_average <= DETECT_THRESHOLD_FOR_I2S) {
    TRACE_CrLf("[AUDIO] i2s detected, average = %d", mic_mvolt_average);

    return e_AUDIO_TYPE_I2S;
  } else if((mic_mvolt_average > DETECT_THRESHOLD_MIN_FOR_NONE) && (mic_mvolt_average < DETECT_THRESHOLD_MAX_FOR_NONE)) {
    TRACE_CrLf("[AUDIO] none detected, average = %d", mic_mvolt_average);

    return e_AUDIO_TYPE_NONE;
  } else if (mic_mvolt_average > DETECT_THRESHOLD_FOR_ADC) {
    TRACE_CrLf("[AUDIO] adc detected, average = %d", mic_mvolt_average);

    return e_AUDIO_TYPE_ADC;
  } 
  
  TRACE_CrLf("[AUDIO] none detected, average = %d", mic_mvolt_average);

  return e_AUDIO_TYPE_NONE;
}

/***************************************************************************************
 *
 *	\fn		int32_t audio_setup(t_RamRet *pt_eeprom, int32_t vRef, e_AUDIO_TYPE e_audioType) 
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
int32_t audio_setup(t_Eeprom *pt_eeprom, int32_t vRef, e_AUDIO_TYPE e_audioType) {
#else
int32_t audio_setup(t_RamRet *pt_eeprom, int32_t vRef, e_AUDIO_TYPE e_audioType) {
#endif  
  int32_t ret;
  
  if(pt_eeprom == NULL)
    return ERROR;

  if(e_audioType == e_AUDIO_TYPE_NONE)
    return ERROR;

  pt_eeprom_ = pt_eeprom;
  vRef_ = vRef;

  i2s_used = (e_audioType == e_AUDIO_TYPE_I2S) ? TRUE : FALSE;

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
