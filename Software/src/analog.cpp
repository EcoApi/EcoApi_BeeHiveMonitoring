/************************************************************************************//**
 *
 *	\file		analog.cpp
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
#include "adc.h"
#include "analog.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static int32_t getExternalAnaRing(int32_t vref_mvolt);
static int32_t getExternalVbatt(int32_t vref_mvolt);
static int32_t getInternalTemp(int32_t vref_mvolt); 
static int32_t getInternalVbatt(int32_t vref_mvolt);
static int32_t getInternalVrefInternal(int32_t vref_mvolt);
static int32_t getInternalVref(void);

/***************************************************************************************
 *
 *	\fn		static int32_t getExternalAnaRing(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getExternalAnaRing(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_RING), LL_ADC_RESOLUTION_12B);
}

/***************************************************************************************
 *
 *	\fn		static int32_t getExternalVbatt(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getExternalVbatt(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_VBAT), LL_ADC_RESOLUTION_12B);
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalTemp(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getInternalTemp(int32_t vref_mvolt) { 
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_TEMPERATURE(vref_mvolt, AnalogRead(INTERNAL_TEMP), LL_ADC_RESOLUTION_12B);
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalVbatt(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getInternalVbatt(int32_t vref_mvolt) {
  __HAL_RCC_ADC1_CLK_ENABLE();

  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(INTERNAL_VBAT), LL_ADC_RESOLUTION_12B) * VBAT_RATIO;
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalVrefInternal(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getInternalVrefInternal(int32_t vref_mvolt) {
#if 0
  uint16_t vref_12b;
  int32_t vref_mvolt, vrefint_mvolt;

  
  /* Optionally, for this example purpose, calculate analog reference       */
  /* voltage (Vref+) from ADC conversion of internal voltage reference      */
  /* VrefInt.                                                               */
  /* This voltage should correspond to value of literal "VDDA_APPLI".       */
  /* Note: This calculation can be performed when value of voltage Vref+    */
  /* is unknown in the application.                                         */
  vrefint_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, vref_12b, LL_ADC_RESOLUTION_12B);

  return vrefint_mvolt;
#else
  return 0;
#endif
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalVref(void)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getInternalVref(void) {
  uint16_t vref_12b;
  int32_t vref_mvolt, vrefint_mvolt;

  __HAL_RCC_ADC1_CLK_ENABLE();

  vref_12b = AnalogRead(INTERNAL_VREF); 

  vref_mvolt = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_12b, LL_ADC_RESOLUTION_12B);
  //vrefint_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, vref_12b, LL_ADC_RESOLUTION_12B);

  return vref_mvolt;
  //return vrefint_mvolt;
}

/***************************************************************************************
 *
 *	\fn		float analog_getVBattPercent(int32_t vbatt)
 *	\brief 
 *
 ***************************************************************************************/
float analog_getVBattPercent(int32_t vbatt) {
  float percent;
  
  /* 18650 battery percent */
  const float battery_max = 4100; //maximum voltage of battery
  const float battery_min = 3500;  //minimum voltage of battery before shutdown

  percent = (float)((float)(vbatt - battery_min) / (float)(battery_max - battery_min)) * 100;
  if(percent > 100)
    percent = 100.0f;

  return percent;  
}

/***************************************************************************************
 *
 *	\fn		int32_t analog_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t analog_setup(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t analog_getData(t_telemetryData *pt_telemetryData)
 *	\brief 
 *
 ***************************************************************************************/
int32_t analog_getData(t_telemetryData *pt_telemetryData) {
  if(pt_telemetryData == NULL)
    return ERROR;
  
  int32_t vref = getInternalVref();
  //static int32_t getInternalVrefInternal(int32_t vref_mvolt);
  int32_t temperatureInternal = getInternalTemp(vref); 
  int32_t vbatt = getInternalVbatt(vref);
  int32_t anaRing = getExternalAnaRing(vref);
  pt_telemetryData->vbatt = (float) getExternalVbatt(vref) / ((float)R2/((float)R1+(float)R2)); 

  TRACE_CrLf("[ANALOG] vbattInt: %d mv, vbattExtPercent: %0.2f, vbattExt: %d mv, ring: %d, temp: %d  C", vbatt,
                                                                                                         analog_getVBattPercent(pt_telemetryData->vbatt),
                                                                                                         pt_telemetryData->vbatt,
                                                                                                         anaRing,
                                                                                                         temperatureInternal);

  return OK;                                                                                                       
}

/***************************************************************************************
 *
 *	\fn		int32_t analog_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t analog_suspend(void) {

  return OK;
}
