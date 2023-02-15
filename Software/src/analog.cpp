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
#define R42 1000000.0
#define R45 2000000.0
#define VBAT_RATIO 4.0

#define BATTERY_MAX 4100 //maximum voltage of battery
#define BATTERY_MIN 3000 //minimum voltage of battery before shutdown

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static t_RamRet *pt_ramRet_ = NULL;
static int32_t vRef_ = 3300;

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
  //__HAL_RCC_ADC1_CLK_ENABLE();

  //return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_RING), LL_ADC_RESOLUTION_12B);

  return 0;
}

/***************************************************************************************
 *
 *	\fn		static int32_t getExternalVbatt(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getExternalVbatt(int32_t vref_mvolt) {
  //__HAL_RCC_ADC1_CLK_ENABLE();

  //return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(EXTERNAL_VBAT), LL_ADC_RESOLUTION_12B);

  return 0;
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

  float vbatt_ana = (float) ((__LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, AnalogRead(INTERNAL_VBAT), LL_ADC_RESOLUTION_12B)) * VBAT_RATIO);

  //TRACE_CrLf("vbatt: %0.2f mv", vbatt_ana);

  float vbatt = vbatt_ana / ((float) R45 / ((float) R45 + (float) R42)) + 28.0; // check with other board 

  return  vbatt;
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalVrefInternal(int32_t vref_mvolt)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t getInternalVrefInternal(int32_t vref_mvolt) {
  uint16_t vref_12b;
  int32_t vrefint_mvolt;

  
  /* Optionally, for this example purpose, calculate analog reference       */
  /* voltage (Vref+) from ADC conversion of internal voltage reference      */
  /* VrefInt.                                                               */
  /* This voltage should correspond to value of literal "VDDA_APPLI".       */
  /* Note: This calculation can be performed when value of voltage Vref+    */
  /* is unknown in the application.                                         */
  vrefint_mvolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mvolt, vref_12b, LL_ADC_RESOLUTION_12B);

  return vrefint_mvolt;
}

/***************************************************************************************
 *
 *	\fn		static int32_t getInternalVref(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t analog_getInternalVref(void) {
  uint16_t vref_12b;
  int32_t vref_mvolt;

  __HAL_RCC_ADC1_CLK_ENABLE();

  vref_12b = AnalogRead(INTERNAL_VREF); 

  vref_mvolt = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_12b, LL_ADC_RESOLUTION_12B);

  return vref_mvolt;
}

/***************************************************************************************
 *
 *	\fn		float analog_getVBattPercent(int32_t vbatt)
 *	\brief 
 *
 ***************************************************************************************/
float analog_getVBattPercent(int32_t vbatt) {
  float percent = (float)((float)(vbatt - BATTERY_MIN) / (float)(BATTERY_MAX - BATTERY_MIN)) * 100;

  return constrain(percent, 0.0, 100.0);
}

/***************************************************************************************
 *
 *	\fn		int32_t analog_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t analog_setup(t_RamRet *pt_ramRet, int32_t vRef) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;
  vRef_ = vRef;

  pinMode(EN_VBATT, OUTPUT);
  digitalWrite(EN_VBATT, HIGH);

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

  vRef_ = analog_getInternalVref();

  int32_t temperatureInternal = getInternalTemp(vRef_); 

  pt_telemetryData->vbatt = getInternalVbatt(vRef_); /* warning amperemeter influence */

  TRACE_CrLf("[ANALOG] vref %d mv, vbattPercent: %0.2f, vbatt: %d mv, temp: %d C", vRef_, 
                                                                                   analog_getVBattPercent(pt_telemetryData->vbatt),
                                                                                   pt_telemetryData->vbatt,
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
  __HAL_RCC_ADC1_CLK_DISABLE();

  pinMode(EN_VBATT, INPUT_ANALOG);

  return OK;
}
