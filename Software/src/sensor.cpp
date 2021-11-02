/************************************************************************************//**
 *
 *	\file		sensor.cpp
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
#include <Wire.h>
#include "sensor.h"
#include "power.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define BMP180_ENABLE (1)
#define ONEWIRE_ENABLE (1)
#define ANALOG_ENABLE (1)
#define HX711_ENABLE (1)
#define MAX9815_ENABLE (0)
#define TILT_ENABLE (0)

#define T_PRECISION (1)
#define H_PRECISION (1)
#define P_PRECISION (0)
#define W_PRECISION (0)
#define V_PRECISION (1)

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static float roundFloat(float value, uint8_t precision);
static int32_t sensor_updateContentInfo(t_RamRet *pt_ramRet, t_telemetryData *pt_telemetryData);

/***************************************************************************************
 *
 *	\fn		static float roundFloat(float value, uint8_t precision)
 *	\brief 
 *
 ***************************************************************************************/
static float roundFloat(float value, uint8_t precision) {
  float factor = pow(10.0f, (float) precision);
  
  return round(value * factor) / factor;
}

/***************************************************************************************
 *
 *	\fn		static int32_t sensor_updateContentInfo(t_RamRet *pt_ramRet, t_telemetryData *pt_telemetryData)
 *	\brief 
 *
 ***************************************************************************************/
static int32_t sensor_updateContentInfo(t_RamRet *pt_ramRet, t_telemetryData *pt_telemetryData) {
  uint8_t i;

  if((pt_ramRet == NULL) || (pt_telemetryData == NULL))
    return ERROR;
#if 0
  if(power_isPoweredOn()) {
    // todo define default value and check 

    pt_telemetryData->contentInfo.details.baseInfo = TRUE;
    pt_telemetryData->contentInfo.details.boot = TRUE;
    pt_telemetryData->contentInfo.details.temperatureOutside = (roundFloat(pt_telemetryData->temperatureOutside, 1) != roundFloat(pt_ramRet->telemetryData.temperatureOutside, 1)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.humidityOutside = (roundFloat(pt_telemetryData->humidityOutside, 1) != roundFloat(pt_ramRet->telemetryData.humidityOutside, 1)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.pressureOutside = (roundFloat(pt_telemetryData->pressureOutside, 0) != roundFloat(pt_ramRet->telemetryData.pressureOutside, 0)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.weight = (roundFloat(pt_telemetryData->weight, 0) != roundFloat(pt_ramRet->telemetryData.weight, 0)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.vbatt = (roundFloat(pt_telemetryData->vbatt / 1000, 1) != roundFloat(pt_ramRet->telemetryData.vbatt / 1000, 1)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.humidityInside = (roundFloat(pt_telemetryData->humidityInside, 1) != roundFloat(pt_ramRet->telemetryData.humidityInside, 1)) ? TRUE : FALSE;
    pt_telemetryData->contentInfo.details.frequency = FALSE; // not implemented for the moment
    pt_telemetryData->contentInfo.details.gas = FALSE; // not impemented for the moment

  } else
#endif  
  {
    pt_telemetryData->contentInfo.details.baseInfo = (pt_ramRet->baseInfoSended == FALSE) ? TRUE : FALSE;
    pt_telemetryData->contentInfo.details.boot = (power_isWatchdogReset() == TRUE) ? TRUE : FALSE;
    pt_telemetryData->contentInfo.details.temperatureOutside = (roundFloat(pt_telemetryData->temperatureOutside, T_PRECISION) != roundFloat(pt_ramRet->telemetryData.temperatureOutside, T_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.humidityOutside = (roundFloat(pt_telemetryData->humidityOutside, H_PRECISION) != roundFloat(pt_ramRet->telemetryData.humidityOutside, H_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.pressureOutside = (roundFloat(pt_telemetryData->pressureOutside, P_PRECISION) != roundFloat(pt_ramRet->telemetryData.pressureOutside, P_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.weight = (roundFloat(pt_telemetryData->weight, W_PRECISION) != roundFloat(pt_ramRet->telemetryData.weight, W_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.vbatt = (roundFloat(pt_telemetryData->vbatt / 1000, V_PRECISION) != roundFloat(pt_ramRet->telemetryData.vbatt / 1000, V_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.humidityInside = (roundFloat(pt_telemetryData->humidityInside, H_PRECISION) != roundFloat(pt_ramRet->telemetryData.humidityInside, H_PRECISION)) ? TRUE : FALSE;
    pt_telemetryData->contentInfo.details.frequency = FALSE; // not implemented for the moment
    pt_telemetryData->contentInfo.details.gas = FALSE; // not impemented for the moment

    pt_telemetryData->contentInfo.details.temperatureInside = FALSE;
    for(i=0;i<pt_telemetryData->contentInfo.details.temperatureInsideCount;i++) {
      if(roundFloat(pt_telemetryData->temperatureInside[i], T_PRECISION) != roundFloat(pt_ramRet->telemetryData.temperatureInside[i], T_PRECISION)) {
        pt_telemetryData->contentInfo.details.temperatureInside = TRUE;
        break;
      }  
    }
  }  

  memcpy(&pt_ramRet->telemetryData, pt_telemetryData, sizeof(t_telemetryData));

  if(pt_ramRet->telemetryData.contentInfo.details.baseInfo || pt_ramRet->telemetryData.contentInfo.details.boot ||
     pt_ramRet->telemetryData.contentInfo.details.temperatureOutside || pt_ramRet->telemetryData.contentInfo.details.humidityOutside ||
     pt_ramRet->telemetryData.contentInfo.details.pressureOutside || pt_ramRet->telemetryData.contentInfo.details.weight ||
     pt_ramRet->telemetryData.contentInfo.details.vbatt || pt_ramRet->telemetryData.contentInfo.details.humidityInside ||
     pt_ramRet->telemetryData.contentInfo.details.frequency || pt_ramRet->telemetryData.contentInfo.details.gas ||
     pt_ramRet->telemetryData.contentInfo.details.temperatureInside) {
    TRACE_CrLf("[SENSOR] data change, binfo %d, boot %d, t_out %d, h_out %d, p_out %d, w %d, vbatt %d, h_in %d, freq %d, gas %d, t_in %d", 
                                                                                                pt_ramRet->telemetryData.contentInfo.details.baseInfo,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.boot,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.temperatureOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.humidityOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.pressureOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.weight,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.vbatt,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.humidityInside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.frequency,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.gas,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.temperatureInside);
                                                                                               
  } else {
    TRACE_CrLf("[SENSOR] data not change");
    return DATA_NOT_CHANGED;
  }

  return OK;  
}

/***************************************************************************************
 *
 *	\fn		int32_t sensor_setup(t_RamRet *pt_ramRet) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t sensor_setup(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;

//io setup

  Wire.setSCL(I2C1_SCL);
  Wire.setSDA(I2C1_SDA);

#if (ONEWIRE_ENABLE == 1)    
  onewire_setup(pt_ramRet);
#endif
#if (BMP180_ENABLE == 1)
  bmp180_setup(pt_ramRet);
#endif
#if (HX711_ENABLE == 1)
  hx711_setup(pt_ramRet);
#endif
#if (ANALOG_ENABLE == 1)
  analog_setup(pt_ramRet);
#endif

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sensor_getData(void) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t sensor_getData(void) {
   t_telemetryData telemetryData; 
  
#if (ONEWIRE_ENABLE == 1)    
    onewire_getData(&telemetryData);
#endif
#if (BMP180_ENABLE == 1)
    bmp180_getData(&telemetryData);
#endif
#if (HX711_ENABLE == 1)
    hx711_getData(&telemetryData);
#endif
#if (ANALOG_ENABLE == 1)
    analog_getData(&telemetryData);
#endif

#if 0
  telemetryData.humidityOutside = t_ramRet.telemetryData.humidityOutside;
  telemetryData.humidityOutside += 1.0;
  if(telemetryData.humidityOutside > 100.0)
    telemetryData.humidityOutside = 0.0;
  telemetryData.humidityInside = telemetryDataCurrent.humidityOutside;
  TRACE_CrLf("[SIMU] enable, humidity in %0.2f rh, out %0.2f rh", telemetryData.humidityInside, telemetryData.humidityOutside);
#else
  telemetryData.humidityInside = 0.0;
  telemetryData.humidityOutside = 0.0;
  TRACE_CrLf("[SIMU] disable, humidity in %0.2f rh, out %0.2f rh", telemetryData.humidityInside, telemetryData.humidityOutside);
#endif

  return sensor_updateContentInfo(pt_ramRet_, &telemetryData);
}

/***************************************************************************************
 *
 *	\fn		int32_t sensor_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t sensor_suspend(void) {

  return OK;
}
