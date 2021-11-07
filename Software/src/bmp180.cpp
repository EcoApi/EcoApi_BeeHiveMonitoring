/************************************************************************************//**
 *
 *	\file		bmp.cpp
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
#include <SFE_BMP180.h>
#include "bmp180.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define ALTITUDE 51.0 // Altitude of EcoApi pont de l'arche France 

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
SFE_BMP180 bmp180;
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/***************************************************************************************
 *
 *	\fn		int32_t bmp180_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t bmp180_setup(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;

  if(bmp180.begin() != TRUE) {
    TRACE_CrLf("[BMP180] init ko");
    return ERROR;
  }

  TRACE_CrLf("[BMP180] init ok");  

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t bmp180_getData(t_telemetryData *pt_telemetryData) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t bmp180_getData(t_telemetryData *pt_telemetryData) {
  double pressure;
  double altitude;
  double temperature;
  char status;
  
  if(pt_telemetryData == NULL)
    return ERROR;
    
  TRACE("[BMP180] provided altitude: %0.0f m", ALTITUDE);
  
  status = bmp180.startTemperature();
  if(status != 0) {
    delay(status);

    status = bmp180.getTemperature(temperature);
    if (status != 0) {
      pt_telemetryData->temperatureOutside = (float) temperature;
      TRACE(", temp: %0.2f C", pt_telemetryData->temperatureOutside);
    }  
  }  

  status = bmp180.startPressure(1 /* oversampling standard */);
  if(status != 0) {
    delay(status);

    status = bmp180.getPressure(pressure, temperature);
    if(status != 0) {
      pt_telemetryData->pressureOutside = bmp180.sealevel(pressure, ALTITUDE); 
      altitude = bmp180.altitude(pressure, pt_telemetryData->pressureOutside);
    
      TRACE(", absolute pressure: %0.2f mb, relative (sea-level) pressure: %0.2f mb, computed altitude: %0.2f m", pressure,
                                                                                                                  pt_telemetryData->pressureOutside,
                                                                                                                  altitude);
    }                                                                                                                 
  }

  TRACE_CrLf(""); 

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t bmp180_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t bmp180_suspend(void) {

  return OK;
}