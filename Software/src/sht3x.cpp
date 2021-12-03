/************************************************************************************//**
 *
 *	\file		sht3x.cpp
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
#include <SHT31.h>
#include "sht3x.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define SHT3X_ADDRESS   0x44

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
SHT31 sht3x;
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/***********************************************************
 ***********************************************************

 WRITE MY LIBRARY !!!!

 ***********************************************************
 **********************************************************/

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t sht3x_setup(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;

  if(sht3x.begin(SHT3X_ADDRESS) == FALSE) {
    TRACE_CrLf("[SHT3X] init ko");
    return ERROR;
  }  

  //sht3x.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); 

  //set single shot with fast and set max accuracy

  sht3x.requestData();

  TRACE_CrLf("[SHT3X] init ok");  

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_getData(t_telemetryData *pt_telemetryData) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t sht3x_getData(t_telemetryData *pt_telemetryData) {
  while (FALSE == sht3x.dataReady()); //todo timeout

  if(FALSE == sht3x.readData())
    return ERROR;

  pt_telemetryData->contentInfo.details.temperatureInsideCount = 1;
  pt_telemetryData->temperatureInside[0] = sht3x.getTemperature();
  pt_telemetryData->humidityInside = sht3x.getHumidity();
    
  TRACE_CrLf("[SHT3X] temp: %0.2f C, humidity %0.2f %rh", pt_telemetryData->temperatureInside[0], pt_telemetryData->humidityInside);

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t sht3x_suspend(void) {
  //sht sensor have auto standby only if single shot mode 

  return OK;
}