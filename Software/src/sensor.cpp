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
#define T_PRECISION (1)
#define H_PRECISION (1)
#define P_PRECISION (0)
#define W_PRECISION (0)
#define V_PRECISION (1)

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static t_RamRet *pt_ramRet_ = NULL;
static uint16_t dataFlag_ = FLAG_NONE;

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

  /* base info */
  if(dataFlag_ & FLAG_BASE_INFO)
    pt_telemetryData->contentInfo.details.baseInfo = (pt_ramRet->baseInfoSended == FALSE) ? TRUE : FALSE;
  else
    pt_telemetryData->contentInfo.details.baseInfo = FALSE;

  /* boot info */
  if(dataFlag_ & FLAG_BOOT_INFO)
    pt_telemetryData->contentInfo.details.boot = (power_isWatchdogReset() == TRUE) ? TRUE : FALSE;
  else
    pt_telemetryData->contentInfo.details.boot = FALSE;

  /* outside info */
  if(dataFlag_ & FLAG_OUT_INFO) {
    pt_telemetryData->contentInfo.details.temperatureOutside = (roundFloat(pt_telemetryData->temperatureOutside, T_PRECISION) != roundFloat(pt_ramRet->telemetryData.temperatureOutside, T_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.humidityOutside = (roundFloat(pt_telemetryData->humidityOutside, H_PRECISION) != roundFloat(pt_ramRet->telemetryData.humidityOutside, H_PRECISION)) ? TRUE : FALSE;  
    pt_telemetryData->contentInfo.details.pressureOutside = (roundFloat(pt_telemetryData->pressureOutside, P_PRECISION) != roundFloat(pt_ramRet->telemetryData.pressureOutside, P_PRECISION)) ? TRUE : FALSE;  
  } else {
    pt_telemetryData->contentInfo.details.temperatureOutside = FALSE;
    pt_telemetryData->contentInfo.details.humidityOutside = FALSE;
    pt_telemetryData->contentInfo.details.pressureOutside = FALSE;
  }

  /* weight info */
  pt_telemetryData->contentInfo.details.scaleType = pt_ramRet->telemetryData.contentInfo.details.scaleType; 
  if(dataFlag_ & FLAG_WEIGHT_INFO)
    pt_telemetryData->contentInfo.details.weight = (roundFloat(pt_telemetryData->weight, W_PRECISION) != roundFloat(pt_ramRet->telemetryData.weight, W_PRECISION)) ? TRUE : FALSE;  
  else
    pt_telemetryData->contentInfo.details.weight = FALSE;
    
  /* batt info */
  if(dataFlag_ & FLAG_BATT_INFO)
    pt_telemetryData->contentInfo.details.vbatt = (roundFloat(pt_telemetryData->vbatt / 1000, V_PRECISION) != roundFloat(pt_ramRet->telemetryData.vbatt / 1000, V_PRECISION)) ? TRUE : FALSE;  
  else 
    pt_telemetryData->contentInfo.details.vbatt = FALSE;
  
  /* audio info */  
  if(dataFlag_ & FLAG_AUDIO_INFO)  
    pt_telemetryData->contentInfo.details.audio = audio_getResultCount() ? TRUE : FALSE;
  else
    pt_telemetryData->contentInfo.details.audio = FALSE;

  /* custom info */
  if(dataFlag_ & FLAG_CUSTOM_INFO) {

  } else {
    pt_telemetryData->contentInfo.details.custom = FALSE; // not impemented for the moment
  }

  /* inside info */
  if(dataFlag_ & FLAG_IN_INFO) {
    pt_telemetryData->contentInfo.details.humidityInside = (roundFloat(pt_telemetryData->humidityInside, H_PRECISION) != roundFloat(pt_ramRet->telemetryData.humidityInside, H_PRECISION)) ? TRUE : FALSE;

    pt_telemetryData->contentInfo.details.temperatureInside = FALSE;
    for(i=0;i<pt_telemetryData->contentInfo.details.temperatureInsideCount;i++) {
      if(roundFloat(pt_telemetryData->temperatureInside[i], T_PRECISION) != roundFloat(pt_ramRet->telemetryData.temperatureInside[i], T_PRECISION)) {
        pt_telemetryData->contentInfo.details.temperatureInside = TRUE;
        break;
      }  
    }
  } else {
    pt_telemetryData->contentInfo.details.humidityInside = FALSE;
    pt_telemetryData->contentInfo.details.temperatureInside = FALSE;
    pt_telemetryData->contentInfo.details.temperatureInsideCount = 0;
  }  

  memcpy(&pt_ramRet->telemetryData, pt_telemetryData, sizeof(t_telemetryData));

  if(pt_ramRet->telemetryData.contentInfo.details.baseInfo || pt_ramRet->telemetryData.contentInfo.details.boot ||
     pt_ramRet->telemetryData.contentInfo.details.temperatureOutside || pt_ramRet->telemetryData.contentInfo.details.humidityOutside ||
     pt_ramRet->telemetryData.contentInfo.details.pressureOutside || pt_ramRet->telemetryData.contentInfo.details.weight ||
     pt_ramRet->telemetryData.contentInfo.details.vbatt || pt_ramRet->telemetryData.contentInfo.details.humidityInside ||
     pt_ramRet->telemetryData.contentInfo.details.audio || pt_ramRet->telemetryData.contentInfo.details.custom ||
     pt_ramRet->telemetryData.contentInfo.details.temperatureInside) {
    TRACE_CrLf("[SENSOR] data change, binfo %d, boot %d, t_out %d, h_out %d, p_out %d, w %d, vbatt %d, h_in %d, audio %d, custom %d, t_in %d", 
                                                                                                pt_ramRet->telemetryData.contentInfo.details.baseInfo,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.boot,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.temperatureOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.humidityOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.pressureOutside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.weight,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.vbatt,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.humidityInside,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.audio,
                                                                                                pt_ramRet->telemetryData.contentInfo.details.custom,
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
int32_t sensor_setup(t_RamRet *pt_ramRet, uint16_t dataFlag) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;
  dataFlag_ = dataFlag;

  Wire.setSCL(I2C1_SCL);
  Wire.setSDA(I2C1_SDA);

  //if(power_isPoweredOn()) {
    /* todo detect all connected sensor here or in get data call and store presence in ram ret */ 
    /* static detection */
    pt_ramRet_->sensorPresence.sht3x = TRUE;
    pt_ramRet_->sensorPresence.si7021 = FALSE;
    pt_ramRet_->sensorPresence.as6200 = FALSE;
    pt_ramRet_->sensorPresence.bmpxxx = TRUE;
    pt_ramRet_->sensorPresence.bmexxx = FALSE;
    pt_ramRet_->sensorPresence.my_custom_sensor = FALSE;
    pt_ramRet_->sensorPresence.onewire = FALSE;
    pt_ramRet_->sensorPresence.audio_ana = TRUE;
    pt_ramRet_->sensorPresence.audio_i2s = FALSE;
    pt_ramRet_->sensorPresence.hx711 = TRUE;
    pt_ramRet_->sensorPresence.ads12xx = FALSE;
    pt_ramRet_->sensorPresence.nau7802 = FALSE;
    //trace found sensors
  //}

  /* weight info */
  if(dataFlag_ & FLAG_WEIGHT_INFO) {
    if(pt_ramRet_->sensorPresence.hx711) {  
      hx711_setup(pt_ramRet_);
    } else if(pt_ramRet_->sensorPresence.ads12xx) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.nau7802) {
      /* to be create */
    }    
  }  

  /* inside info */
  if(dataFlag_ & FLAG_IN_INFO) {
    if(pt_ramRet_->sensorPresence.onewire) {  
      onewire_setup(pt_ramRet_);
    } else if(pt_ramRet_->sensorPresence.sht3x) {
      sht3x_setup(pt_ramRet_);
    } else if(pt_ramRet_->sensorPresence.si7021) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.as6200) { 
      /* to be create */
    }
  }

  /* outside info */
  if(dataFlag_ & FLAG_OUT_INFO) {
    if(pt_ramRet_->sensorPresence.bmpxxx) { 
      bmp180_setup(pt_ramRet_);
    } else if(pt_ramRet_->sensorPresence.bmexxx) {
      /* to be create */
    }    
  }

  uint32_t vRef = analog_getInternalVref();

  /* audio info */
  if(dataFlag_ & FLAG_AUDIO_INFO) {
    if(pt_ramRet_->sensorPresence.audio_ana) {
      audio_setup(pt_ramRet_, vRef);
    } else if(pt_ramRet_->sensorPresence.audio_i2s) {
      /* to be create */
    }  
  }

  /* batt info */ 
  if(dataFlag_ & FLAG_BATT_INFO) { //always present
    analog_setup(pt_ramRet_, vRef);
  }

  /* custom info */
  if((dataFlag_ & FLAG_CUSTOM_INFO) && pt_ramRet_->sensorPresence.my_custom_sensor) {
    /* to be create */
  } 

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
  
  /* inside info */
  if(dataFlag_ & FLAG_IN_INFO) {
    if(pt_ramRet_->sensorPresence.onewire) {  
      onewire_getData(&telemetryData);
    } else if(pt_ramRet_->sensorPresence.sht3x) {
      sht3x_getData(&telemetryData);
    } else if(pt_ramRet_->sensorPresence.si7021) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.as6200) { 
      /* to be create */
    } 
  }

  /* outside info */
  if(dataFlag_ & FLAG_OUT_INFO) {
    if(pt_ramRet_->sensorPresence.bmpxxx) { 
      bmp180_getData(&telemetryData);
    } else if(pt_ramRet_->sensorPresence.bmexxx) {
      /* to be create */
    }   
  }

  /* weight info */
  if(dataFlag_ & FLAG_WEIGHT_INFO) {
    if(pt_ramRet_->sensorPresence.hx711) {  
      hx711_getData(&telemetryData);
    } else if(pt_ramRet_->sensorPresence.ads12xx) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.nau7802) {
      /* to be create */
    }    
  } 

  /* audio info */
  if(dataFlag_ & FLAG_AUDIO_INFO) {
    if(pt_ramRet_->sensorPresence.audio_ana) {
      audio_getData(&telemetryData);
    } else if(pt_ramRet_->sensorPresence.audio_i2s) {
      /* to be create */
    }  
  }

  /* batt info */ 
  if(dataFlag_ & FLAG_BATT_INFO) { //always present
    analog_getData(&telemetryData);
  }

  /* custom info */
  if((dataFlag_ & FLAG_CUSTOM_INFO) && pt_ramRet_->sensorPresence.my_custom_sensor) {
    /* to be create */
  }

  return sensor_updateContentInfo(pt_ramRet_, &telemetryData);
}

/***************************************************************************************
 *
 *	\fn		int32_t sensor_getData(void) 
 *	\brief 
 *
 ***************************************************************************************/
int8_t sensor_getAudioData(uint8_t *p_data, uint8_t dataSize) {
  FFT_RESULTS fftResult;
  uint8_t resultCount;
  uint8_t rawDataSize;
  
  if((p_data == NULL) || !dataSize)
    return 0;
  
  resultCount = audio_getResultCount();

  if(!resultCount)
    return 0;

  if(audio_getResult(&fftResult, 0) != OK)
    return 0;

  rawDataSize = sizeof(fftResult.binOffset) + sizeof(fftResult.binSize) + sizeof(fftResult.binCount) + (fftResult.binCount * sizeof(fftResult.values[0]));

  if(rawDataSize > dataSize)
    return 0;

  //todo more result or more bins

  memcpy(p_data, (uint8_t*) &fftResult, rawDataSize);

  return rawDataSize;
}

/***************************************************************************************
 *
 *	\fn		int32_t sensor_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t sensor_suspend(void) {
  /* inside info */
  if(dataFlag_ & FLAG_IN_INFO) {
    if(pt_ramRet_->sensorPresence.onewire) {  
      onewire_suspend();
    } else if(pt_ramRet_->sensorPresence.sht3x) {
      sht3x_suspend();
    } else if(pt_ramRet_->sensorPresence.si7021) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.as6200) { 
      /* to be create */
    } 
  }

  /* outside info */
  if(dataFlag_ & FLAG_OUT_INFO) {
    if(pt_ramRet_->sensorPresence.bmpxxx) { 
      bmp180_suspend();
    } else if(pt_ramRet_->sensorPresence.bmexxx) {
      /* to be create */
    }   
  }

  /* weight info */
  if(dataFlag_ & FLAG_WEIGHT_INFO) {
    if(pt_ramRet_->sensorPresence.hx711) {  
      hx711_suspend();
    } else if(pt_ramRet_->sensorPresence.ads12xx) {
      /* to be create */
    } else if(pt_ramRet_->sensorPresence.nau7802) {
      /* to be create */
    }    
  } 

  /* audio info */
  if(dataFlag_ & FLAG_AUDIO_INFO) {
    if(pt_ramRet_->sensorPresence.audio_ana) {
      audio_suspend();
    } else if(pt_ramRet_->sensorPresence.audio_i2s) {
      /* to be create */
    }  
  }

  /* batt info */ 
  if(dataFlag_ & FLAG_BATT_INFO) { //always present
    analog_suspend();
  }

  /* custom info */
  if((dataFlag_ & FLAG_CUSTOM_INFO) && pt_ramRet_->sensorPresence.my_custom_sensor) {
    /* to be create */
  }

  pinMode(I2C1_SCL, INPUT);
  pinMode(I2C1_SDA, INPUT);

  Wire.end();

  return OK;
}