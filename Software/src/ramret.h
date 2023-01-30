/************************************************************************************//**
 *
 *	\file		ramret.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __RAMRET_H
#define __RAMRET_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <stdint.h>
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
#define MAX_TEMPERATURE_INSIDE 2
#define LORA_DEFAULT_MAX_WAIT_DOWNLINK_CYCLE 4  // max value 7 (3 bits)

#if (USE_EEPROM == 0)
  #define AUDIO_DEFAULT_BIN_OFFSET 13 // 13 * 3.789062Hz = 49.26Hz first not audible
  #define AUDIO_DEFAULT_BIN_SIZE 13
  #define AUDIO_MAX_BINS 20
  #define AUDIO_DEFAULT_MAX_SEND_DATA_CYCLE 2 // max value 7 (3 bits)
  #define AUDIO_SEND_DATA_OFFSET 1 // minimum minutes for duty cyle ttn
  #define AUDIO_DEFAULT_SAMPLING_FREQUENCY 1

  #define DEFAULT_SEND_FREQUENCY 5 //15 // minutes
  #define DEFAULT_UPDATE_TIME_PERIOD 86400 // seconds
#endif

#define DEFAULT_MOTION_DETECT_PERIOD  60 // seconds min ttn between two frame
#define DEFAULT_STANDBY_MOTION_DETECT_TIMEOUT 30 // seconds

#define MIN_SLEEP_PERIOD 60 // seconds
#define MAX_SLEEP_PERIOD 86400 // seconds

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
#if (USE_EEPROM == 0)
typedef struct t_SensorPresence_ {
  uint16_t sht3x:1; /* i2c */ 
  uint16_t si7021:1; /* i2c */
  uint16_t as6200:1; /* i2c */
  uint16_t bmpxxx:1; /* i2c (check if same address) */
  uint16_t bmexxx:1; /* i2c (check if same address) */
  uint16_t my_custom_sensor:1;  /* i2c create slave with register 0x00 = n telemetry, 0x01 data type 1, 0x02 data 1, 0x03 data type n, 0x04 data n ... */
  uint16_t audio_ana:1; /* when present add info input (warning check power consumption */
  
  uint16_t audio_i2s:1; /* bus check */
  uint16_t hx711:1; /* bus check */
  uint16_t ads12xx:1; /* bus check */
  uint16_t nau7802:1; /* bus check */
  uint16_t reserved:5;
} t_SensorPresence;
#endif

typedef union t_ContentInformation_ {
  struct {    
    uint8_t baseInfo:1;
    uint8_t boot:1;
    uint8_t temperatureOutside:1;
    uint8_t humidityOutside:1;
    uint8_t pressureOutside:1;
    uint8_t weight:1;
    uint8_t vbatt:1;
    uint8_t humidityInside:1;
    
    uint8_t audio:1;
    uint8_t custom:1;
    uint8_t temperatureInside:1;
    uint8_t temperatureInsideCount:3;
    uint8_t scaleType:1;
    uint8_t motionDetectionOrPowerOn:1;
  } details;
  uint8_t data[2];  
} t_ContentInformation;

typedef struct t_telemetryData_ {
  // i2c
  float temperatureInside[MAX_TEMPERATURE_INSIDE]; 
  float humidityInside; 

  //bmp180 or bmeXXX or ...
  float temperatureOutside; 
  float pressureOutside; 
  float humidityOutside; 

  //hx711
  float weight; 

  //analog
  int32_t vbatt; 
  //float vbattExtPercent; 
  
  //audio ana/i2s or accelero
  //not in ram retention  

  //custom

  //not in ram retention   

  t_ContentInformation contentInfo; 
} t_telemetryData; 

#if (USE_EEPROM == 0)
typedef struct t_hx711Settings_ {
  long offset; 
  float calibrationFactor; 
} t_hx711Settings;
#endif

typedef struct t_loraSettings_ {
  uint32_t seqnoUp; 
  uint32_t seqnoDown; 
    
  uint8_t waitDownlinkMaxCycle:3; // max cycle to wait downlink 
  uint8_t waitDownlinkCounter:3; // if counter > max cyle then wait downlink once
  uint8_t reserved:2;
} t_loraSettings;

#if (USE_EEPROM == 0)
typedef struct t_audioSettings_ {
  uint16_t binOffset; /* start bin */
  uint8_t binSize; /* bin count in one output bin */
  uint8_t binCount; /* bin output count */

  uint8_t gain:2; /* 0: low, 1:medium, 2:hight, 3:very hight */
  uint8_t samplingFrequency:2; /* 0: 2000, 1: 4000, 2: 8000, 3: 16000 Hz */
  uint8_t reserved1:4;

  uint8_t sendDataMaxCycle:3; // max cycle to send audio data
  uint8_t sendDataCounter:3; // if counter > max cyle then send audio data 
  uint8_t sendData:1; // indicate send audio data next cycle
  uint8_t reserved:1;
} t_audioSettings;
#endif

typedef struct t_RamRet_ {
  uint32_t u32_magicStart; 
  uint8_t isUsed:1;
  uint8_t hx711_calibrated:1;
  uint8_t timeUpdated:1;
  uint8_t traceEnable:1;
  uint8_t baseInfoSended:1;
  uint8_t forceNewJoining:1;
  uint8_t reserved:2; 
    
  uint8_t boot; //wdg boot to be developped

  uint32_t lastSendTime; // timestamp
  uint32_t lastUpdateTime; // timestamp
  uint32_t lastSendMotionOrPowerTime; // timestamp

#if (USE_EEPROM == 0)  
  uint8_t sendFrequency; // minutes
  
  t_hx711Settings hx711Settings;
  t_audioSettings audioSettings;
  t_SensorPresence sensorPresence; 
#endif
  
  t_loraSettings loraSettings;

  t_telemetryData telemetryData; 

  uint32_t u32_magicEnd; 
} t_RamRet; 

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t ramret_init(t_RamRet *pt_ramRet, bool init);
int32_t ramret_save(t_RamRet *pt_ramRet);
uint8_t ramret_isNew(void);
void ramret_clean(t_RamRet *pt_ramRet);

#ifdef __cplusplus
}
#endif
#endif