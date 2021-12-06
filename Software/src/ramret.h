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

#define AUDIO_DEFAULT_BIN_OFFSET 13 // 13 * 3.789062Hz = 49.26Hz first not audible
#define AUDIO_DEFAULT_BIN_SIZE 13
#define AUDIO_MAX_BINS 20
#define AUDIO_DEFAULT_MAX_SEND_DATA_CYCLE 2 // max value 7 (3 bits)
#define AUDIO_SEND_DATA_OFFSET 1 // minimum minutes for duty cyle ttn
#define AUDIO_DEFAULT_SAMPLING_FREQUENCY 1

#define DEFAULT_SEND_FREQUENCY 2 //15 // minutes

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef struct t_SensorPresence_ {
  uint16_t sht3x:1; /* i2c */ 
  uint16_t si7021:1; /* i2c */
  uint16_t as6200:1; /* i2c */
  uint16_t bmpxxx:1; /* i2c (check if same address) */
  uint16_t bmexxx:1; /* i2c (check if same address) */
  uint16_t my_custom_sensor:1;  /* i2c create slave with register 0x00 = n telemetry, 0x01 data type 1, 0x02 data 1, 0x03 data type n, 0x04 data n ... */
  uint16_t onewire:1; /* 1 wire ds18b20 or other temp / humidity (see maxim family code) */
  uint16_t audio_ana:1; /* when present add info input (warning check power consumption */
  
  uint16_t audio_i2s:1; /* bus check */
  uint16_t hx711:1; /* bus check */
  uint16_t ads12xx:1; /* bus check */
  uint16_t nau7802:1; /* bus check */
  uint16_t reserved:4;
} t_SensorPresence;

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
    uint8_t reserved:1;
  } details;
  uint8_t data[2];  
} t_ContentInformation;

typedef struct t_telemetryData_ {
  //1wire or i2c
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

typedef struct t_hx711Settings_ {
  long offset; 
  float calibrationFactor; 
} t_hx711Settings;

typedef struct t_loraSettings_ {
  uint32_t seqnoUp; 
  uint32_t seqnoDown; 
    
  uint8_t waitDownlinkMaxCycle:3; // max cycle to wait downlink 
  uint8_t waitDownlinkCounter:3; // if counter > max cyle then wait downlink once
  uint8_t reserved:2;
} t_loraSettings;

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

typedef struct t_RamRet_ {
  uint8_t isUsed:1;
  uint8_t hx711_calibrated:1;
  uint8_t timeUpdated:1;
  uint8_t traceEnable:1;
  uint8_t baseInfoSended:1;
  uint8_t forceNewJoining:1;
  uint8_t reserved:2; 
    
  uint8_t boot; //wdg boot to be developped
  uint8_t sendFrequency; // minutes
  uint32_t lastSendTime; // timestamp

  t_hx711Settings hx711Settings;
  t_loraSettings loraSettings;
  t_audioSettings audioSettings;
  
  t_SensorPresence sensorPresence; 
  t_telemetryData telemetryData; 

  uint32_t u32_magic; 
} t_RamRet; 

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
uint8_t ramret_init(t_RamRet *pt_ramRet);
void ramret_save(t_RamRet *pt_ramRet);
void ramret_dump(t_RamRet *pt_ramRet);
uint8_t ramret_isNew(void);
void ramret_clean(t_RamRet *pt_ramRet);

#ifdef __cplusplus
}
#endif
#endif