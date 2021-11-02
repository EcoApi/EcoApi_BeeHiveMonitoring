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

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef union t_ContentInformation_ {
  struct {    
    //byte 1
    uint8_t baseInfo:1;
    uint8_t boot:1;
    uint8_t temperatureOutside:1;
    uint8_t humidityOutside:1;
    uint8_t pressureOutside:1;
    uint8_t weight:1;
    uint8_t vbatt:1;
    uint8_t humidityInside:1;
    //byte2
    uint8_t frequency:1;
    uint8_t gas:1;
    uint8_t temperatureInside:1;
    uint8_t temperatureInsideCount:3;
    uint8_t scaleType:1;
    uint8_t reserved:1;
  } details;
  uint8_t data[2];  
} t_ContentInformation;

typedef struct t_telemetryData_ {
  //1wire or i2c
  float temperatureInside[MAX_TEMPERATURE_INSIDE]; // 2 * 4 bytes
  float humidityInside; // 4 bytes

  //bmp180 or bmeXXX or ...
  float temperatureOutside; // 4 bytes
  float pressureOutside; // 4 bytes
  float humidityOutside; // 4 bytes

  //hx711
  float weight; // 4 bytes

  //analog
  int32_t vbatt; // 4 bytes
  //float vbattExtPercent; // 4 bytes
  
  //sound or accelero
  //not in ram retention  

  //gas sensor
  //not in ram retention   

  t_ContentInformation contentInfo; // 2 bytes
} t_telemetryData; //size : 10 x 4 bytes

typedef struct t_RamRet_ {
    uint8_t isUsed:1;
    uint8_t hx711_calibrated:1;
    uint8_t timeUpdated:1;
    uint8_t traceEnable:1;
    uint8_t baseInfoSended:1;
    uint8_t forceNewJoining:1;
    uint8_t reserved:2; // 1 byte
    
    uint8_t boot; // 1 byte

    long hx711_offset; // 4 bytes
    float hx711_calibrationFactor; // 4 bytes

    uint32_t LORA_seqnoUp; // 4 bytes
    uint32_t LORA_seqnoDown; // 4 bytes
    uint32_t LORA_lastSendTime; // 4 bytes

    t_telemetryData telemetryData; // 10 x 4 bytes
    
    uint32_t u32_magic; // 4 bytes
} t_RamRet; //size : 16 x 4 bytes //todo we can packed struct

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