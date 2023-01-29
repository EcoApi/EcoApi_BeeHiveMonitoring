/************************************************************************************//**
 *
 *	\file		eeprom.h
 *
 *	\brief
 *
 *	\date		21 jan. 2022
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __EEPROM_H
#define __EEPROM_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <stdint.h>
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (USE_EEPROM == 1)

/***************************************************************************************/
/* Define
/***************************************************************************************/
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
  uint16_t audio_ana:1; /* when present add info input (warning check power consumption */
  
  uint16_t audio_i2s:1; /* bus check */
  uint16_t hx711:1; /* bus check */
  uint16_t ads12xx:1; /* bus check */
  uint16_t nau7802:1; /* bus check */
  uint16_t reserved:5;
} t_SensorPresence;

typedef struct t_hx711Settings_ {
  long offset; 
  float calibrationFactor; 
} t_hx711Settings;

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

typedef struct t_Eeprom_ {
  uint32_t magicStart;

  uint8_t sendFrequency; // minutes

  t_hx711Settings hx711Settings;
  t_audioSettings audioSettings;

  t_SensorPresence sensorPresence;

  uint32_t magicEnd;
} t_Eeprom;


/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t eeprom_init(t_Eeprom *pt_eeprom);
int32_t eeprom_save(t_Eeprom *pt_eeprom);
bool eeprom_isNew(void);
#endif

#ifdef __cplusplus
}
#endif
#endif