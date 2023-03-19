/************************************************************************************//**
 *
 *	\file		utils.h
 *
 *	\brief
 *
 *	\date		22 jan. 2023
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __UTILS_H
#define __UTILS_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************/
/* Define
/**************************************************************************************/
#if 0
#define SET_3V3_INTERNAL(state) { \
                                  if(state == true) { \
                                    pinMode(EN_3V3_INT, OUTPUT); \
                                    digitalWrite(EN_3V3_INT, HIGH); \
                                  } else { \
                                    pinMode(EN_3V3_EXT, INPUT_PULLDOWN); \
                                    digitalWrite(EN_3V3_INT, LOW); \
                                  } \
                                }
#endif

#define SET_3V3_EXTERNAL(state) { \
                                  if(state == true) { \
                                    pinMode(EN_3V3_EXT, OUTPUT); \
                                    digitalWrite(EN_3V3_EXT, HIGH); \
                                  } else { \
                                    pinMode(EN_3V3_EXT, INPUT_PULLDOWN); \
                                    digitalWrite(EN_3V3_EXT, LOW); \
                                  } \
                                }


/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
void unwedge_i2c_bus(uint32_t scl_pin, uint32_t sda_pin);
uint32_t time_lapse(uint32_t u32_start, uint32_t u32_end);

#ifdef __cplusplus
}
#endif

#endif