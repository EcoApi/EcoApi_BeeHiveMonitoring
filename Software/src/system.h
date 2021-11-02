/************************************************************************************//**
 *
 *	\file		system.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __SYSTEM_H
#define __SYSTEM_H

/***************************************************************************************/
/*	Includes																		  
/***************************************************************************************/
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
typedef enum e_BOR_LEVEL_{
  BOR_Level_0 = OB_BOR_OFF, //Disabled brown-out detection, from 1.62 V to 2.1 V
  BOR_Level_1 = OB_BOR_LEVEL1, //According to the datasheet, from 2.1 to 2.4 V
	BOR_Level_2 = OB_BOR_LEVEL2, //According to the datasheet, from 2.4 to 2.7 V
	BOR_Level_3 = OB_BOR_LEVEL3, //According to the datasheet, from 2.7 to 3.6 V
} e_BOR_LEVEL;

/***************************************************************************************/
/*	Shared Functions																
/***************************************************************************************/  
void system_clockConfig_16MHz(void);
int32_t system_getBorLevel(e_BOR_LEVEL *pe_borLevel);
int32_t system_setBorLevel(e_BOR_LEVEL e_borLevel);
void system_reset(void);

#ifdef __cplusplus
}
#endif

#endif


