/************************************************************************************//**
 *
 *	\file		trace.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __TRACE_H
#define __TRACE_H

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include <stdio.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************/
/* Define
/**************************************************************************************/
#define TRACE(fmt, ...) trace_callback(FALSE, fmt, ##__VA_ARGS__)
#define TRACE_CrLf(fmt, ...) trace_callback(TRUE, fmt, ##__VA_ARGS__)
#define TRACE_DUMP(msg, buf, size) trace_display_buf(msg, buf, size)

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
void trace_init(HardwareSerial *p_serial);
void trace_setState(bool enable);
bool trace_getState(void);
void trace_callback(uint8_t withCrLf, const char * format, ... );
void trace_display_buf(const char* msg, const uint8_t* p_buffer, uint32_t bufferSize);

#ifdef __cplusplus
}
#endif

#endif