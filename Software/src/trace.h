#ifndef __TRACE_H
#define __TRACE_H

/***************************************************************************************/
/*	Includes																		   */
/***************************************************************************************/
#include "board.h"
#include "ramret.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
 * define
 **************************************************************************************/
#define TRACE(fmt, ...) trace_callback(FALSE, fmt, ##__VA_ARGS__)
#define TRACE_CrLf(fmt, ...) trace_callback(TRUE, fmt, ##__VA_ARGS__)

/***************************************************************************************/
/*	Typedef                                                                            */
/***************************************************************************************/

/***************************************************************************************/
/*	Shared Functions																   */
/***************************************************************************************/  
void trace_init(t_RamRet *pt_ramRet);
void trace_setState(uint8_t u8_enable);
uint8_t trace_getState(void);
void trace_callback(uint8_t withCrLf, const char * format, ... );

#ifdef __cplusplus
}
#endif

#endif