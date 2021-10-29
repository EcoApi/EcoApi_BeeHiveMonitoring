 /************************************************************************************//**
 *
 *	\file		trace.c
 *
 *	\brief
 *
 *	\date		30 sept. 2019
 *
 *	\author		rvaast
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes																		                                       */
/***************************************************************************************/
#include "trace.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/***************************************************************************************/
/*	Defines		  	 	 															                                     */
/***************************************************************************************/


/***************************************************************************************/
/*	Defines		  	 	 															                                     */
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    */
/***************************************************************************************/
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         */
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		void trace_init(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
void trace_init(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return;

  pt_ramRet_ = pt_ramRet;

  printf("[TRACE] init ok\r\n");
}

/************************************************************************************
 *
 *	\fn		void trace_setState(uint8_t u8_enable)
 *	\brief 
 *
 ***************************************************************************************/
void trace_setState(uint8_t u8_enable) {
  if((pt_ramRet_ == NULL) || (pt_ramRet_->traceEnable == u8_enable))
    return;
  
  if(u8_enable)
    printf("[TRACE] enable\r\n");
  else
    printf("[TRACE] disable\r\n");

  pt_ramRet_->traceEnable = u8_enable;
  ramret_save(pt_ramRet_);
}

/************************************************************************************
 *
 *	\fn		uint8_t trace_getState(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t trace_getState(void) {
  if(pt_ramRet_ == NULL)
    return FALSE;
  
  return pt_ramRet_->traceEnable;
}  

/************************************************************************************
 *
 *	\fn		void trace_callback(uint8_t withCrLf, const char * format, ... )
 *	\brief 
 *
 ***************************************************************************************/
void trace_callback(uint8_t withCrLf, const char * format, ... ) {
  va_list argptr;
    
  if((pt_ramRet_ == NULL) || (pt_ramRet_->traceEnable == FALSE))
    return;

  va_start(argptr, format);
  vprintf(format, argptr);
  va_end(argptr);
  
  if(withCrLf)
    printf("\r\n");
}
