/************************************************************************************//**
 *
 *	\file		lora.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __LORA_H_
#define __LORA_H_

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include "board.h"
#include "ramret.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef enum {
    e_TX_COMPLETE = 0,
    e_DATA_NOT_CHANGED,
    e_SEND_FAILED,
    e_RX_TIME,
} e_LORA_EVENT;

typedef int32_t (*fn_lora_sendData)(uint8_t*, uint8_t, uint8_t*);
typedef int32_t (*fn_lora_receiveData)(uint8_t*, uint8_t);
typedef void (*fn_lora_event)(e_LORA_EVENT, void*);

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_receiveData fn_receiveData, fn_lora_event fn_event);
int32_t lora_suspend(void);
void lora_process(void);
int32_t lora_getTime(void) ;
void lora_schedule(uint32_t time);

#ifdef __cplusplus
}
#endif
#endif