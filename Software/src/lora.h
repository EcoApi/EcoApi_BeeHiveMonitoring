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
#include "power.h"

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
    e_TX_RX_ACK = 0,
    e_RX_DATA,
    e_TX_DATA_QUEUED,
    e_TX_DONE,
    e_DATA_NOT_CHANGED,
    e_SEND_FAILED,
    e_RX_TIME,
} e_LORA_EVENT;

typedef int32_t (*fn_lora_sendData)(uint8_t*, uint8_t, uint8_t*);
typedef void (*fn_lora_event)(e_LORA_EVENT, void*, uint32_t);
#if (STANDBY_ENABLE == 0)
    typedef void (*fn_lora_wakeup)(void);
#endif

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/ 
#if (STANDBY_ENABLE == 1)
    int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_event fn_event);
#else
    int32_t lora_setup(t_RamRet *pt_ramRet, fn_lora_sendData fn_sendData, fn_lora_event fn_event, fn_lora_wakeup fn_wakeup);
    void lora_schedule(uint32_t time);
#endif

int32_t lora_suspend(void);
void lora_process(void);
int32_t lora_getTime(void);

#ifdef __cplusplus
}
#endif
#endif