/************************************************************************************//**
 *
 *	\file		board.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __BOARD_H_
#define __BOARD_H_

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F411xE) //blackpill
    #include "stm32f4xx_hal_rcc.h"
    #include "stm32f4xx_hal_adc.h"
    //#include "stm32f4xx_ll_adc.h"
#elif defined(STM32F103xB) //bluepill 
    #include "stm32f1xx_hal_rcc.h"
    #include "stm32f1xx_hal_adc.h"
    //#include "stm32f1xx_ll_adc.h"
#else
    #error "Bad µC defined"
#endif

#include "stm32yyxx_ll_adc.h"

/***************************************************************************************/
/* Define
/***************************************************************************************/
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))

/*
-O0 : optimize for compile time and debug-ability. Most performance optimizations are disabled. This is the default for GCC.
-O1: pick the low hanging fruit in terms of performance, without impacting compilation time too much.
-O2: optimize for performance more aggressively, but not at the cost of larger code size.
-O3: optimize for performance at all cost, no matter the code size or compilation time impact.
-Ofast a.k.a. ludicrous mode: take off the guard rails, and disregard strict C standard compliance in the name of speed. This may lead to invalid programs.
-Os optimize for code size. This uses O2 as the baseline, but disables some optimizations. For example, it will not inline3 code if that leads to a size increase.

or

#pragma GCC push_options
#pragma GCC optimize ("O3")

 Code that needs optimizing

#pragma GCC pop_options
*/

//#define WEAK __attribute__ ((weak))

#define TRUE (1)
#define FALSE !(TRUE)

#define OK (0)
#define ERROR (-1)
#define DATA_NOT_CHANGED (ERROR - 1)

#define HX711_RATE PB_5
#define HX711_DT PB_6
#define HX711_CK PB_7

#define SPI1_MOSI PA_7
#define SPI1_MISO PA_6
#define SPI1_SCK PA_5
//#define SPI1_NSS PA_15
#define FLASH_NSS PA_3
#define LORA_NSS PA_4
#define LORA_RST PB_0
#define LORA_DIO0 PB_1
#define LORA_DIO1 PB_2
#define LORA_DIO2 PB_10
//#define LORA_DIO3 30//PB_15

#define UART1_TX PA_9
#define UART1_RX PA_10

#define PWR_OFF PA_15

#define LED_INFO PA_8



#define WAKEUP_PIN PWR_WAKEUP_PIN1
#define WAKEUP PA_0

//#define BTN_RST ?
//#define ONEWIRE_DATA PA_7
//#define ANA_VBATT PB_0
#define SCALE_TYPE PB_15

#define BTN_TARE PA_2
#define EN_VBATT BTN_TARE

#define TRACE_EN PB_14
//#define BTN_ACT PA_4
//#define RING PA_1 //SYS_WKUP1

#define WDG_RELOAD PC_13

#define MIC_ANA PA_2
#define MIC_GAIN PB_13

#define I2S_SD MIC_ANA
#define I2S_CK MIC_GAIN
#define I2S_WS PB_14

#define I2C1_SCL PB_8
#define I2C1_SDA PB_9


#define EN_3V3_INT PB_3
#define EN_3V3_EXT PB_4

#ifdef __cplusplus
}
#endif
#endif