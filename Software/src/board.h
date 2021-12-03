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

#define HX711_DT PB_8
#define HX711_CK PB_9
#define SPI1_MOSI PB_5
#define SPI1_MISO PB_4
#define SPI1_SCK PB_3
#define SPI1_NSS PA_15
#define LORA_NSS PA_12
#define LORA_RST PA_11
#define LORA_DIO0 27//PB_12
#define LORA_DIO1 28//PB_13
#define LORA_DIO2 29//PB_14
#define LORA_DIO3 30//PB_15
#define UART1_TX PA_9
#define UART1_RX PA_10

#define LED_INFO PA_8

#define R1 300000.0
#define R2 300000.0

#define WAKEUP_PIN PWR_WAKEUP_PIN1

//#define BTN_RST ?
#if defined(STM32F411xE) //blackpill
    #define ONEWIRE_DATA PA_7
    #define ANA_VBATT PB_0
    #define SCALE_TYPE PA_6
    #define BTN_TARE PA_5
    #define BTN_ACT PA_4
    #define RING PA_1 //SYS_WKUP1
    #define RING_ANA PA_1
    #define MIC_ANA PA_2
    #define I2C1_SCL PB_6
    #define I2C1_SDA PB_7

    #define VBAT_RATIO 4
#elif defined(STM32F103xB) //bluepill 
    #define ONEWIRE_DATA PB_0
    #define ANA_VBATT PB_1
    #define SCALE_TYPE PA_7
    #define BTN_TARE PA_6
    #define BTN_ACT PA_5
    #define RING PA_0 //WKUP
    #define RING_ANA PA_2
    #define MIC_ANA PA_3
    #define I2C1_SCL PB_10
    #define I2C1_SDA PB_11

    #define VBAT_RATIO 1 //must be define
#else
    #error "Bad µC defined"
#endif

#ifdef __cplusplus
}
#endif
#endif