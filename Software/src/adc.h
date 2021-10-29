#ifndef __ADC_H
#define __ADC_H

/***************************************************************************************/
/*	Includes																		   */
/***************************************************************************************/
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
 * define
 **************************************************************************************/

/**
 * \enum  Pin_ADC
 * \brief
 *
 */
typedef enum Pin_ADC_ {
#if defined(STM32F411xE) //blackpill
    EXTERNAL_RING = 1, //ADC1_IN1 -> RING_ANA
    EXTERNAL_VBAT = 8, //ADC1_IN8 -> ANA_VBATT  
#elif defined(STM32F103xB) //bluepill 
    //EXTERNAL_RING = 1, //ADC1_IN1 -> RING_ANA
    //EXTERNAL_VBAT = 8, //ADC1_IN8 -> ANA_VBATT 
#else
    #error "Bad µC defined"
#endif
    INTERNAL_TEMP = 32,
    INTERNAL_VREF,
    INTERNAL_VBAT,   

    Pin_ADC_MAX
} Pin_ADC;

/***************************************************************************************/
/*	Shared Functions																   */
/***************************************************************************************/
uint16_t AnalogRead(Pin_ADC pin);

#ifdef __cplusplus
}
#endif

#endif 
