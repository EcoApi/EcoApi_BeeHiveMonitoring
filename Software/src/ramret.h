#ifndef __RAMRET_H
#define __RAMRET_H

/***************************************************************************************/
/*	Includes																		   */
/***************************************************************************************/
#include <stdint.h>
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
 * define
 **************************************************************************************/
typedef struct t_RamRet_ {
    uint8_t isUsed:1;
    uint8_t hx711_calibrated:1;
    uint8_t timeUpdated:1;
    uint8_t traceEnable:1;
    uint8_t reserved:4;

    uint8_t boot;
    long hx711_offset;
    float hx711_calibrationFactor; 

    uint32_t LORA_seqnoUp;
    uint32_t LORA_seqnoDown;

    uint32_t u32_magic;
} t_RamRet;

/***************************************************************************************/
/*	Shared Functions																   */
/***************************************************************************************/  
uint8_t ramret_init(t_RamRet *pt_ramRet);
void ramret_save(t_RamRet *pt_ramRet);
void ramret_dump(t_RamRet *pt_ramRet);
uint8_t ramret_isNew(void);
void ramret_clean(t_RamRet *pt_ramRet);

#ifdef __cplusplus
}
#endif

#endif