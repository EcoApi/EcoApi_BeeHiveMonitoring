/************************************************************************************//**
 *
 *	\file		ramret.cpp
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes				
/***************************************************************************************/
#include "ramret.h"

#include <FRAM_MB85RC_I2C.h>
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define RAMRET_MAGIC              0x65526574 //eRet

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
FRAM_MB85RC_I2C fram(MB85RC_ADDRESS_A100, false, 0, 4);

static uint8_t u8_ramRetIsNew = FALSE;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		uint8_t ramret_init(t_RamRet *pt_ramRet, bool init)
 *	\brief 
 *
 ***************************************************************************************/
int32_t ramret_init(t_RamRet *pt_ramRet, bool init) {
	byte result;

  if(0 != fram.begin())
    return ERROR;

  if(0 != fram.readArray(0, sizeof(t_RamRet), (uint8_t*) pt_ramRet))
    return ERROR;

  //TRACE_DUMP("[FRAM] read : ", (uint8_t*) pt_ramRet, sizeof(t_RamRet));

  if((init == true) || (RAMRET_MAGIC != pt_ramRet->u32_magicStart) || (RAMRET_MAGIC != pt_ramRet->u32_magicEnd)) {
    memset(pt_ramRet, 0, sizeof(t_RamRet)); 

    u8_ramRetIsNew = TRUE;
    
    /* lora */
    pt_ramRet->loraSettings.waitDownlinkMaxCycle = LORA_DEFAULT_MAX_WAIT_DOWNLINK_CYCLE;
    
    /* other */
#if (USE_EEPROM == 0) 
   /* audio */
    pt_ramRet->audioSettings.binOffset = AUDIO_DEFAULT_BIN_OFFSET;
    pt_ramRet->audioSettings.binSize = AUDIO_DEFAULT_BIN_SIZE;
    pt_ramRet->audioSettings.binCount = AUDIO_MAX_BINS;
    pt_ramRet->audioSettings.samplingFrequency = AUDIO_DEFAULT_SAMPLING_FREQUENCY;
    // gain
    // other audio

    /* common */
    pt_ramRet->sendFrequency = DEFAULT_SEND_FREQUENCY;
#endif

    pt_ramRet->dataCommon.lastSendMotionOrPowerTime = 0;
    pt_ramRet->dataCommon.lastUpdateTime = 0;
    pt_ramRet->dataCommon.lastSendTime = 0;
    pt_ramRet->dataCommon.hx711_calibrated = false;

    pt_ramRet->dataCommon.sendAudioDataMaxCycle = AUDIO_DEFAULT_MAX_SEND_DATA_CYCLE;

    /* common */
    pt_ramRet->u32_magicStart = RAMRET_MAGIC;
    pt_ramRet->u32_magicEnd = RAMRET_MAGIC;
  }  

  return OK;
}

#include "utils.h"

/************************************************************************************
 *
 *	\fn		int32_t ramset_save(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t ramret_save(t_RamRet *pt_ramRet) {
  t_RamRet t_ramRetTmp;
  byte result;
  
  if(pt_ramRet == NULL)
    return ERROR;

  if(0 != fram.writeArray(0, sizeof(t_RamRet), (uint8_t*) pt_ramRet)) {
    TRACE_CrLf("[RRAM] save ko, write");
    return ERROR;
  }

 
  if(0 != fram.readArray(0, sizeof(t_RamRet), (uint8_t*) &t_ramRetTmp)) {
    TRACE_CrLf("[RRAM] save ko, read");
    return ERROR;
  }

  for(int i=0;i<sizeof(t_RamRet);i++) {
    if(((uint8_t*) pt_ramRet)[i] != ((uint8_t*) &t_ramRetTmp)[i]) {
      TRACE_DUMP("[FRAM] write : ", (uint8_t*) pt_ramRet, sizeof(t_RamRet));
      TRACE_DUMP("[FRAM] verify : ", (uint8_t*) &t_ramRetTmp, sizeof(t_RamRet));

      TRACE_CrLf("[RRAM] save ko, verify");
      return ERROR;
    }  
  }

  TRACE_CrLf("[RRAM] save ok");

  return OK;
}

/************************************************************************************
 *
 *	\fn		uint8_t ramret_isNew(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t ramret_isNew(void) {
  return u8_ramRetIsNew;
}

/************************************************************************************
 *
 *	\fn		void ramret_clean(t_RamRet *pt_ramRet) 
 *	\brief 
 *
 ***************************************************************************************/
void ramret_clean(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return;
  
  pt_ramRet->u32_magicStart = 0xffffffff;
  pt_ramRet->u32_magicEnd = 0xffffffff;
  ramret_save(pt_ramRet);

  TRACE_CrLf("[RRAM] save ok");
}