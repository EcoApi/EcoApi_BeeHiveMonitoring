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
#include "rtc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
//#define RAMRET_MAGIC              0x52526574 //RRet
#define RAMRET_MAGIC              0x65526574 //eRet

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static uint8_t u8_ramRetIsNew = FALSE;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		uint8_t ramret_init(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t ramret_init(t_RamRet *pt_ramRet) {
  uint32_t *pu32_rawRamRetData = (uint32_t*)(void*)pt_ramRet;
  uint32_t u32_i, u32_maxBackupRegister = sizeof(t_RamRet) / sizeof(uint32_t);

  if((pt_ramRet != NULL) && (u32_maxBackupRegister <= RTC_BACKUP_MAX_INDEX)) {
    for(u32_i=0;u32_i<u32_maxBackupRegister;u32_i++)
      pu32_rawRamRetData[u32_i] = rtc_backupRead(u32_i);
  } else
    pt_ramRet->isUsed = 0;

  if(0 == pt_ramRet->isUsed || RAMRET_MAGIC != pt_ramRet->u32_magic) {
    memset(pt_ramRet, 0, sizeof(t_RamRet)); 

    u8_ramRetIsNew = TRUE;
    
    /* lora */
    pt_ramRet->loraSettings.waitDownlinkMaxCycle = LORA_DEFAULT_MAX_WAIT_DOWNLINK_CYCLE;
    
    /* audio */
    pt_ramRet->audioSettings.binOffset = AUDIO_DEFAULT_BIN_OFFSET;
    pt_ramRet->audioSettings.binSize = AUDIO_DEFAULT_BIN_SIZE;
    pt_ramRet->audioSettings.binCount = AUDIO_MAX_BINS;
    pt_ramRet->audioSettings.sendDataMaxCycle = AUDIO_DEFAULT_MAX_SEND_DATA_CYCLE;
    pt_ramRet->audioSettings.samplingFrequency = AUDIO_DEFAULT_SAMPLING_FREQUENCY;
    // gain
    // other audio

    /* common */
    pt_ramRet->sendFrequency = DEFAULT_SEND_FREQUENCY;

    pt_ramRet->isUsed = TRUE;
    pt_ramRet->u32_magic = RAMRET_MAGIC;
    
    return TRUE;
  }  

  return FALSE;
}

/************************************************************************************
 *
 *	\fn		void ramset_save(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
void ramret_save(t_RamRet *pt_ramRet) {
  uint32_t *pu32_rawRamRetData = (uint32_t*)(void*)pt_ramRet;
  uint32_t u32_i, u32_maxBackupRegister = sizeof(t_RamRet) / sizeof(uint32_t);

  if(pt_ramRet == NULL)
    return;

  if(u32_maxBackupRegister > RTC_BACKUP_MAX_INDEX)
    return;

  rtc_backupWrite(u32_maxBackupRegister - 1, 0); //Erase the ramret magic

  for(u32_i=0;u32_i<u32_maxBackupRegister;u32_i++)
    rtc_backupWrite(u32_i, pu32_rawRamRetData[u32_i]);
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
  
  pt_ramRet->u32_magic = 0xffffffff;
  ramret_save(pt_ramRet);
}