/************************************************************************************//**
 *
 *	\file		eeprom.cpp
 *
 *	\brief
 *
 *	\date		21 jan. 2022
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes				
/***************************************************************************************/
#define USE_FRAM 1

#include "eeprom.h"

#include <I2C_eeprom.h>

#include "trace.h"

#if (USE_EEPROM == 1)

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define EEPROM_MAGIC              0x6550726d //ePrm

#define EEPROM_START_ADR 0

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
I2C_eeprom eeprom(0x50, I2C_DEVICESIZE_24LC32);

static bool eepromIsNew = false;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		int32_t ramret_init(t_Eeprom *pt_eeprom)
 *	\brief 
 *
 ***************************************************************************************/
int32_t eeprom_init(t_Eeprom *pt_eeprom) {
  if(pt_eeprom == NULL)
    return ERROR;
  
  if(!eeprom.begin())
    return ERROR;

  if(0 == eeprom.readBlock(EEPROM_START_ADR, (uint8_t *) pt_eeprom, sizeof(t_Eeprom)))
    return ERROR;  

  //TRACE_DUMP("[EEPROM] read : ", (uint8_t *) pt_eeprom, sizeof(t_Eeprom));

  if(EEPROM_MAGIC != pt_eeprom->magicStart || EEPROM_MAGIC != pt_eeprom->magicEnd) {
    memset(pt_eeprom, 0, sizeof(t_Eeprom)); 

    eepromIsNew = true;
    
    /* audio */
    pt_eeprom->audioSettings.binOffset = AUDIO_DEFAULT_BIN_OFFSET;
    pt_eeprom->audioSettings.binSize = AUDIO_DEFAULT_BIN_SIZE;
    pt_eeprom->audioSettings.binCount = AUDIO_MAX_BINS;
    pt_eeprom->audioSettings.samplingFrequency = AUDIO_DEFAULT_SAMPLING_FREQUENCY;
    // gain
    // other audio

    /* common */
    pt_eeprom->sendFrequency = DEFAULT_SEND_FREQUENCY;

    pt_eeprom->magicStart = EEPROM_MAGIC;
    pt_eeprom->magicEnd = EEPROM_MAGIC;

    if(OK != eeprom_save(pt_eeprom))
      return ERROR;
  }  

  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t eeprom_save(t_Eeprom *pt_eeprom)
 *	\brief 
 *
 ***************************************************************************************/
int32_t eeprom_save(t_Eeprom *pt_eeprom) {
  t_Eeprom t_eepromTmp;
  
  if(pt_eeprom == NULL)
    return ERROR;

  if(0 != eeprom.setBlock(EEPROM_START_ADR, 0x00, sizeof(t_Eeprom)))
    return ERROR;

  if(0 != eeprom.writeBlock(EEPROM_START_ADR, (uint8_t *) pt_eeprom, sizeof(t_Eeprom)))
    return ERROR;

  if(0 == eeprom.readBlock(EEPROM_START_ADR, (uint8_t *) &t_eepromTmp, sizeof(t_Eeprom)))
    return ERROR;

  
  //TRACE_DUMP("[EEPROM] write : ", (uint8_t *) pt_eeprom, sizeof(t_Eeprom));
  //TRACE_DUMP("[EEPROM] read : ", (uint8_t *) &t_eepromTmp, sizeof(t_Eeprom));

  for(int i=0;i<sizeof(t_Eeprom);i++) {
    if(((uint8_t*) pt_eeprom)[i] != ((uint8_t*) &t_eepromTmp)[i]) {
      return ERROR;
    }  
  }

  TRACE_CrLf("[EEPROM] save ok");

  return OK;
}

/************************************************************************************
 *
 *	\fn		bool eeprom_isNew(void)
 *	\brief 
 *
 ***************************************************************************************/
bool eeprom_isNew(void) {
  return eepromIsNew;
}

#endif