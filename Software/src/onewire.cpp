/************************************************************************************//**
 *
 *	\file		onewire.cpp
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
#include <OneWire.h>
#include "onewire.h"
#include "power.h"
#include "trace.h"

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
OneWire ow(ONEWIRE_DATA);  
static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/***************************************************************************************
 *
 *	\fn		int32_t onewire_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
int32_t onewire_setup(t_RamRet *pt_ramRet) {
  if(pt_ramRet == NULL)
    return ERROR;

  pt_ramRet_ = pt_ramRet;

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t onewire_getData(t_telemetryData *pt_telemetryData)
 *	\brief 
 *
 ***************************************************************************************/
int32_t onewire_getData(t_telemetryData *pt_telemetryData) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte address[8];

  if(pt_telemetryData == NULL)
    return ERROR;

  TRACE("[1WIRE]");
  
  ow.reset_search();

  if ( !ow.search(address)) {
    TRACE_CrLf(" no more addresses.");
    ow.reset_search();
    return ERROR;
  }
  
  TRACE(" addr: ");
  for(i=0;i<8;i++)
    TRACE("%02x", address[i]);

  if (OneWire::crc8(address, 7) != address[7]) {
      TRACE_CrLf(", bad crc");
      return ERROR;
  }
 
  switch (address[0]) {
    case 0x10:
      TRACE(", DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      TRACE(", DS18B20");
      type_s = 0;
      break;
    case 0x22:
      TRACE(", DS1822");
      type_s = 0;
      break;
    default:
      TRACE(", unknown");
      return ERROR;
  } 

  ow.reset();
  ow.select(address);
  ow.write(0x44, 1); // start conversion, with parasite power on at the end
  
  if(power_isPoweredOn())
    delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ow.reset();
  ow.select(address);    
  ow.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ow.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  pt_telemetryData->contentInfo.details.temperatureInsideCount = 1;
  pt_telemetryData->temperatureInside[0] = (float)raw / 16.0;

  TRACE_CrLf(", temp: %0.2f C", pt_telemetryData->temperatureInside[0]);

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t onewire_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t onewire_suspend(void) {

  return OK;
}