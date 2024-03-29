/************************************************************************************//**
 *
 *	\file		hx711.cpp
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
#include <HX711.h>
#include "hx711.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define LB2KG  0.45352 // livre to kg factor 0,453592
#define CALWEIGHT 2.00 //Kg
#define DEFAULT_CALIFACTOR -7050

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static HX711 scale;

#if (USE_EEPROM == 1)
static t_Eeprom *pt_eeprom_ = NULL;
#else
static t_RamRet *pt_eeprom_ = NULL;
#endif

static t_RamRet *pt_ramRet_ = NULL;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
#if (USE_EEPROM == 1)
static void hx711_calibrate(t_Eeprom *pt_eeprom, t_RamRet *pt_ramRet);
#else
static void hx711_calibrate(t_RamRet *pt_eeprom, t_RamRet *pt_ramRet);
#endif

/***************************************************************************************
 *
 *	\fn		static void hx711_calibrate(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
static void hx711_calibrate(t_Eeprom *pt_eeprom, t_RamRet *pt_ramRet) {
#else
static void hx711_calibrate(t_RamRet *pt_eeprom, t_RamRet *pt_ramRet) {
#endif

  if((pt_eeprom == NULL) || (pt_ramRet == NULL))
    return;
  
  pt_eeprom->hx711Settings.offset = 0;
  pt_eeprom->hx711Settings.calibrationFactor = DEFAULT_CALIFACTOR;  
  
  pinMode(LED_INFO, OUTPUT);
  pinMode(BTN_TARE, INPUT);

  delay(50);

  while (LOW == digitalRead(BTN_TARE)) {
    TRACE_CrLf("[HX711] start calibration, remove all weight and press <TARE> button");
    digitalWrite(LED_INFO, HIGH);
    delay(150);
    digitalWrite(LED_INFO, LOW);
    delay(150);
  };
  digitalWrite(LED_INFO, HIGH);

  TRACE_CrLf("[HX711] calibration_factor: %0.0f", pt_eeprom->hx711Settings.calibrationFactor);
  scale.set_scale(pt_eeprom->hx711Settings.calibrationFactor / LB2KG);

  if (HIGH == digitalRead(BTN_TARE)) {
    TRACE_CrLf("[HX711] clear scale");
    
    while (HIGH == digitalRead(BTN_TARE)); // wait leaves the button
    
    delay(200); //short delay
    
    scale.tare();
    pt_eeprom->hx711Settings.offset = scale.get_offset();
    TRACE_CrLf("[HX711] current offset: %d", pt_eeprom->hx711Settings.offset);

    while (LOW == digitalRead(BTN_TARE)) {
      TRACE_CrLf("[HX711] place %0.2f kg and press <TARE> button", CALWEIGHT);
      digitalWrite(LED_INFO, HIGH);
      delay(50);
      digitalWrite(LED_INFO, LOW);
      delay(50);
    };
    digitalWrite(LED_INFO, HIGH);

    TRACE_CrLf("[HX711] please wait calibration");
    
    uint8_t done = false;
    uint8_t flipDirCount = 0;
    int8_t direction = 1;
    uint8_t dirScale = 100;
    double data = abs(scale.get_units());
    double prevData = data;
  
    while (!done) {
      data = abs(scale.get_units());
      TRACE_CrLf("[HX711] data: %0.2f, abs: %0.4f, calibration_factor: %0.0f", data, abs(data - CALWEIGHT), pt_eeprom->hx711Settings.calibrationFactor);
      
      if (abs(data - CALWEIGHT) >= 0.01) {
        if (abs(data - CALWEIGHT) < abs(prevData - CALWEIGHT) && direction != 1 && data < CALWEIGHT) {
          direction = 1;
          flipDirCount++;
        } else if (abs(data - CALWEIGHT) >= abs(prevData - CALWEIGHT) && direction != -1 && data > CALWEIGHT) {
          direction = -1;
          flipDirCount++;
        }

        if (flipDirCount > 2) {
          if (dirScale != 1) {
            dirScale = dirScale / 10;
            flipDirCount = 0;
            TRACE_CrLf("[HX711] dirScale: %d", dirScale);
          }
        }
        
        pt_eeprom->hx711Settings.calibrationFactor += direction * dirScale;
        scale.set_scale(pt_eeprom->hx711Settings.calibrationFactor / LB2KG);
        
        delay(5); //short delay
         
        prevData = data; // keep old data
      } else {
        pt_ramRet->dataCommon.hx711_calibrated = TRUE;
        done = true;

        TRACE_CrLf("[HX711] stop calibration, new currentOffset: %d, new calibration_factor: %0.0f", pt_eeprom->hx711Settings.offset,
                                                                                                      pt_eeprom->hx711Settings.calibrationFactor);

        digitalWrite(LED_INFO, LOW);
        delay(250);
        digitalWrite(LED_INFO, HIGH);
        delay(250);
        digitalWrite(LED_INFO, LOW);  

        pinMode(BTN_TARE, INPUT);
        pinMode(LED_INFO, INPUT);                                                                                          
      }
    } 
  } 

  scale.set_offset(pt_eeprom->hx711Settings.offset);
}  

/***************************************************************************************
 *
 *	\fn		int32_t hx711_setup(t_RamRet *pt_ramRet)
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
int32_t hx711_setup(t_Eeprom *pt_eeprom, t_RamRet *pt_ramRet) {
#else
int32_t hx711_setup(t_RamRet *pt_eeprom, t_RamRet *pt_ramRet) {
#endif

  if((pt_eeprom == NULL) || (pt_ramRet == NULL))
    return ERROR;
  
  pt_eeprom_ = pt_eeprom;
  pt_ramRet_ = pt_ramRet;

  pinMode(HX711_RATE, OUTPUT);
  digitalWrite(HX711_RATE, HIGH); // 80 sps
  //digitalWrite(HX711_RATE, LOW); // 10 sps

  scale.begin(HX711_DT, HX711_CK);

  scale.power_up();

  if(pt_ramRet_->dataCommon.hx711_calibrated == FALSE) {  
    hx711_calibrate(pt_eeprom_, pt_ramRet_);

#if (USE_EEPROM == 1)
    eeprom_save(pt_eeprom);
#else
    ramret_save(pt_ramRet);
#endif

    pinMode(SCALE_TYPE, INPUT_PULLUP);
    pt_ramRet_->telemetryData.contentInfo.details.scaleType = digitalRead(SCALE_TYPE);
    pinMode(SCALE_TYPE, INPUT_ANALOG);

    TRACE_CrLf("[HX711] new calibration, currentOffset: %d, calibration_factor: %0.0f, scale type %d", pt_eeprom_->hx711Settings.offset,
                                                                                                       pt_eeprom_->hx711Settings.calibrationFactor,
                                                                                                       pt_ramRet_->telemetryData.contentInfo.details.scaleType);    
  } else {
    scale.set_scale(pt_eeprom_->hx711Settings.calibrationFactor / LB2KG);
    scale.set_offset(pt_eeprom_->hx711Settings.offset);

    TRACE_CrLf("[HX711] restore calibration, currentOffset: %d, calibration_factor: %0.0f, scale type %d", pt_eeprom_->hx711Settings.offset,
                                                                                                           pt_eeprom_->hx711Settings.calibrationFactor,
                                                                                                           pt_ramRet_->telemetryData.contentInfo.details.scaleType);
  }

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t hx711_getData(t_telemetryData *pt_telemetryData)
 *	\brief 
 *
 ***************************************************************************************/
int32_t hx711_getData(t_telemetryData *pt_telemetryData) {
  if(pt_telemetryData == NULL)
    return ERROR;

#if (STANDBY_ENABLE == 0)   
  scale.power_up();
#endif  

#if 0
  pt_telemetryData->weight = abs(scale.get_units());
#else
  pt_telemetryData->weight = abs(scale.get_units(2));
#endif
  // issue with abs missing at the 4th digit after the dot (bug!!)
  if (0.0000 - pt_telemetryData->weight > 0.0001) 
    pt_telemetryData->weight = 0.00; //reset to zero

  TRACE_CrLf("[HX711] weight: %0.1f kg", pt_telemetryData->weight);

  scale.power_down();

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t hx711_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t hx711_suspend(void) {

  pinMode(BTN_TARE, INPUT);
  pinMode(SCALE_TYPE, INPUT);

  //HX711_DT, HX711_CK

  //scale.power_down();

  //delayMicroseconds(60);

  pinMode(HX711_DT, INPUT);
  pinMode(HX711_CK, INPUT);

  return OK;
}