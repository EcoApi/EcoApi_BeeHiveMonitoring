/************************************************************************************//**
 *
 *	\file		power.cpp
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
#include "power.h"
#include "system.h"
#include "ramret.h"
#include "trace.h"
#include "rtc.h"
#include "stm32yyxx_ll_rcc.h"
#include <stdio.h>

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static int32_t s32_lastState = STANDBY_NOT_RESUMED;
static uint8_t u8_independentWindowWatchdogReset = FALSE;
static uint8_t u8_lowPowerReset = FALSE;
static uint8_t u8_pinReset = FALSE;
static uint8_t u8_softwareReset = FALSE;
static uint8_t u8_windowWatchdogReset = FALSE;
static uint8_t u8_borReset = FALSE;
static uint8_t u8_poweredOn = FALSE;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static void power_stopMode(void);
static void power_standbyMode(void);
static void power_sleepMode(void);
static void power_enableWakeUpPin(uint32_t u32_wakeupPin);
static void power_disableWakeUpPin(uint32_t u32_wakeupPin);

/************************************************************************************
 *
 *	\fn		static void power_stopMode(void)
 *	\brief 
 *
 ***************************************************************************************/
#if 0
static void power_stopMode(void) {
  __disable_irq();

#ifdef __HAL_RCC_WAKEUPSTOP_CLK_CONFIG
  /* Select HSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
#endif

  /* Enter Stop mode */
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  HAL_ResumeTick();

  //system_clockConfig();

  __enable_irq();

  system_reset();
}
#endif

/************************************************************************************
 *
 *	\fn		static void power_standbyMode(void)
 *	\brief 
 *
 ***************************************************************************************/
static void power_standbyMode(void) {
  HAL_PWR_EnterSTANDBYMode();
}

#if 0
/************************************************************************************
 *
 *	\fn		static void power_sleepMode(void)
 *	\brief 
 *
 ***************************************************************************************/
static void power_sleepMode(void) {
  system_clockConfig_32KHz();
  //__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);

  /* Disable Prefetch Buffer */
  //__HAL_FLASH_PREFETCH_BUFFER_DISABLE();

  /* Enable the power down mode during Sleep mode */
  __HAL_FLASH_SLEEP_POWERDOWN_ENABLE();

  /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
  HAL_SuspendTick();
    
  /* Enter Sleep Mode , wake up is done once Key push button is pressed */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
 
  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
  HAL_ResumeTick();

  system_reset();
}
#endif

/************************************************************************************
 *
 *	\fn		static void power_enableWakeUpPin(uint32_t u32_wakeupPin)
 *	\brief 
 *
 ***************************************************************************************/ 
static void power_enableWakeUpPin(uint32_t u32_wakeupPin) {
  HAL_PWR_DisableWakeUpPin(u32_wakeupPin);

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  
  HAL_PWR_EnableWakeUpPin(u32_wakeupPin);
}

/************************************************************************************
 *
 *	\fn		static void power_disableWakeUpPin(uint32_t u32_wakeupPin)
 *	\brief 
 *
 ***************************************************************************************/
static void power_disableWakeUpPin(uint32_t u32_wakeupPin) {
  HAL_PWR_DisableWakeUpPin(u32_wakeupPin);

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

/************************************************************************************
 *
 *	\fn		int32_t power_sleep(e_WAKEUP_TYPE e_wakeupType, uint32_t u32_sleepTime, uint32_t u32_wakeupPin)
 *	\brief 
 *
 ***************************************************************************************/
int32_t power_sleep(e_SLEEP_MODE e_mode, e_WAKEUP_TYPE e_wakeupType, uint32_t u32_sleepTime, uint32_t u32_wakeupPin) {
  if(e_mode > e_SLEEP_MODE_STANDBY)
    goto error;
  
  if((e_mode == e_SLEEP_MODE_OFF) && (e_wakeupType != e_WAKEUP_TYPE_RTC))
    goto error;

  switch(e_wakeupType) {
    case e_WAKEUP_TYPE_RTC:{
      if(!u32_sleepTime)
        goto error;

      if(e_mode == e_SLEEP_MODE_OFF) {
        if(rtc_external_enableWakeUpRtc(u32_sleepTime) != OK) {
          TRACE_CrLf("[POWER] error en wakeup rtc external");
          goto error;
        }
      } else {
        if(rtc_internal_enableWakeUpRtc(u32_sleepTime * 1000 * 1000) != OK) {
          TRACE_CrLf("[POWER] error en wakeup rtc internal");
          goto error;
        }
      }
      break;  
    }
    case e_WAKEUP_TYPE_INPUT:
      if(!u32_wakeupPin)
        goto error;

      if(e_mode != e_SLEEP_MODE_SLEEP)
        power_enableWakeUpPin(u32_wakeupPin);
      else
        power_disableWakeUpPin(u32_wakeupPin);  
      break;

    case e_WAKEUP_TYPE_BOTH:
      if((!u32_sleepTime) || (!u32_wakeupPin))
        goto error;

      if(rtc_internal_enableWakeUpRtc(u32_sleepTime * 1000 * 1000) != OK) {
        TRACE_CrLf("[POWER] error en wakeup rtc internal");
        goto error;
      }
      
      if(e_mode != e_SLEEP_MODE_SLEEP)
        power_enableWakeUpPin(u32_wakeupPin);
      else
        power_disableWakeUpPin(u32_wakeupPin);  
      break;
  }

  switch(e_mode) {
    case e_SLEEP_MODE_OFF:
      digitalWrite(PWR_OFF, HIGH);
      pinMode(PWR_OFF, OUTPUT);
      
      delay(100);

      TRACE_CrLf("[POWER] error power off (motion or power on)");

      rtc_internal_disableWakeUpTimer();

      goto error;

    case e_SLEEP_MODE_STANDBY:
      //HAL_EnableDBGStandbyMode();

      power_standbyMode();

      TRACE_CrLf("[POWER] error en stanby");
      goto error;

#if 0
    case e_SLEEP_MODE_STOP:
      power_stopMode();
      break;


    case e_SLEEP_MODE_SLEEP:
      power_sleepMode();
      break;


#endif      
  }  

  system_reset();
  
  return OK;

error:
  delay(500);

  system_reset();

  return ERROR;
}

/************************************************************************************
 *
 *	\fn		int32_t power_resumeSleep(e_SLEEP_MODE e_mode, e_WAKEUP_TYPE e_wakeupType, uint32_t u32_wakeupPin)
 *	\brief 
 *
 ***************************************************************************************/
int32_t power_resumeSleep(e_SLEEP_MODE e_mode, e_WAKEUP_TYPE e_wakeupType, uint32_t u32_wakeupPin) {
  switch(e_wakeupType) {
    case e_WAKEUP_TYPE_RTC:
      break;  

    case e_WAKEUP_TYPE_INPUT:
      if(!u32_wakeupPin)
        goto error;

      if(e_mode != e_SLEEP_MODE_SLEEP)
        power_enableWakeUpPin(u32_wakeupPin);
      else
        power_disableWakeUpPin(u32_wakeupPin);
        
      break;

    case e_WAKEUP_TYPE_BOTH:
      if(!u32_wakeupPin)
        goto error;
        
      if(e_mode != e_SLEEP_MODE_SLEEP)  
        power_enableWakeUpPin(u32_wakeupPin);
      else
        power_disableWakeUpPin(u32_wakeupPin);
        
      break;
  }

  switch(e_mode) {
    case e_SLEEP_MODE_STANDBY:
      power_standbyMode();
      break;
#if 0
    case e_SLEEP_MODE_SLEEP:
      power_sleepMode();
      break;

    case e_SLEEP_MODE_STOP:
      power_stopMode();
      break;
#endif      
  }  

  system_reset();

  return OK;

error:
  system_reset();

  return ERROR;
}

/************************************************************************************
 *
 *	\fn		void power_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t power_init(void) {
  int32_t s32_ret = OK;
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_WU)) {
    s32_lastState = STANDBY_RESUMED;
    s32_ret = s32_lastState;
  }

  __HAL_PWR_GET_FLAG(PWR_FLAG_WU);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

  if(LL_RCC_IsActiveFlag_IWDGRST())
    u8_independentWindowWatchdogReset = TRUE;

  if(LL_RCC_IsActiveFlag_LPWRRST())
    u8_lowPowerReset = TRUE;

  if(LL_RCC_IsActiveFlag_PINRST())
    u8_pinReset = TRUE;

  if(LL_RCC_IsActiveFlag_SFTRST())
    u8_softwareReset = TRUE;

  if(LL_RCC_IsActiveFlag_WWDGRST())
    u8_windowWatchdogReset = TRUE;

  if(LL_RCC_IsActiveFlag_BORRST())
    u8_borReset = TRUE;

  __HAL_RCC_CLEAR_RESET_FLAGS();

  if(u8_pinReset && u8_borReset)
    u8_poweredOn = TRUE;

  return s32_ret;
}

/************************************************************************************
 *
 *	\fn		void power_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t power_getLastState(void) {
  return s32_lastState;
}

/************************************************************************************
 *
 *	\fn		uint8_t power_isPoweredOn(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t power_isPoweredOn(void) {
  return u8_poweredOn;
}

/************************************************************************************
 *
 *	\fn		uint8_t power_isWatchdogReset(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t power_isWatchdogReset(void) {
  return u8_independentWindowWatchdogReset;
}

/************************************************************************************
 *
 *	\fn		uint8_t power_isWatchdogReset(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t power_isPinReset(void) {
  return u8_pinReset;
}

/************************************************************************************
 *
 *	\fn		uint8_t power_isSoftwareReset(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t power_isSoftwareReset(void) {
  return u8_softwareReset;
}

/************************************************************************************
 *
 *	\fn		uint8_t power_traceState(void)
 *	\brief 
 *
 ***************************************************************************************/
void power_traceState(void) {
  TRACE_CrLf("[POWER] wu %d, sb %d, pwr %d, iwdgr %d, lpr %d, pr %d, sr %d, wdgr %d, br %d, po %d", 
                                                                 (int) __HAL_PWR_GET_FLAG(PWR_FLAG_WU),
                                                                 (int) __HAL_PWR_GET_FLAG(PWR_FLAG_SB),
                                                                 u8_poweredOn,
                                                                 u8_independentWindowWatchdogReset,
                                                                 u8_lowPowerReset,
                                                                 u8_pinReset,
                                                                 u8_softwareReset,
                                                                 u8_windowWatchdogReset,
                                                                 u8_borReset,
                                                                 u8_poweredOn);
}   