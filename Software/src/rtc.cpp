/************************************************************************************//**
 *
 *	\file		rtc.cpp
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
#include "rtc.h"
#include "stm32yyxx_ll_rtc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
//see 1.1.4 RTC clock configuration in /doc/en.DM00226326.pdf
// Note: LSI is around (37KHz), these dividers should work either way
// ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)
// modify RTC_ASYNCH_PREDIV & RTC_SYNCH_PREDIV in board/<BN>/mpconfigport.h to change sub-second ticks
// default is 3906.25 us, min is ~30.52 us (will increase Ivbat by ~500nA)
//#if (USE_LSI == 1)
  #define RTC_ASYNCH_PREDIV    0x7F
  #define RTC_SYNCH_PREDIV     0xF9
  //#define RTC_SYNCH_PREDIV     0x0130
//#else
//  #define RTC_ASYNCH_PREDIV  0x7F
//  #define RTC_SYNCH_PREDIV   0x00FF
//#endif

#define RTC_CLOCK LSI_VALUE
#define RTC_CLOCK_US (((uint64_t)RTC_CLOCK << 32 ) / 1000000)

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
static RTC_HandleTypeDef RtcHandle;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		int32_t rtc_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_init(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

#if 1
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    return ERROR;
  }

  __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    return ERROR;
  }
#else
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    return ERROR;
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    return ERROR;
  }
#endif


#ifdef __HAL_RCC_RTCAPB_CLK_ENABLE
  __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif

  __HAL_RCC_RTC_ENABLE();

  RtcHandle.Instance = RTC;
  //RtcHandle.State = HAL_RTC_STATE_RESET;
#if defined(STM32F1)
  RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
#else  
  /* Set the RTC time base to 1s */
  /* Configure RTC prescaler and RTC data registers as follow:
  - Hour Format = Format 24
  - Asynch Prediv = Value according to source clock
  - Synch Prediv = Value according to source clock
  - OutPut = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//#if defined (RTC_OUTPUT_REMAP_NONE)
//  RtcHandle.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
//#endif /* defined (RTC_OUTPUT_REMAP_NONE) */
//#if defined (RTC_OUTPUT_PULLUP_NONE)
//  RtcHandle.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;
//#endif /* defined (RTC_OUTPUT_PULLUP_NONE) */
#endif

  //__HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK) {
      return ERROR;
  }

#if !defined(STM32F1)
  /* STM32F1 : there are no shadow registers */
  /* STM32F2 : shadow registers can not be bypassed */
  if (HAL_RTCEx_EnableBypassShadow(&RtcHandle) != HAL_OK) {
    return ERROR;
  }
#endif

  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_init(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_start(uint32_t u32_lsFrequency) {
#if 0
  RtcHandle.Instance = RTC;
  //RtcHandle.State = HAL_RTC_STATE_RESET;
#if defined(STM32F1)
  RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
#else   
  /* Set the RTC time base to 1s */
  /* Configure RTC prescaler and RTC data registers as follow:
  - Hour Format = Format 24
  - Asynch Prediv = Value according to source clock
  - Synch Prediv = Value according to source clock
  - OutPut = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
#endif

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK) {
      return ERROR;
  }

  if(HAL_RTC_WaitForSynchro(&RtcHandle) != HAL_OK) {
    
  }

#if defined(STM32L412xx)
  if (HAL_RTCEx_SetLowPowerCalib(&RtcHandle, RTC_LPCAL_SET) != HAL_OK) {
    return ERROR;
  }
#endif
#endif

  if (!rtc_isEnabled()) {
    rtc_write(0);
  }

  return OK;
}

/************************************************************************************
 *
 *	\fn		int32_t rtc_deinit(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_deinit(void) {
  if(HAL_RTC_DeInit(&RtcHandle) != HAL_OK) {
    return ERROR;
  }
  
  HAL_PWR_DisableBkUpAccess();
    
  __HAL_RCC_RTC_DISABLE();

  return OK;
}

/************************************************************************************
 *
 *	\fn		time_t rtc_read(void) 
 *	\brief 
 *
 ***************************************************************************************/
uint32_t rtc_read(void) {
  RTC_DateTypeDef dateStruct = {0};
  RTC_TimeTypeDef timeStruct = {0};
  struct tm timeinfo;
  time_t timestamp;
 
#if defined(STM32F1) 
  RtcHandle.Instance = RTC;
  return RTC_ReadTimeCounter(&RtcHandle);
#else
    // Read actual date and time
    // Warning: the time must be read first!
    HAL_RTC_GetTime(&RtcHandle, &timeStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &dateStruct, RTC_FORMAT_BIN);
 
    // Setup a tm structure based on the RTC
    /* tm_wday information is ignored by _rtc_maketime */
    /* tm_isdst information is ignored by _rtc_maketime */
    timeinfo.tm_mon  = dateStruct.Month - 1;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year + 68;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
 
    if (_rtc_maketime(&timeinfo, &timestamp, RTC_4_YEAR_LEAP_YEAR_SUPPORT) == false) {
      return 0;
    }
 
    return (uint32_t) timestamp;
#endif    
}

/************************************************************************************
 *
 *	\fn		void rtc_write(time_t timestamp)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_write(uint32_t timestamp) {
#if defined(STM32F1) 
  RtcHandle.Instance = RTC;
  if (RTC_WriteTimeCounter(&RtcHandle, t) != HAL_OK) {
    return ERROR;
  }
#else
  RTC_DateTypeDef dateStruct = {0};
  RTC_TimeTypeDef timeStruct = {0};

  struct tm timeinfo;
  if (_rtc_localtime((time_t) timestamp, &timeinfo, RTC_4_YEAR_LEAP_YEAR_SUPPORT) == FALSE) {
    return ERROR;
  }
 
  // Fill RTC structures
  if (timeinfo.tm_wday == 0) {
    dateStruct.WeekDay    = 7;
  } else {
    dateStruct.WeekDay    = timeinfo.tm_wday;
  }
  
  dateStruct.Month          = timeinfo.tm_mon + 1;
  dateStruct.Date           = timeinfo.tm_mday;
  dateStruct.Year           = timeinfo.tm_year - 68;
  timeStruct.Hours          = timeinfo.tm_hour;
  timeStruct.Minutes        = timeinfo.tm_min;
  timeStruct.Seconds        = timeinfo.tm_sec;
 
//#if !(TARGET_STM32F1)
  timeStruct.TimeFormat     = RTC_HOURFORMAT_24;
  timeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  timeStruct.StoreOperation = RTC_STOREOPERATION_RESET;
//#endif /* TARGET_STM32F1 */
 
  if (HAL_RTC_SetDate(&RtcHandle, &dateStruct, RTC_FORMAT_BIN) != HAL_OK) {
    return ERROR;
  }

  if (HAL_RTC_SetTime(&RtcHandle, &timeStruct, RTC_FORMAT_BIN) != HAL_OK) {
    return ERROR;
  }
#endif

  return OK;
}
 
/************************************************************************************
 *
 *	\fn		int32_t rtc_isEnabled(void) 
 *	\brief 
 *
 ***************************************************************************************/ 
int32_t rtc_isEnabled(void) {
#if defined (RTC_FLAG_INITS) /* all STM32 except STM32F1 */
  return LL_RTC_IsActiveFlag_INITS(RTC);
#else /* RTC_FLAG_INITS */ /* TARGET_STM32F1 */
  return ((RTC->CRL & RTC_CRL_RSF) ==  RTC_CRL_RSF);
#endif /* RTC_FLAG_INITS */
}

/************************************************************************************
 *
 *	\fn		void rtc_enableWakeUpRtc(uint32_t u32_sleepTime)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */) {
  uint32_t u32_wakeUpCounter, u32_wakeUpClock;

  /*The Following Wakeup sequence is highly recommended prior to each Standby mode entry
     mainly  when using more than one wakeup source this is to not miss any wakeup event.
       - Disable all used wakeup sources,
       - Clear all related wakeup flags, 
       - Re-enable all used wakeup sources,
       - Enter the Standby mode.
     */
  if(HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle) != HAL_OK) {
    return ERROR;
  }  

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* Ex for Wakeup period resolution with RTCCLK=32768 Hz :
   *    RTCCLK_DIV2: ~122us < wakeup period < ~4s
   *    RTCCLK_DIV4: ~244us < wakeup period < ~8s
   *    RTCCLK_DIV8: ~488us < wakeup period < ~16s
   *    RTCCLK_DIV16: ~976us < wakeup period < ~32s
   *    CK_SPRE_16BITS: 1s < wakeup period < (0xFFFF+ 1) x 1 s = 65536 s (18 hours)
   *    CK_SPRE_17BITS: 18h+1s < wakeup period < (0x1FFFF+ 1) x 1 s = 131072 s (36 hours)
   */
  if (u32_sleepTime < (0x10000 * 2 / RTC_CLOCK * 1000000) ) { // (0xFFFF + 1) * RTCCLK_DIV2 / RTC_CLOCK * 1s
    u32_wakeUpCounter = (((uint64_t)u32_sleepTime * RTC_CLOCK_US) >> 32) >> 1;
    u32_wakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV2;
  } else if (u32_sleepTime < (0x10000 * 4 / RTC_CLOCK * 1000000) ) {
    u32_wakeUpCounter = (((uint64_t)u32_sleepTime * RTC_CLOCK_US) >> 32) >> 2;
    u32_wakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV4;
  } else if (u32_sleepTime < (0x10000 * 8 / RTC_CLOCK * 1000000) ) {
    u32_wakeUpCounter = (((uint64_t)u32_sleepTime * RTC_CLOCK_US) >> 32) >> 3;
    u32_wakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV8;
  } else if (u32_sleepTime < (0x10000 * 16 / RTC_CLOCK * 1000000) ) {
    u32_wakeUpCounter = (((uint64_t)u32_sleepTime * RTC_CLOCK_US) >> 32) >> 4;
    u32_wakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV16;
  } else {
    u32_wakeUpCounter = (u32_sleepTime / 1000000) ;
    u32_wakeUpClock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
  }

  /*Re-enable all used wakeup sources*/
    /*## Setting the Wake up time ############################################*/
  /*  RTC Wakeup Interrupt Generation:
      Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
      Wakeup Time = Wakeup Time Base * WakeUpCounter 
                  = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
      ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

      To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
        RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
        Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
        Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
        ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
#if defined (RTC_WUTR_WUTOCLR) /* STM32L5 */
  if(HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, u32_wakeUpCounter, u32_wakeUpClock, 0) != HAL_OK)
#else 
  if(HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, u32_wakeUpCounter, u32_wakeUpClock) != HAL_OK)
#endif
  {  
    return ERROR;
  } 

  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn); 

  return OK;  
}

/************************************************************************************
 *
 *	\fn		void rtc_isEnabledWakeUpRtc()
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_isEnabledWakeUpRtc(void) {
  uint8_t u8_isRTCWakeUpEnabled =  (RtcHandle.Instance->CR & (RTC_CR_WUTE)) ? 1 : 0;
  uint8_t u8_isRTCWakeUpEnabledIT = (RtcHandle.Instance->CR & (RTC_IT_WUT)) ? 1 : 0;

  if( u8_isRTCWakeUpEnabled && u8_isRTCWakeUpEnabledIT )
    return TRUE;

  return FALSE;  
}

/************************************************************************************
 *
 *	\fn		void rtc_desableWakeUpTimer(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_disableWakeUpTimer(void) {
  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);
  NVIC_DisableIRQ(RTC_WKUP_IRQn);

  return OK;
}

/************************************************************************************
 *
 *	\fn		void RTC_IRQHandler(void) 
 *	\brief 
 *
 ***************************************************************************************/
void RTC_IRQHandler(void) {
  HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandle);

#ifdef __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG
  __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
#endif  
}

/************************************************************************************
 *
 *	\fn		uint32_t rtc_backupRead( uint32_t u32_index ) 
 *	\brief 
 *
 ***************************************************************************************/
uint32_t rtc_backupRead(uint32_t u32_index) {
  if(u32_index <= RTC_BACKUP_MAX_INDEX)
    return HAL_RTCEx_BKUPRead(&RtcHandle, u32_index);
  
  return 0;
}

/************************************************************************************
 *
 *	\fn		rtc_backupWrite( uint32_t u32_index, uint32_t u32_value ) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_backupWrite( uint32_t u32_index, uint32_t u32_value ) {
  if(u32_index <= RTC_BACKUP_MAX_INDEX) {
    HAL_RTCEx_BKUPWrite( &RtcHandle, u32_index, u32_value );
    return TRUE;
  }

  return FALSE;
}