/************************************************************************************//**
 *
 *	\file		rtc.c
 *
 *	\brief
 *
 *	\date		30 sept. 2019
 *
 *	\author		rvaast
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes																		                                       */
/***************************************************************************************/
#include "rtc.h"
#include "stm32yyxx_ll_rtc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     */
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
/*	Local variables                                                                    */
/***************************************************************************************/
static RTC_HandleTypeDef RtcHandle;

/***************************************************************************************/
/*	Local Functions prototypes                                                         */
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		int32_t rtc_init(void)
 *	\brief 
 *
 ***************************************************************************************/
//#include "stm32_hal_legacy.h"

int32_t rtc_init(void)
{
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
int32_t rtc_start(uint32_t u32_lsFrequency)
{
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
int32_t rtc_deinit(void)
{
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
uint32_t rtc_read(void)
{
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
int32_t rtc_write(uint32_t timestamp)
{
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
int32_t rtc_isEnabled(void)
{
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
#if 1
int32_t rtc_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */)
{
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
#else

volatile uint32_t LPTICKER_counter = 0;
volatile uint32_t LPTICKER_RTC_time = 0;

uint32_t rtc_read_lp(void)
{
    /* RTC_time_tick is the addition of the RTC time register (in second) and the RTC sub-second register
    *  This time value is breaking each 24h (= 86400s = 0x15180)
    *  In order to get a U32 continuous time information, we use an internal counter : LPTICKER_counter
    *  This counter is the addition of each spent time since last function call
    *  Current RTC time is saved into LPTICKER_RTC_time
    *  NB: rtc_read_lp() output is not the time in us, but the LPTICKER_counter (frequency LSE/4 = 8kHz => 122us)
    */
    //core_util_critical_section_enter();
    struct tm timeinfo;

    /* Since the shadow registers are bypassed we have to read the time twice and compare them until both times are the same */
    /* We don't have to read date as we bypass shadow registers */
    uint32_t Read_time = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);
    uint32_t Read_SubSeconds = (uint32_t)(RTC->SSR);

    while ((Read_time != (RTC->TR & RTC_TR_RESERVED_MASK)) || (Read_SubSeconds != (RTC->SSR))) {
        Read_time = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);
        Read_SubSeconds = (uint32_t)(RTC->SSR);
    }

    timeinfo.tm_hour = RTC_Bcd2ToByte((uint8_t)((Read_time & (RTC_TR_HT  | RTC_TR_HU))  >> 16));
    timeinfo.tm_min  = RTC_Bcd2ToByte((uint8_t)((Read_time & (RTC_TR_MNT | RTC_TR_MNU)) >> 8));
    timeinfo.tm_sec  = RTC_Bcd2ToByte((uint8_t)((Read_time & (RTC_TR_ST  | RTC_TR_SU))  >> 0));

    uint32_t RTC_time_tick = (timeinfo.tm_sec + timeinfo.tm_min * 60 + timeinfo.tm_hour * 60 * 60) * RTC_SYNCH_PREDIV + RTC_SYNCH_PREDIV - Read_SubSeconds; // Max 0x0001-517F * 8191 + 8191 = 0x2A2E-AE80

    if (LPTICKER_RTC_time <= RTC_time_tick) {
        LPTICKER_counter += (RTC_time_tick - LPTICKER_RTC_time);
    } else {
        /* When RTC time is 0h00.01 and was 11H59.59, difference is "current time + 24h - previous time" */
        LPTICKER_counter += (RTC_time_tick + 24 * 60 * 60 * RTC_SYNCH_PREDIV - LPTICKER_RTC_time);
    }
    LPTICKER_RTC_time = RTC_time_tick;

    //core_util_critical_section_exit();
    return LPTICKER_counter;
}

int32_t rtc_set_wake_up_timer(uint32_t timestamp)
{
    /* RTC periodic auto wake up timer is used
    *  This WakeUpTimer is loaded to an init value => WakeUpCounter
    *  then timer starts counting down (even in low-power modes)
    *  When it reaches 0, the WUTF flag is set in the RTC_ISR register
    */
    uint32_t WakeUpCounter;
    uint32_t WakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV4;

    //core_util_critical_section_enter();

    /* MBED API gives the timestamp value to set
    *  WakeUpCounter is then the delta between timestamp and the current tick (LPTICKER_counter)
    *  If the current tick preceeds timestamp value, max U32 is added
    */
    uint32_t current_lp_time = rtc_read_lp();
    if (timestamp < current_lp_time) {
        WakeUpCounter = 0xFFFFFFFF - current_lp_time + timestamp;
    } else {
        WakeUpCounter = timestamp - current_lp_time;
    }

    /* RTC WakeUpCounter is 16 bits
    *  Corresponding time value depends on WakeUpClock
    *  - RTC clock divided by 4  : max WakeUpCounter value is  8s (precision around 122 us)
    *  - RTC clock divided by 8  : max WakeUpCounter value is 16s (precision around 244 us)
    *  - RTC clock divided by 16 : max WakeUpCounter value is 32s (precision around 488 us)
    *  - 1 Hz internal clock 16b : max WakeUpCounter value is 18h (precision 1 s)
    *  - 1 Hz internal clock 17b : max WakeUpCounter value is 36h (precision 1 s)
    */
    if (WakeUpCounter > 0xFFFF) {
        WakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV8;
        WakeUpCounter = WakeUpCounter / 2;

        if (WakeUpCounter > 0xFFFF) {
            WakeUpClock = RTC_WAKEUPCLOCK_RTCCLK_DIV16;
            WakeUpCounter = WakeUpCounter / 2;

            if (WakeUpCounter > 0xFFFF) {
                /* Tick value needs to be translated in seconds : TICK * 16 (previous div16 value) / RTC clock (32768) */
                WakeUpClock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
                WakeUpCounter = WakeUpCounter / 2048;

                if (WakeUpCounter > 0xFFFF) {
                    /* In this case 2^16 is added to the 16-bit counter value */
                    WakeUpClock = RTC_WAKEUPCLOCK_CK_SPRE_17BITS;
                    WakeUpCounter = WakeUpCounter - 0x10000;
                }
            }
        }
    }

    RtcHandle.Instance = RTC;
    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

#if defined (RTC_WUTR_WUTOCLR) /* STM32L5 */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, WakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV4, 0) != HAL_OK) {
        return ERROR;
    }
#else /* RTC_WUTR_WUTOCLR */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, WakeUpCounter, WakeUpClock) != HAL_OK) {
        return ERROR;
    }
#endif /* RTC_WUTR_WUTOCLR */

    //NVIC_SetVector(RTC_WKUP_IRQn, (uint32_t)_RTC_IRQHandler);
    //irq_handler = (void (*)(void))lp_ticker_irq_handler;
    //NVIC_EnableIRQ(RTC_WKUP_IRQn);
    
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn); 
    
    //core_util_critical_section_exit();

    return OK;
}

int32_t rtc_enableWakeUpRtc(uint32_t u32_sleepTime /* usecond */) {
  return rtc_set_wake_up_timer(u32_sleepTime / 1000);
}


#endif

/************************************************************************************
 *
 *	\fn		void rtc_isEnabledWakeUpRtc()
 *	\brief 
 *
 ***************************************************************************************/
int32_t rtc_isEnabledWakeUpRtc()
{
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
int32_t rtc_disableWakeUpTimer(void)
{
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
void RTC_IRQHandler(void)
{
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
uint32_t rtc_backupRead( uint32_t u32_index )
{
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
int32_t rtc_backupWrite( uint32_t u32_index, uint32_t u32_value )
{
  if(u32_index <= RTC_BACKUP_MAX_INDEX)
  {
    HAL_RTCEx_BKUPWrite( &RtcHandle, u32_index, u32_value );
    return TRUE;
  }

  return FALSE;
}