/************************************************************************************//**
 *
 *	\file		system.c
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
#include "system.h"

#if 0
/***************************************************************************************/
/*	Defines		  	 	 															                                     */
/***************************************************************************************/
#define MAX_CAPTURE 5

/***************************************************************************************/
/*	Local variables                                                                    */
/***************************************************************************************/

static TIM_HandleTypeDef    TimerLsCaptureHandle;

uint16_t au16_captureValue[MAX_CAPTURE]; /* = {0, 0};*/
__IO uint32_t u32_lsFreq = 0;
__IO uint32_t u32_captureNumber = 0;

uint16_t au32_frequencyValue[MAX_CAPTURE]; /* = {0, 0};*/
__IO uint32_t u32_frequencyNumber = 0;

__IO uint32_t u32_periodValue = 0;
#endif

/***************************************************************************************/
/*	Local Functions prototypes                                                         */
/***************************************************************************************/

/************************************************************************************
 *
 *	\fn		void system_clockConfig_80MHz(void)
 *	\brief 
 *
 ***************************************************************************************/
void system_clockConfig_16MHz(void) /* 16 MHz on HSI */
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState             = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_OFF;
  /*RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM             = 2;
  RCC_OscInitStruct.PLL.PLLN             = 20;
  RCC_OscInitStruct.PLL.PLLQ             = 4;
  RCC_OscInitStruct.PLL.PLLR             = 2;*/
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType       = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}
#if 0
void system_clockConfig_100MHz(void) /* 100 MHz on HSE */
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSIState             = RCC_HSI_OFF;
  RCC_OscInitStruct.HSEState             = RCC_HSE_ON;
  RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM             = 25;
  RCC_OscInitStruct.PLL.PLLN             = 192;
  RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ             = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    system_reset();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType       = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    system_reset();
  }

  HAL_RCC_EnableCSS();
}

/************************************************************************************
 *
 *	\fn		void HAL_RCC_CSSCallback(void)
 *	\brief call when hse error detected
 *
 ***************************************************************************************/
void HAL_RCC_CSSCallback(void) {
  system_reset();
} 
#endif

#if 0
/************************************************************************************
 *
 *	\fn		void system_clockConfig_32KHz(void)
 *	\brief 
 *
 ***************************************************************************************/

void system_clockConfig_32KHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue = 0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  /* Set MSI range to 0 */
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
}
#endif

#if 1
/************************************************************************************
 *
 *	\fn		 int32_t system_getBorLevel(e_BOR_LEVEL *pe_borLevel) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t system_getBorLevel(e_BOR_LEVEL *pe_borLevel) {
	FLASH_OBProgramInitTypeDef FLASH_Handle;
	
  if(pe_borLevel == NULL)
    return HAL_ERROR;

  if(HAL_FLASH_Unlock() != HAL_OK)
		return HAL_ERROR;

  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  HAL_FLASH_OB_Unlock();

	HAL_FLASHEx_OBGetConfig(&FLASH_Handle);

  HAL_FLASH_OB_Lock();
  
  HAL_FLASH_Lock();

  *pe_borLevel = (e_BOR_LEVEL) FLASH_Handle.BORLevel;

	return HAL_OK;
}

/************************************************************************************
 *
 *	\fn		int32_t system_setBorLevel(e_BOR_LEVEL e_borLevel)
 *	\brief 
 *
 ***************************************************************************************/
int32_t system_setBorLevel(e_BOR_LEVEL e_borLevel) {
  HAL_StatusTypeDef status = HAL_ERROR;
	FLASH_OBProgramInitTypeDef FLASH_Handle;
	e_BOR_LEVEL e_currentBorLevel = OB_BOR_OFF;

  system_getBorLevel(&e_currentBorLevel);

  if(e_currentBorLevel != e_borLevel) {
		FLASH_Handle.OptionType = OPTIONBYTE_BOR;
    FLASH_Handle.BORLevel = (e_BOR_LEVEL) e_borLevel;

    if(HAL_FLASH_Unlock() != HAL_OK)
		  return HAL_ERROR;

    //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

		HAL_FLASH_OB_Unlock();
		
		HAL_FLASHEx_OBProgram(&FLASH_Handle); 

		status = HAL_FLASH_OB_Launch(); /* Generate System Reset to load the new option byte values ***************/
		
    HAL_FLASH_OB_Lock();

    HAL_FLASH_Lock();
	}
	
	return status;
}
#endif

/************************************************************************************
 *
 *	\fn		void system_reset(void) 
 *	\brief 
 *
 ***************************************************************************************/
void system_reset(void) {
  NVIC_SystemReset();
}

/************************************************************************************
 *
 *	\fn		void system_rescueReset(t_RamRet *pt_ramRet) 
 *	\brief 
 *
 ***************************************************************************************/
void system_rescueReset(t_RamRet *pt_ramRet) {
  ramret_clean(pt_ramRet);

  NVIC_SystemReset();
}

#if 0
/************************************************************************************
 *
 *	\fn		void system_printLsiCapturedFrequency(void)
 *	\brief 
 *
 ***************************************************************************************/ 
void system_printLsCapturedFrequency(void) {
  uint8_t i;

  for(i=0;i<u32_frequencyNumber;i++) {
#if (USE_LSI == 1)
    TRACE("LSI: capture %d = %d Hz", i, au32_frequencyValue[i]);
#else
    TRACE("LSE: capture %d = %d Hz", i, au32_frequencyValue[i]);
#endif   
  }  
} 

/************************************************************************************
 *
 *	\fn		uint32_t system_getLsFrequency(void)
 *	\brief 
 *
 ***************************************************************************************/ 
uint32_t system_getLsFrequency(void) {
  TIM_IC_InitTypeDef    TIMInput_Config;
  
  __HAL_RCC_TIM16_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0x0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  HAL_RCC_GetHCLKFreq();

  TimerLsCaptureHandle.Instance = TIM16;
  
  /* TIM21 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM21 CH1.
     The Rising edge is used as active edge.
     The TIM21 CCR1 is used to compute the frequency value. 
  ------------------------------------------------------------ */
  TimerLsCaptureHandle.Init.Prescaler         = 0; 
  TimerLsCaptureHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;  
  TimerLsCaptureHandle.Init.Period            = 0xFFFF; 
  TimerLsCaptureHandle.Init.ClockDivision     = 0;   
  if(HAL_TIM_IC_Init(&TimerLsCaptureHandle) != HAL_OK)
  {
#if (USE_LSI == 1)
    return LSI_VALUE;
#else
    return LSE_VALUE;
#endif
  }
  
  /* Connect internally the TIM21_CH1 Input Capture to the LSI clock output */
#if (USE_LSI == 1)
  HAL_TIMEx_RemapConfig(&TimerLsCaptureHandle, TIM_TIM16_TI1_LSI);
#else
  HAL_TIMEx_RemapConfig(&TimerLsCaptureHandle, TIM_TIM16_TI1_LSE);
#endif  

  /* Configure the Input Capture of channel 1 */
  TIMInput_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
  TIMInput_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIMInput_Config.ICPrescaler = TIM_ICPSC_DIV8;
  TIMInput_Config.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimerLsCaptureHandle, &TIMInput_Config, TIM_CHANNEL_1) != HAL_OK)
  {
#if (USE_LSI == 1)
    return LSI_VALUE;
#else
    return LSE_VALUE;
#endif
  }

  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&TimerLsCaptureHandle, TIM_CHANNEL_1) != HAL_OK)
  {
#if (USE_LSI == 1)
    return LSI_VALUE;
#else
    return LSE_VALUE;
#endif
  }

  /* Wait until the TIM21 get 2 LSI edges */
  while(u32_captureNumber != MAX_CAPTURE)
  {
  }

  HAL_TIM_IC_Stop_IT(&TimerLsCaptureHandle, TIM_CHANNEL_1);
  HAL_TIM_IC_DeInit(&TimerLsCaptureHandle);

  HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);

  __HAL_RCC_TIM16_CLK_DISABLE();

  return u32_lsFreq;
}

/************************************************************************************
 *
 *	\fn		TIM_HandleTypeDef* system_getTimerLsCaptureHandle(void)
 *	\brief 
 *
 ***************************************************************************************/ 
TIM_HandleTypeDef* system_getTimerLsCaptureHandle(void) {
  return &TimerLsCaptureHandle;
}

/************************************************************************************
 *
 *	\fn		TIM_HandleTypeDef* system_getTimerLsCaptureHandle(void)
 *	\brief 
 *
 ***************************************************************************************/ 
void system_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  uint8_t i;
  
  au16_captureValue[u32_captureNumber] = HAL_TIM_ReadCapturedValue(&TimerLsCaptureHandle, TIM_CHANNEL_1);

  if(u32_captureNumber) {
    if (au16_captureValue[u32_captureNumber-1] > au16_captureValue[u32_captureNumber]) { /* Compute the period length */
      u32_periodValue = (uint16_t)(0xFFFF - au16_captureValue[u32_captureNumber-1] + au16_captureValue[u32_captureNumber] + 1);
    } else {
      u32_periodValue  = (uint16_t)(au16_captureValue[u32_captureNumber] - au16_captureValue[u32_captureNumber-1]);
    }

    au32_frequencyValue[u32_frequencyNumber] = (uint32_t) SystemCoreClock / u32_periodValue;
    au32_frequencyValue[u32_frequencyNumber] *= 8;

    u32_frequencyNumber++;
  }

  u32_captureNumber++;

  if (u32_captureNumber >= MAX_CAPTURE) {
    u32_periodValue = 0;
    for(i=0;i<u32_frequencyNumber;i++) {
      if((au32_frequencyValue[i] >= 26000) && (au32_frequencyValue[i] <= 56000)) {
        u32_lsFreq += au32_frequencyValue[i];

        if(i) { /* dynamic average */
          u32_lsFreq /= 2;
        }
      }
    }

    if(i <= 1) { /* no average computed */
#if (USE_LSI == 1)
      u32_lsFreq = LSI_VALUE;
#else
      u32_lsFreq = LSE_VALUE;
#endif    
      
    }

    HAL_TIM_IC_Stop_IT(&TimerLsCaptureHandle, TIM_CHANNEL_1);
  }
}

/************************************************************************************
 *
 *	\fn		void TIM21_IRQHandler(void)
 *	\brief 
 *
 ***************************************************************************************/ 
void TIM1_UP_TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimerLsCaptureHandle);
}
#endif