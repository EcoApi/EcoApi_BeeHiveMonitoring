/************************************************************************************//**
 *
 *	\file		system.cpp
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
#include "system.h"

/***************************************************************************************/
/*	Local Functions prototypes                                                         */
/***************************************************************************************/

/***************************************************************************************
 *
 *	\fn		void system_clockConfig_16MHz(void)
 *	\brief 16 MHz on HSI
 *
 ***************************************************************************************/
void system_clockConfig_16MHz(void) {
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
	e_BOR_LEVEL e_currentBorLevel = BOR_Level_0;

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