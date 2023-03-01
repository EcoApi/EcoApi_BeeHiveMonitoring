/************************************************************************************//**
 *
 *	\file		audio_adc.cpp
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
#include "audio_adc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define FFT_USE_WINDOWING             1
#define FFT_USE_FILTER_HIGH           0
#define FFT_USE_FILTER_LOW            0
#define FTT_USE_FREQ_FACTOR           1

#define FFT_ADC_SAMPLING_SIZE         (FFT_OUTPUT_SIZE * 2)
#define FFT_ADC_BUFFER_SIZE           (FFT_OUTPUT_SIZE * (FTT_COUNT + (FTT_COUNT % 2)))
#define FFT_COMPLEX_INPUT             (FFT_OUTPUT_SIZE * 2UL)

#if (FTT_USE_FREQ_FACTOR == 1)
  #if (FFT_SAMPLE_FREQ_HZ == 2000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9526) /* For sampling 2000Hz, res 1.91Hz */
  #elif (FFT_SAMPLE_FREQ_HZ == 4000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.94) /* For sampling 4000Hz, res 3.79Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 8000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9466) /* For sampling 8000Hz, res 7.6Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 16000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9451) /* For sampling 16000Hz, res 15.20Hz */      
  #else
    //try sample 4100Hz for beep with ratio 1.95
    #define FFT_SAMPLE_FREQ_FACTOR      (1.950) /* For sampling 16000Hz */      
  #endif
#else
  /*
    Apply in input 100Hz, 200Hz, 500Hz, 1000Hz, 1500Hz, 2000Hz, etc ....
    And read the frequency in trace (for exemple 54.69HZ): Max value: 15.6448 at [7] = 54.69 Hz" 
    And divide each target frequency by each measure : ex 1000 / 54.69 HZ = Ratio for 1000 Hz target
    Finish by compute average of each ratio (FFT_SAMPLE_FREQ_FACTOR = average of each ratio)
  */
  #define FFT_SAMPLE_FREQ_FACTOR      (1.0)
#endif  

#define FFT_SAMPLE_RES_HZ             ((float32_t)(((float32_t) FFT_SAMPLE_FREQ_HZ / (float32_t) FFT_ADC_SAMPLING_SIZE) * (float32_t) FFT_SAMPLE_FREQ_FACTOR))
#define FFT_HIGH_CUTT_OFF_FREQ        50 /* Hz */
#define FFT_LOW_CUTT_OFF_FREQ         50 /* Hz */

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
#if (USE_EEPROM == 1)
static t_Eeprom *pt_eeprom_ = NULL;
#else
static t_RamRet *pt_eeprom_ = NULL;
#endif

static int32_t vRef_ = 3300; /* mV */

static uint32_t adcData[FFT_ADC_BUFFER_SIZE];
static FFT_RESULTS fftResult[FTT_COUNT];
volatile uint8_t adc_fftPerformed;

volatile uint8_t adc_dataCount;
volatile uint8_t adc_fftStarted;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static void MX_ADC1_Init(void);
static void MX_ADC1_Deinit(void);
static void MX_TIM2_Init(void);
static void MX_TIM2_Deinit(void);
static void MX_DMA_Init(void);
static void MX_DMA_Deinit(void);
static void MX_GPIO_Init(void); 
static void MX_GPIO_Deinit(void);

static void _HAL_ADC_MspInit(ADC_HandleTypeDef* hadc); 
static void _HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);
static void _HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
static void _HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);

//static void _HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim);
static void _HAL_TIM_OC_DelayElapsedCallback(void);

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE; //ENABLE; /* circular or not */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  _HAL_ADC_MspInit(&hadc1); // my mspInit use for using st framework instead of arduino framework

  if(HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1; //ADC_CHANNEL_2; //
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_ADC1_Deinit(void) {
  if(HAL_ADC_DeInit(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  _HAL_ADC_MspDeInit(&hadc1); 
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (HAL_RCC_GetHCLKFreq() / FFT_SAMPLE_FREQ_HZ) - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  _HAL_TIM_Base_MspInit(&htim2); // my mspInit use for using st framework instead of arduino framework

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

#if (DEBUG_SAMPLING_FREQUENCY == 1)
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  /* fix for arduino framework !!! */
  timerObj_t *obj = get_timer_obj(&htim2);
  HardwareTimer *HT = (HardwareTimer *)(obj->__this);
  uint32_t channel;
  switch (htim2.Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1: channel = 1; break;
    case HAL_TIM_ACTIVE_CHANNEL_2: channel = 2; break;
    case HAL_TIM_ACTIVE_CHANNEL_3: channel = 3; break;
    case HAL_TIM_ACTIVE_CHANNEL_4: channel = 4; break;
    default: 
      return;
  }
  HT->attachInterrupt(channel, _HAL_TIM_OC_DelayElapsedCallback);
  /* end fix */

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);  // make sure that any pending interrupt is cleared
  HAL_NVIC_EnableIRQ(TIM2_IRQn);  

  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
#endif
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_TIM2_Deinit(void) {
  if (HAL_TIM_Base_DeInit(&htim2) != HAL_OK) {
    Error_Handler();
  }

  _HAL_TIM_Base_MspDeInit(&htim2); // my mspInit use for using st framework instead of arduino framework

#if (DEBUG_SAMPLING_FREQUENCY == 1)
  HAL_NVIC_DisableIRQ(TIM2_IRQn);  

  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
#endif
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_DMA_Deinit(void) {
  __HAL_RCC_DMA2_CLK_DISABLE();

  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

#if (DEBUG_SAMPLING_FREQUENCY == 1)
  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif  
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_GPIO_Deinit(void) {
#if (DEBUG_SAMPLING_FREQUENCY == 1)
  /*Configure GPIO pin : PA8 */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
#endif 

  //__HAL_RCC_GPIOA_CLK_DISABLE();
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void _HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* ADC1 GPIO Configuration PA2-WKUP     ------> ADC1_I2 */
    GPIO_InitStruct.Pin = GPIO_PIN_1; //GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void _HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration PA2    ------> ADC2_IN0 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 /*GPIO_PIN_2*/);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void _HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void _HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void ADC_DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void _TIM2_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim2);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief call all 1024 bytes
 *
 ***************************************************************************************/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if(++adc_dataCount >= FTT_COUNT) {
    HAL_ADC_Stop_DMA(&hadc1);
  }  
  
  if(!adc_fftStarted) {
    adc_fftStarted = TRUE;
  }

#if (DEBUG_SAMPLING_FREQUENCY == 1)  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif  
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief call all 2048 bytes
 *
 ***************************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(++adc_dataCount < FTT_COUNT) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcData[/*pwr of 2*/adc_dataCount*FFT_OUTPUT_SIZE], FFT_ADC_BUFFER_SIZE);
  }

  if(!adc_fftStarted) {
    adc_fftStarted = TRUE;
  }

#if (DEBUG_SAMPLING_FREQUENCY == 1)  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif  
}

#if (DEBUG_SAMPLING_FREQUENCY == 1)
/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
//static void _HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
static void _HAL_TIM_OC_DelayElapsedCallback(void)
{
  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}
#endif

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_setup(t_Eeprom *pt_eeprom)
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
int32_t audio_adc_setup(t_Eeprom *pt_eeprom, int32_t vRef) {
#else
int32_t audio_adc_setup(t_RamRet *pt_eeprom, int32_t vRef) {
#endif  
  if(pt_eeprom == NULL)
    return ERROR;

  pt_eeprom_ = pt_eeprom;
  vRef_ = vRef;

  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  __disable_irq();
  adc_dataCount = 0;
  adc_fftPerformed = 0;
  adc_fftStarted = FALSE;
  __enable_irq();

  HAL_TIM_Base_Start(&htim2);

#if (DEBUG_SAMPLING_FREQUENCY == 1)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1); //debug timer
#endif

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcData, FFT_ADC_BUFFER_SIZE);

  TRACE_CrLf("[AUDIO] adc init ok");  

  return OK;
}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
/***************************************************************************************
 *
 *	\fn		int32_t sht3x_getData(t_telemetryData *pt_telemetryData) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_adc_getData(t_telemetryData *pt_telemetryData) {
  uint16_t adcIndex;
  float32_t f32_fftSamples[FFT_OUTPUT_SIZE];
  float32_t m_fft_input_f32[FFT_COMPLEX_INPUT];
  float32_t m_fft_output_f32[FFT_OUTPUT_SIZE];
  const arm_cfft_instance_f32 *S;
  float32_t dcComponent;
  uint32_t fftTimeout = 500000;
  uint32_t i, y;

  while(TRUE) {
    if((adc_fftStarted == FALSE) || (adc_fftPerformed > FTT_COUNT)) {
      if((fftTimeout--) == 0) {
        TRACE_CrLf("[AUDIO] error fft not started");
        return ERROR; //todo qos
      } 
        
      continue;
    }  

    adcIndex = adc_fftPerformed * FFT_OUTPUT_SIZE;
  
    //TRACE_CrLf("new fft %d:", adc_fftPerformed);  
    //print_dataInteger("adc samples", &adcData[adcIndex], FFT_OUTPUT_SIZE);

    /* scale 12bits adc sample and compute DC component */
    dcComponent = 0.0;

    for(i=adcIndex;i<adcIndex + FFT_OUTPUT_SIZE;i++) {
      f32_fftSamples[i - adcIndex] = (float32_t) (__LL_ADC_CALC_DATA_TO_VOLTAGE(vRef_, adcData[i], LL_ADC_RESOLUTION_12B) / 1000.0);

      dcComponent += f32_fftSamples[i - adcIndex];
    }
    dcComponent = dcComponent / (float32_t) FFT_OUTPUT_SIZE;

    TRACE_CrLf("[AUDIO] fft DC component %f", dcComponent);

    /* remove dc component for each sample */
    for(i=0;i<FFT_OUTPUT_SIZE;i++)
      f32_fftSamples[i] -= dcComponent;

    //print_dataFloat("adc samples without dc component", f32_fftSamples, FFT_OUTPUT_SIZE);

#if (FFT_USE_WINDOWING == 1) 
    windowing(FFT_OUTPUT_SIZE, f32_fftSamples, e_WINDOW_TYPE_HAMMING, 1.0, f32_fftSamples);

    //print_dataFloat("adc samples after windowing", f32_fftSamples, FFT_OUTPUT_SIZE);
#endif

    /* Convert the uint32_t array containing the samples of the Audio ADC to an float array with complex numbers. The real part will be placed on the even 
     * indexes and the imaginary part will be set to 0 on all uneven indexes. This means that the complex input array is twice the size of the number of
     * samples.
     */
    for(i=0;i<FFT_COMPLEX_INPUT;i+=2) {
      y = (i) ? i / 2 : i;
    
      m_fft_input_f32[i] = f32_fftSamples[y]; // Real part.
      m_fft_input_f32[i+1] = 0; // Img part.
    }
    
    //print_plotterFloat(m_fft_input_f32, FFT_COMPLEX_INPUT);

    /* Use CFFT module to process the data.
     * Ensure that the used module is equal to the number of complex number of samples.
     */
    switch (FFT_OUTPUT_SIZE) {
      case 16:
        S = &arm_cfft_sR_f32_len16;
        break;
      case 32:
        S = &arm_cfft_sR_f32_len32;
        break;
      case 64:
        S = &arm_cfft_sR_f32_len64;
        break;
      case 128:
        S = &arm_cfft_sR_f32_len128;
        break;
      case 256:
        S = &arm_cfft_sR_f32_len256;
        break;
      case 512:
        S = &arm_cfft_sR_f32_len512;
        break;
      case 1024:
        S = &arm_cfft_sR_f32_len1024;
        break;
      case 2048:
        S = &arm_cfft_sR_f32_len2048;
        break;
      case 4096:
        S = &arm_cfft_sR_f32_len4096;
        break;
    }
  
    arm_cfft_f32(S, m_fft_input_f32, 0, 1);

#if (FFT_USE_FILTER_HIGH == 1)
    //high-pass filter
    float32_t FcutHigh = FFT_HIGH_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ; 
  
    for(uint32_t i=0;i<FFT_COMPLEX_INPUT;i+=2) { //set frequencies <FFT_HIGH_CUTT_OFF_FREQ Hz to zero
      if(((float32_t)i < (FcutHigh * 2)) || ((float32_t)i > ((float32_t) FFT_COMPLEX_INPUT - (FcutHigh * 2)))) {
        m_fft_input_f32[i] = 0; // Real part.
        m_fft_input_f32[i+1] = 0; // Img part.
      }
    }
#endif

#if (FFT_USE_FILTER_LOW == 1)
    //low-pass filter
    float32_t FcutLow = FFT_LOW_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ;

    for(uint32_t i = 0; i< FFT_COMPLEX_INPUT; i+=2) { //set frequencies >FFT_HIGH_CUTT_OFF_FREQ Hz to zero
      if ((((float32_t)i > (FcutLow*2)) && (i < FFT_OUTPUT_SIZE)) || ((i > FFT_OUTPUT_SIZE) &&((float32_t)i < ((float32_t)FFT_COMPLEX_INPUT - (FcutLow * 2))))) {
        m_fft_input_f32[i] = 0; // Real part.
        m_fft_input_f32[i+1] = 0; // Img part.
      }
    }  
#endif

    /* Calculate the magnitude */
    arm_cmplx_mag_f32(m_fft_input_f32, m_fft_output_f32, FFT_OUTPUT_SIZE);

    /* Remove first bin correspond to the DC component */
    m_fft_output_f32[0] = 0.0; 

    //print_plotterFloat(m_fft_output_f32, FFT_OUTPUT_SIZE); // full 
    //print_plotterFloat(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)
    //print_dataFloat("fft uncompressed", m_fft_output_f32, FFT_OUTPUT_SIZE / 2);
    print_fft_max(m_fft_output_f32, FFT_OUTPUT_SIZE / 2, FFT_SAMPLE_RES_HZ); // N/2 (nyquist-theorm)

    /* Create new fft output and compress the data */
    fft_create_result(&fftResult[adc_fftPerformed],
                      m_fft_output_f32,
                      pt_eeprom_->audioSettings.binCount,
                      pt_eeprom_->audioSettings.binOffset,
                      pt_eeprom_->audioSettings.binSize,
                      FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)

    //print_dataShort("fft compressed", fftResult[adc_fftPerformed].values, fftResult[adc_fftPerformed].bins);
    print_fft_result(&fftResult[adc_fftPerformed], FFT_SAMPLE_RES_HZ, FFT_OUTPUT_SIZE / 2);
    //print_plotterUShort(fftResult[adc_fftPerformed].values, fftResult[adc_fftPerformed].bins); 


    if(++adc_fftPerformed >= adc_dataCount) {
      __disable_irq();
      adc_fftStarted = FALSE;
      __enable_irq();
    }  

#if (FFT_LOOP_ENABLED == 0)
    if(adc_fftPerformed >= FTT_COUNT)
      break;
#else
    if(adc_fftPerformed >= FTT_COUNT) {
      fftTimeout = 500000;
      
      __disable_irq();
      adc_dataCount = 0;
      __enable_irq();

      adc_fftPerformed = 0;
      adc_fftStarted = FALSE; 
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcData, FFT_ADC_BUFFER_SIZE);
    }
#endif
  }

  return OK;
}
//#pragma GCC pop_options

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_adc_suspend(void) {
  MX_TIM2_Deinit();  
  MX_ADC1_Deinit();
  MX_DMA_Deinit();
  MX_GPIO_Deinit();

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
uint8_t audio_adc_getResultCount(void) {
  return adc_fftPerformed;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_adc_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex) {
  if((p_fftResult == NULL) || (resultIndex > adc_fftPerformed))
    return ERROR;

  memcpy(p_fftResult, &fftResult[resultIndex], sizeof(FFT_RESULTS)); 
  
  return OK;
}
