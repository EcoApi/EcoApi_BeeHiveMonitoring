/************************************************************************************//**
 *
 *	\file		audio_i2s.cpp
 *
 *	\brief
 *
 *	\date		21 feb. 2023
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes				
/***************************************************************************************/
#include "audio_i2s.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define FFT_USE_WINDOWING             1
#define FFT_USE_FILTER_HIGH           1
#define FFT_USE_FILTER_LOW            0
#define FTT_USE_FREQ_FACTOR           1
#define FTT_USE_I2S_STANDBY_OFFSET    1

#define I2S_N_U16_BY_CHANNEL          2
#define I2S_N_CHANNEL                 2
#define FFT_CYCLE                     (FTT_COUNT + (FTT_COUNT % 2))

#if (FTT_USE_I2S_STANDBY_OFFSET == 1)
  /*
    Compute :
      Use #define FFT_I2S_COMPUTE_STANDBY_OFFSET 1 for offset computation 
  */

  #if (FFT_SAMPLE_FREQ_HZ == 2000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 2000Hz, res 1.91Hz */
  #elif (FFT_SAMPLE_FREQ_HZ == 4000)
    #define FFT_I2S_STANDBY_OFFSET      (250) /* For sampling 4000Hz, res 3.79Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 8000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 8000Hz, res 7.6Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 16000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 16000Hz, res 15.20Hz */      
  #else
    //try sample 4100Hz for beep with ratio 1.95
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 16000Hz */      
  #endif

  #define FFT_I2S_STANDBY_OFFSET_SIZE   (FFT_I2S_STANDBY_OFFSET * I2S_N_U16_BY_CHANNEL * I2S_N_CHANNEL)
#else
  #define FFT_I2S_STANDBY_OFFSET_SIZE   (0)
#endif 

#define FFT_I2S_SAMPLING_SIZE         (FFT_OUTPUT_SIZE * 2)
#define FFT_I2S_BUFFER_SIZE           (((FFT_OUTPUT_SIZE * I2S_N_U16_BY_CHANNEL * I2S_N_CHANNEL) + FFT_I2S_STANDBY_OFFSET_SIZE) * FFT_CYCLE)

#if (FTT_USE_FREQ_FACTOR == 1)
  #if (FFT_SAMPLE_FREQ_HZ == 2000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9526) /* For sampling 2000Hz, res 1.91Hz */
  #elif (FFT_SAMPLE_FREQ_HZ == 4000)
    #define FFT_SAMPLE_FREQ_FACTOR      (2) /* For sampling 4000Hz, res 3.79Hz */
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

#define FFT_SAMPLE_RES_HZ             ((float32_t)(((float32_t) FFT_SAMPLE_FREQ_HZ / (float32_t) FFT_I2S_SAMPLING_SIZE) * (float32_t) FFT_SAMPLE_FREQ_FACTOR))
#define FFT_HIGH_CUTT_OFF_FREQ        40 /* Hz */
#define FFT_LOW_CUTT_OFF_FREQ         50 /* Hz */

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
#if (USE_EEPROM == 1)
static t_Eeprom *pt_eeprom_ = NULL;
#else
static t_RamRet *pt_eeprom_ = NULL;
#endif

static uint16_t i2sData[FFT_I2S_BUFFER_SIZE];
static FFT_RESULTS fftResult[FTT_COUNT];
volatile uint8_t i2s_fftPerformed;
volatile uint8_t i2s_dataCount = 0;
volatile uint8_t i2s_fftStarted;

arm_rfft_fast_instance_f32 S;

I2S_HandleTypeDef hi2s4;
DMA_HandleTypeDef hdma_spi4_rx;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static void MX_I2S4_Init(void);
static void MX_I2S4_Deinit(void);
static void MX_DMA_Init(void);
static void MX_DMA_Deinit(void);
static void MX_GPIO_Init(void); 
static void MX_GPIO_Deinit(void);

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_I2S4_Init(void) {
  hi2s4.Instance = SPI4;
  hi2s4.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s4.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s4.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s4.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s4.Init.AudioFreq = 4000U; //I2S_AUDIOFREQ_8K;
  hi2s4.Init.CPOL = I2S_CPOL_LOW; //I2S_CPOL_HIGH;
  hi2s4.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s4.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s4) != HAL_OK) {
    Error_Handler();
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_I2S4_Deinit(void) {
  if (HAL_I2S_DeInit(&hi2s4) != HAL_OK) {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  //__HAL_RCC_GPIOC_CLK_ENABLE();
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static void MX_GPIO_Deinit(void) {
  //__HAL_RCC_GPIOA_CLK_DISABLE();
  //__HAL_RCC_GPIOB_CLK_DISABLE();
  //__HAL_RCC_GPIOC_CLK_DISABLE();
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (hi2s->Instance == SPI4) {
    /** Initializes the peripherals clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 128;
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**I2S4 GPIO Configuration
     PB12    ------> I2S4_WS
     PB13    ------> I2S4_CK
     PA1     ------> I2S4_SD
     */

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2S4 DMA Init */
    /* SPI4_RX Init */
    hdma_spi4_rx.Instance = DMA2_Stream0;
    hdma_spi4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_spi4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi4_rx) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmarx, hdma_spi4_rx);
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) {
  if (hi2s->Instance == SPI4) {
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**I2S4 GPIO Configuration
    PA1     ------> I2S4_SD
    PB12     ------> I2S4_WS
    PB13     ------> I2S4_CK
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);

    /* I2S4 DMA DeInit */
    HAL_DMA_DeInit(hi2s->hdmarx);
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void I2S_DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief call all 1024 bytes
 *
 ***************************************************************************************/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (hi2s->Instance == SPI4) {
    if(++i2s_dataCount >= FTT_COUNT)
      HAL_I2S_DMAStop(&hi2s4);
  
    if(!i2s_fftStarted)
      i2s_fftStarted = TRUE;  
  }  
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief call all 2048 bytes
 *
 ***************************************************************************************/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (hi2s->Instance == SPI4) {
    i2s_dataCount++;

    HAL_I2S_DMAStop(&hi2s4);

    if(!i2s_fftStarted) 
      i2s_fftStarted = TRUE; 
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_setup(t_Eeprom *pt_eeprom)
 *	\brief 
 *
 ***************************************************************************************/
#if (USE_EEPROM == 1)
int32_t audio_i2s_setup(t_Eeprom *pt_eeprom) {
#else
int32_t audio_i2s_setup(t_RamRet *pt_eeprom) {
#endif  
  if(pt_eeprom == NULL)
    return ERROR;

  pt_eeprom_ = pt_eeprom;

  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S4_Init();

  HAL_I2S_DMAStop(&hi2s4);
  HAL_Delay(500);

  __disable_irq();
  i2s_dataCount = 0;
  i2s_fftPerformed = 0;
  i2s_fftStarted = FALSE;
  __enable_irq();

  HAL_I2S_Receive_DMA(&hi2s4, (uint16_t *) i2sData, FFT_I2S_BUFFER_SIZE / 2);

  arm_rfft_fast_init_f32(&S, FFT_OUTPUT_SIZE);

  TRACE_CrLf("[AUDIO] i2s init ok");  

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_getData(t_telemetryData *pt_telemetryData) 
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_i2s_getData(t_telemetryData *pt_telemetryData) {
  uint16_t i2sIndex;
  float32_t f32_fftSamples[FFT_OUTPUT_SIZE];
  float32_t m_fft_output_f32[FFT_OUTPUT_SIZE];
  
  float32_t dcComponent;
  uint32_t fftTimeout = 500000;
  uint32_t i, y;
  uint16_t *p_i2sData = NULL;

  while(TRUE) {
    if((i2s_fftStarted == FALSE) || (i2s_fftPerformed > FTT_COUNT)) {
      if((fftTimeout--) == 0) {
        TRACE_CrLf("[AUDIO] error fft not started");
        return ERROR; //todo qos
      } 
        
      continue;
    }  

    i2sIndex = i2s_fftPerformed * FFT_OUTPUT_SIZE;
  
    p_i2sData = (FFT_I2S_STANDBY_OFFSET_SIZE) ? &i2sData[i2sIndex + FFT_I2S_STANDBY_OFFSET_SIZE] : &i2sData[i2sIndex];

    //TRACE_CrLf("New fft %d: index %d", i2s_fftPerformed, i2sIndex);  
    //print_dataInteger("I2S samples", (uint32_t*)p_i2sData, FFT_I2S_BUFFER_SIZE / 2);
    //print_dataShort("I2S samples", (uint16_t*) p_i2sData, FFT_I2S_BUFFER_SIZE / 2);
    //print_dataU16Hex("I2S samples", p_i2sData, FFT_I2S_BUFFER_SIZE / 2);

    dcComponent = 0.0;

	  /* extract 16bits from 24 bits left i2s mono sample and compute DC component */
	  for(i = 0; i < FFT_OUTPUT_SIZE; i++ ) { 
      //uint16_t part_1_left = i2sData[((i * sizeof(uint32_t)) + i2sIndex)];
      //uint16_t part_2_left = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 1];
      uint32_t part_left = (p_i2sData[i * sizeof(uint32_t)] << 16) | p_i2sData[(i * sizeof(uint32_t)) + 1];
      int32_t part_left_signed = (int32_t) part_left >> 8;
      //int32_t part_left_signed = (int32_t) part_left >> 14; 

      //uint16_t part_1_right = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 2];
      //uint16_t part_2_right = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 3];
      //uint32_t part_right = (i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 2] << 16) | i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 3];

      f32_fftSamples[i] = (float32_t) part_left_signed;

      //TRACE_CrLf("%d: L%08x LS%08x %d", i, part_left, part_left_signed, part_left_signed);

      dcComponent += f32_fftSamples[i];
    }
    
    dcComponent = dcComponent / (float32_t) FFT_OUTPUT_SIZE;

    TRACE_CrLf("[AUDIO] fft DC component %f", dcComponent);

    /* remove dc component for each sample */
    //for(i=0;i<FFT_OUTPUT_SIZE;i++)
    //  f32_fftSamples[i] -= dcComponent;

    //print_dataFloat("i2s samples without dc component", f32_fftSamples, FFT_OUTPUT_SIZE);

#if (FFT_I2S_COMPUTE_STANDBY_OFFSET == 1)
    for(i=0;i<FFT_OUTPUT_SIZE;i++) {
      if(f32_fftSamples[i] >= -100000.0)
        continue;
    
      break;
    }

  TRACE("offset exit standby %d [index %d]\r\n", (i == FFT_OUTPUT_SIZE) ? 0 : i, i);
#endif

#if (FFT_USE_WINDOWING == 1) 
    windowing(FFT_OUTPUT_SIZE, f32_fftSamples, e_WINDOW_TYPE_HAMMING, 1.0, f32_fftSamples);

    //print_dataFloat("i2s samples after windowing", f32_fftSamples, FFT_OUTPUT_SIZE);
#endif

    arm_rfft_fast_f32(&S, f32_fftSamples, m_fft_output_f32, 0);

#if (FFT_USE_FILTER_HIGH == 1)
    //high-pass filter
    float32_t FcutHigh = FFT_HIGH_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ; 
  
    for(uint32_t i=0;i<FFT_OUTPUT_SIZE;i++) { //set frequencies <FFT_HIGH_CUTT_OFF_FREQ Hz to zero
      if(((float32_t)i < (FcutHigh * 2)) || ((float32_t)i > ((float32_t) FFT_OUTPUT_SIZE - (FcutHigh * 2)))) {
        m_fft_output_f32[i] = 0; // Real part.
      }
    }
#endif

#if (FFT_USE_FILTER_LOW == 1)
    //low-pass filter
    float32_t FcutLow = FFT_LOW_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ;

    for(uint32_t i = 0; i< FFT_OUTPUT_SIZE; i++) { //set frequencies >FFT_HIGH_CUTT_OFF_FREQ Hz to zero
      if ((((float32_t)i > (FcutLow*2)) && (i < FFT_OUTPUT_SIZE)) || ((i > FFT_OUTPUT_SIZE) &&((float32_t)i < ((float32_t)FFT_OUTPUT_SIZE - (FcutLow * 2))))) {
        m_fft_output_f32[i] = 0; // Real part.
      }
    }  
#endif

    /* Calculate the magnitude */
    arm_cmplx_mag_f32(m_fft_output_f32, m_fft_output_f32, FFT_OUTPUT_SIZE / 2);

    //print_plotterFloat(m_fft_output_f32, FFT_OUTPUT_SIZE); // full 
    //print_plotterFloat(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)
    //print_dataFloat("fft uncompressed", m_fft_output_f32, FFT_OUTPUT_SIZE / 2);
    //print_fft_min(m_fft_output_f32, FFT_OUTPUT_SIZE / 2, FFT_SAMPLE_RES_HZ); // N/2 (nyquist-theorm)
    print_fft_max(m_fft_output_f32, FFT_OUTPUT_SIZE / 2, FFT_SAMPLE_RES_HZ); // N/2 (nyquist-theorm)

    /*float32_t min_value = 0;
    uint32_t  min_val_index = 0;

    arm_min_f32(m_fft_output_f32, FFT_OUTPUT_SIZE / 2, &min_value, &min_val_index);

    for(uint32_t i=0;i<FFT_OUTPUT_SIZE / 2;i++) { 
      m_fft_output_f32[i] -= min_value;
    }*/

    /* Remove first bin correspond to the DC component */
    m_fft_output_f32[0] = 0.0; 

    /* Create new fft output and compress the data */
    fft_create_result(&fftResult[i2s_fftPerformed],
                      m_fft_output_f32,
                      pt_eeprom_->audioSettings.binCount,
                      pt_eeprom_->audioSettings.binOffset,
                      pt_eeprom_->audioSettings.binSize,
                      FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)

    //print_dataShort("fft compressed", fftResult[i2s_fftPerformed].values, fftResult[i2s_fftPerformed].bins);
    print_fft_result(&fftResult[i2s_fftPerformed], FFT_SAMPLE_RES_HZ, FFT_OUTPUT_SIZE / 2);
    //print_plotterUShort(fftResult[i2s_fftPerformed].values, fftResult[i2s_fftPerformed].bins); 

    if(++i2s_fftPerformed >= i2s_dataCount) {
      __disable_irq();
      i2s_fftStarted = FALSE;
      __enable_irq();
    }  

#if (FFT_LOOP_ENABLED == 0)
    if(i2s_fftPerformed >= FTT_COUNT)
      break;
#else
    if(i2s_fftPerformed >= FTT_COUNT) {
      fftTimeout = 500000;
      
      __disable_irq();
      i2s_dataCount = 0;
      i2s_fftStarted = FALSE; 
      __enable_irq();

      i2s_fftPerformed = 0;
      
      HAL_I2S_Receive_DMA(&hi2s4, (uint16_t *) i2sData, FFT_I2S_BUFFER_SIZE / 2);
    }
#endif
  }

  return OK;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_i2s_suspend(void) {
  MX_I2S4_Deinit();
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
uint8_t audio_i2s_getResultCount(void) {
  return i2s_fftPerformed;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
int32_t audio_i2s_getResult(FFT_RESULTS *p_fftResult, uint8_t resultIndex) {
  if((p_fftResult == NULL) || (resultIndex > i2s_fftPerformed))
    return ERROR;

  memcpy(p_fftResult, &fftResult[resultIndex], sizeof(FFT_RESULTS)); 
  
  return OK;
}