/************************************************************************************//**
 *
 *	\file		adc.cpp
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
#include "adc.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#ifdef HAL_ADC_MODULE_ENABLED

#ifndef ADC_SAMPLINGTIME
#if defined(ADC_SAMPLETIME_8CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_8CYCLES_5;
#elif defined(ADC_SAMPLETIME_12CYCLES)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_12CYCLES;
#elif defined(ADC_SAMPLETIME_12CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_12CYCLES_5;
#elif defined(ADC_SAMPLETIME_13CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_13CYCLES_5;
#elif defined(ADC_SAMPLETIME_15CYCLES)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_15CYCLES;
#elif defined(ADC_SAMPLETIME_16CYCLES)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_16CYCLES;
#elif defined(ADC_SAMPLETIME_19CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_19CYCLES_5;
#endif
#endif /* !ADC_SAMPLINGTIME */

#if defined(ADC_VER_V5_V90) && !defined(ADC3_SAMPLINGTIME)
#define ADC3_SAMPLINGTIME       ADC3_SAMPLETIME_24CYCLES_5;
#endif

#if defined(ADC4_SAMPLETIME_19CYCLES_5) && !defined(ADC4_SAMPLINGTIME)
#define ADC4_SAMPLINGTIME       ADC4_SAMPLETIME_19CYCLES_5;
#endif

/*
 * Minimum ADC sampling time is required when reading
 * internal channels so set it to max possible value.
 * It can be defined more precisely by defining:
 * ADC_SAMPLINGTIME_INTERNAL
 * to the desired ADC sample time.
 */
#ifndef ADC_SAMPLINGTIME_INTERNAL
#if defined(ADC_SAMPLETIME_480CYCLES)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_480CYCLES
#elif defined(ADC_SAMPLETIME_384CYCLES)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_384CYCLES
#elif defined(ADC_SAMPLETIME_810CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_810CYCLES_5
#elif defined(ADC_SAMPLETIME_814CYCLES)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_814CYCLES
#elif defined(ADC_SAMPLETIME_640CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_640CYCLES_5
#elif defined(ADC_SAMPLETIME_601CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_601CYCLES_5
#elif defined(ADC_SAMPLETIME_247CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_247CYCLES_5
#elif defined(ADC_SAMPLETIME_239CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_239CYCLES_5
#elif defined(ADC_SAMPLETIME_160CYCLES_5)
#define ADC_SAMPLINGTIME_INTERNAL ADC_SAMPLETIME_160CYCLES_5
#else
#error "ADC sampling time could not be defined for internal channels!"
#endif
#endif /* !ADC_SAMPLINGTIME_INTERNAL */

#ifndef ADC_CLOCK_DIV
#ifdef ADC_CLOCK_SYNC_PCLK_DIV4
#define ADC_CLOCK_DIV       ADC_CLOCK_SYNC_PCLK_DIV4
#elif ADC_CLOCK_SYNC_PCLK_DIV2
#define ADC_CLOCK_DIV       ADC_CLOCK_SYNC_PCLK_DIV2
#elif defined(ADC_CLOCK_ASYNC_DIV4)
#define ADC_CLOCK_DIV       ADC_CLOCK_ASYNC_DIV4
#endif
#endif /* !ADC_CLOCK_DIV */

#ifndef ADC_REGULAR_RANK_1
#define ADC_REGULAR_RANK_1  1
#endif

#define ADC_RESOLUTION              12

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static uint32_t get_adc_channel(Pin_ADC pin);
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
static uint16_t adc_read_value(Pin_ADC pin, uint32_t resolution);

/************************************************************************************
 *
 *	\fn		static uint32_t get_adc_channel(Pin_ADC pin)
 *	\brief
 *
 ***************************************************************************************/
static uint32_t get_adc_channel(Pin_ADC pin) {
  uint32_t channel = 0;
  switch (pin) {
#if 1
#ifdef ADC_CHANNEL_0
    case 0:
      channel = ADC_CHANNEL_0;
      break;
#endif
    case 1:
      channel = ADC_CHANNEL_1;
      break;
    case 2:
      channel = ADC_CHANNEL_2;
      break;
    case 3:
      channel = ADC_CHANNEL_3;
      break;
    case 4:
      channel = ADC_CHANNEL_4;
      break;
    case 5:
      channel = ADC_CHANNEL_5;
      break;
    case 6:
      channel = ADC_CHANNEL_6;
      break;
    case 7:
      channel = ADC_CHANNEL_7;
      break;
    case 8:
      channel = ADC_CHANNEL_8;
      break;
    case 9:
      channel = ADC_CHANNEL_9;
      break;
    case 10:
      channel = ADC_CHANNEL_10;
      break;
    case 11:
      channel = ADC_CHANNEL_11;
      break;
    case 12:
      channel = ADC_CHANNEL_12;
      break;
    case 13:
      channel = ADC_CHANNEL_13;
      break;
    case 14:
      channel = ADC_CHANNEL_14;
      break;
    case 15:
      channel = ADC_CHANNEL_15;
      break;
#ifdef ADC_CHANNEL_16
    case 16:
      channel = ADC_CHANNEL_16;
      break;
#endif
    case 17:
      channel = ADC_CHANNEL_17;
      break;
#ifdef ADC_CHANNEL_18
    case 18:
      channel = ADC_CHANNEL_18;
      break;
#endif
#ifdef ADC_CHANNEL_19
    case 19:
      channel = ADC_CHANNEL_19;
      break;
#endif
#ifdef ADC_CHANNEL_20
    case 20:
      channel = ADC_CHANNEL_20;
      break;
    case 21:
      channel = ADC_CHANNEL_21;
      break;
    case 22:
      channel = ADC_CHANNEL_22;
      break;
    case 23:
      channel = ADC_CHANNEL_23;
      break;
    case 24:
      channel = ADC_CHANNEL_24;
      break;
    case 25:
      channel = ADC_CHANNEL_25;
      break;
    case 26:
      channel = ADC_CHANNEL_26;
      break;
#ifdef ADC_CHANNEL_27
    case 27:
      channel = ADC_CHANNEL_27;
      break;
    case 28:
      channel = ADC_CHANNEL_28;
      break;
    case 29:
      channel = ADC_CHANNEL_29;
      break;
    case 30:
      channel = ADC_CHANNEL_30;
      break;
    case 31:
      channel = ADC_CHANNEL_31;
      break;
#endif
#endif
#endif
#if defined(ADC_CHANNEL_TEMPSENSOR)
    case INTERNAL_TEMP:
      channel = ADC_CHANNEL_TEMPSENSOR;
      break;
#endif
#if defined(ADC_CHANNEL_TEMPSENSOR_ADC1)
    case PADC_TEMP:
      channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
      break;
#endif
#if defined(ADC5) && defined(ADC_CHANNEL_TEMPSENSOR_ADC5)
    case PADC_TEMP_ADC5:
      channel = ADC_CHANNEL_TEMPSENSOR_ADC5;
      break;
#endif
#ifdef ADC_CHANNEL_VREFINT
    case INTERNAL_VREF:
      channel = ADC_CHANNEL_VREFINT;
      break;
#endif
#ifdef ADC_CHANNEL_VBAT
    case INTERNAL_VBAT:
      channel = ADC_CHANNEL_VBAT;
      break;
#endif
    default:
      channel = 0;
      break;
  }    

  return channel;
}

/************************************************************************************
 *
 *	\fn		static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
 *	\brief 
 *
 ***************************************************************************************/ 
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from != to) {
    if (from > to) {
      value = (value < (uint32_t)(1 << (from - to))) ? 0 : ((value + 1) >> (from - to)) - 1;
    } else {
      if (value != 0) {
        value = ((value + 1) << (to - from)) - 1;
      }
    }
  }
  return value;
}

/************************************************************************************
 *
 *	\fn	static uint16_t adc_read_value(Pin_ADC pin, uint32_t resolution)
 *	\brief This function will set the ADC to the required value
 *
 ***************************************************************************************/
static uint16_t adc_read_value(Pin_ADC pin, uint32_t resolution) {
  ADC_HandleTypeDef AdcHandle = {};
  ADC_ChannelConfTypeDef  AdcChannelConf = {};
  __IO uint16_t uhADCxConvertedValue = 0;
  uint32_t samplingTime = ADC_SAMPLINGTIME;
  uint32_t channel = 0;

  AdcHandle.Instance = ADC1;
  channel = get_adc_channel(pin);

  if(channel >= INTERNAL_TEMP)
    samplingTime = ADC_SAMPLINGTIME_INTERNAL;

#ifdef ADC_CLOCK_DIV
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_DIV;                 /* (A)synchronous clock mode, input ADC clock divided */
#endif


#ifdef ADC_RESOLUTION_12B
  switch (resolution) {
#ifdef ADC_RESOLUTION_6B
    case 6:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_6B;             /* resolution for converted data */
      break;
#endif
    case 8:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_8B;             /* resolution for converted data */
      break;
    case 10:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_10B;            /* resolution for converted data */
      break;
    case 12:
    default:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_12B;            /* resolution for converted data */
      break;
#ifdef ADC_RESOLUTION_14B
    case 14:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_14B;            /* resolution for converted data */
      break;
#endif
#ifdef ADC_RESOLUTION_16B
    case 16:
      AdcHandle.Init.Resolution          = ADC_RESOLUTION_16B;            /* resolution for converted data */
      break;
#endif
  }
#else
  UNUSED(resolution);
#endif
#ifdef ADC_DATAALIGN_RIGHT
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
#endif
#ifdef ADC_SCAN_SEQ_FIXED
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_SEQ_FIXED;            /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
#else
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
#endif
#ifdef ADC_EOC_SINGLE_CONV
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(ADC1_V2_5)

  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
#endif  
  //AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;                       /* ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered */
#ifdef ADC_CHANNELS_BANK_B
  AdcHandle.Init.ChannelsBank          = bank;
#elif defined(ADC_CHANNELS_BANK_A)
  AdcHandle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
#endif
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#if !defined(STM32F0xx) && !defined(STM32L0xx)
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
#endif
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx) && \
    !defined(STM32WLxx)
  AdcHandle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
#endif
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#if !defined(STM32F1xx) && !defined(ADC1_V2_5)
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
#endif
#if !defined(STM32F1xx) && !defined(STM32H7xx) && !defined(STM32MP1xx) && \
    !defined(ADC1_V2_5)
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
#endif
#ifdef ADC_CONVERSIONDATA_DR
  AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;      /* Regular Conversion data stored in DR register only */
#endif
#ifdef ADC_OVR_DATA_OVERWRITTEN
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
#endif
#ifdef ADC_LEFTBITSHIFT_NONE
  AdcHandle.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;         /* No bit shift left applied on the final ADC convesion data */
#endif

#if defined(STM32F0xx)
  AdcHandle.Init.SamplingTimeCommon    = samplingTime;
#endif
#if defined(STM32G0xx)
  AdcHandle.Init.SamplingTimeCommon1   = samplingTime;              /* Set sampling time common to a group of channels. */
  AdcHandle.Init.SamplingTimeCommon2   = samplingTime;              /* Set sampling time common to a group of channels, second common setting possible.*/
#endif
#if defined(STM32L0xx)
  AdcHandle.Init.LowPowerFrequencyMode = DISABLE;                       /* To be enabled only if ADC clock < 2.8 MHz */
  AdcHandle.Init.SamplingTime          = samplingTime;
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F3xx) && !defined(STM32F4xx) && !defined(STM32F7xx) && \
    !defined(STM32L1xx)
  AdcHandle.Init.OversamplingMode      = DISABLE;
  /* AdcHandle.Init.Oversample ignore for STM32L0xx as oversampling disabled */
  /* AdcHandle.Init.Oversampling ignored for other as oversampling disabled */
#endif
#if defined(ADC_CFGR_DFSDMCFG) && defined(DFSDM1_Channel0)
  AdcHandle.Init.DFSDMConfig           = ADC_DFSDM_MODE_DISABLE;        /* ADC conversions are not transferred by DFSDM. */
#endif
#ifdef ADC_TRIGGER_FREQ_HIGH
  AdcHandle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif
#ifdef ADC_VREF_PPROT_NONE
  AdcHandle.Init.VrefProtection = ADC_VREF_PPROT_NONE;
#endif

  AdcHandle.State = HAL_ADC_STATE_RESET;
  AdcHandle.DMA_Handle = NULL;
  AdcHandle.Lock = HAL_UNLOCKED;
  /* Some other ADC_HandleTypeDef fields exists but not required */

  //g_current_pin = pin; /* Needed for HAL_ADC_MspInit*/

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    return 0;
  }

  AdcChannelConf.Channel      = channel;                          /* Specifies the channel to configure into ADC */

#if defined(STM32G4xx) || defined(STM32L4xx) || defined(STM32L5xx) || \
    defined(STM32WBxx)
  if (!IS_ADC_CHANNEL(&AdcHandle, AdcChannelConf.Channel)) {
#else
  if (!IS_ADC_CHANNEL(AdcChannelConf.Channel)) {
#endif
    return 0;
  }
#if defined(ADC_SCAN_SEQ_FIXED) && defined(ADC_RANK_CHANNEL_NUMBER)
  AdcChannelConf.Rank         = ADC_RANK_CHANNEL_NUMBER;          /* Enable the rank of the selected channels when not fully configurable */
#else
  AdcChannelConf.Rank         = ADC_REGULAR_RANK_1;               /* Specifies the rank in the regular group sequencer */
#endif
#if !defined(STM32L0xx)
#if !defined(STM32G0xx)
  AdcChannelConf.SamplingTime = samplingTime;                     /* Sampling time value to be set for the selected channel */
#else
  AdcChannelConf.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;        /* Sampling time value to be set for the selected channel */
#endif
#endif
#if defined(ADC_DIFFERENTIAL_ENDED) && !defined(ADC1_V2_5)
  AdcChannelConf.SingleDiff   = ADC_SINGLE_ENDED;                 /* Single-ended input channel */
  AdcChannelConf.OffsetNumber = ADC_OFFSET_NONE;                  /* No offset subtraction */
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32G0xx) && !defined(STM32L0xx) && !defined(STM32L1xx) && \
    !defined(STM32WBxx) && !defined(STM32WLxx) && \
    !defined(ADC1_V2_5)
  AdcChannelConf.Offset = 0;                                      /* Parameter discarded because offset correction is disabled */
#endif
#if defined (STM32H7xx) || defined(STM32MP1xx)
  AdcChannelConf.OffsetRightShift = DISABLE;                      /* No Right Offset Shift */
  AdcChannelConf.OffsetSignedSaturation = DISABLE;                /* Signed saturation feature is not used */
#endif

  if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK) {
    return 0;
  }

#if defined(ADC_CR_ADCAL) || defined(ADC_CR2_RSTCAL)
#if defined(ADC_CALIB_OFFSET)
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
#elif defined(ADC_SINGLE_ENDED) && !defined(ADC1_V2_5)
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
#else
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) !=  HAL_OK)
#endif
  {
    /* ADC Calibration Error */
    return 0;
  }
#endif

  if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
    /* Start Conversion Error */
    return 0;
  }

  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
    /* End Of Conversion flag not set on time */
    return 0;
  }

  /* Check if the continous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }

  if (HAL_ADC_Stop(&AdcHandle) != HAL_OK) {
    /* Stop Conversation Error */
    return 0;
  }

  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
    return 0;
  }

  if (__LL_ADC_COMMON_INSTANCE(AdcHandle.Instance) != 0U) {
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(AdcHandle.Instance), LL_ADC_PATH_INTERNAL_NONE);
  }

  return uhADCxConvertedValue;
}

/************************************************************************************
 *
 *	\fn		uint32_t analogRead(Pin_ADC pin)
 *	\brief This function will set the ADC to the required value
 *
 ***************************************************************************************/
uint16_t AnalogRead(Pin_ADC pin) {
  uint32_t value = 0;
  
  value = adc_read_value(pin, ADC_RESOLUTION);

  //valuer du pont diviseur interne et externe a prendre en compte 

  //value = mapResolution(value, ADC_RESOLUTION, 10);
  
  //return (float)value * (1.0f / (float)0xFFF); // 12 bits range

  return value;
}
#endif 
