/************************************************************************************//**
 *
 *	\file		audio_common.h
 *
 *	\brief
 *
 *	\date		2 nov. 2021
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

#ifndef __AUDIO_COMMON_H_
#define __AUDIO_COMMON_H_

/***************************************************************************************/
/*	Includes																		
/***************************************************************************************/
#include <string.h>
#include <stdio.h>

#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include <math.h>

#include "trace.h"
#include "board.h"
#include "ramret.h"
#include "eeprom.h"  

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************/
/* Define
/***************************************************************************************/
#define DEBUG_SAMPLING_FREQUENCY      0
#define FFT_LOOP_ENABLED              0

//todo use pt_ramRet->audioSettings.samplingFrequency
#define FFT_SAMPLE_FREQ_HZ            (4000) /* only : 2000 / 4000 / 8000 / 16000 Hz */
#define FFT_OUTPUT_SIZE               1024UL
#define FTT_COUNT                     1//1 

/***************************************************************************************/
/* Typedef                                                                        
/***************************************************************************************/
typedef enum e_FFT_SAMPLE_TYPE_ {
  e_FFT_SAMBLE_TYPE_ADC = 0,
  e_FFT_SAMBLE_TYPE_I2S,
} e_FFT_SAMPLE_TYPE;

PACK(struct FFT_RESULTS_ {
  uint8_t sampleType; /* sampleType */
  uint16_t binOffset; /* start bin */
  uint8_t binSize; /* bin count in one outputbin */
  uint8_t binCount; /* bin output count */
  uint16_t values[AUDIO_MAX_BINS];
});
typedef FFT_RESULTS_ FFT_RESULTS;

typedef enum e_WindowType_ {
  e_WINDOW_TYPE_NONE = 0, // square (no window)
  e_WINDOW_TYPE_PARZEN,
  e_WINDOW_TYPE_WELCH,
  e_WINDOW_TYPE_HANNING,
  e_WINDOW_TYPE_HAMMING,
  e_WINDOW_TYPE_BLACKMAN,
  e_WINDOW_TYPE_STEEPER, //30-dB/octave rolloff window
} e_WindowType;

/***************************************************************************************/
/*	Shared Functions																  
/***************************************************************************************/  
void print_dataU16Hex(const char* sz_title, const uint16_t *p_data, uint16_t size);
void print_dataU32Hex(const char* sz_title, const uint32_t *p_data, uint16_t size); 
void print_dataShort(const char* sz_title, const uint16_t *p_data, uint16_t size);
void print_dataInteger(const char* sz_title, const uint32_t *p_data, uint16_t size);
void print_dataFloat(const char* sz_title, const float32_t *p_data, uint16_t size);
void print_plotterFloat(float const * p_data, uint16_t size);
void print_plotterUShort(uint16_t const * p_data, uint16_t size);
void print_fft_min(float * m_fft_output_f32, uint16_t data_size, float sample_res_hz);
void print_fft_max(float * m_fft_output_f32, uint16_t data_size, float sample_res_hz);
void print_fft_result(FFT_RESULTS *p_fftResult, float sample_res_hz, uint32_t fft_size);
void windowing (int n, const float32_t *data, e_WindowType e_windowType, float32_t scale, float32_t *out);
void fft_create_result(FFT_RESULTS *p_fftResult, float *p_fftValue, uint8_t binOutputCount, uint16_t binOffset, uint8_t binOutputSize, uint16_t fftSize, e_FFT_SAMPLE_TYPE e_sampleType);

#ifdef __cplusplus
}
#endif
#endif