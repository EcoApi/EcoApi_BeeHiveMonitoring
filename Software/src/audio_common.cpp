/************************************************************************************//**
 *
 *	\file		audio_common.cpp
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
#include <string.h>
#include <stdio.h>
#include <float.h>

#include "audio_common.h"
#include "trace.h"

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#ifndef M_PI
  #define M_PI       3.14159265358979323846
#endif

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/
static float floatMap(float x, float in_min, float in_max, float out_min, float out_max);
static double parzen (int i, int nn);
static double welch (int i, int nn);
static double hanning (int i, int nn);
static double hamming (int i, int nn);
static double blackman (int i, int nn);
static double steeper (int i, int nn);

/***************************************************************************************
 *
 *	\fn		float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
 *	\brief 
 *
 ***************************************************************************************/
static float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_dataU16Hex(const char* sz_title, const uint16_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE_CrLf("%s :", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %04x ] ", p_data[i]);
		else
			TRACE_CrLf("[ %04x ]", p_data[i]);
	}	
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_dataU32Hex(const char* sz_title, const uint32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE_CrLf("%s :", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %08x ] ", p_data[i]);
		else
			TRACE_CrLf("[ %08x ]", p_data[i]);
	}	
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_dataShort(const char* sz_title, const uint16_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE_CrLf("[AUDIO] %s :", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %d ] ", p_data[i]);
		else
			TRACE_CrLf("[ %d ]", p_data[i]);
	}	
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_dataInteger(const char* sz_title, const uint32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE_CrLf("[AUDIO] %s :", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %d ] ", p_data[i]);
		else
			TRACE_CrLf("[ %d ]", p_data[i]);
	}	
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_dataFloat(const char* sz_title, const float32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("[AUDIO] %s :\r\n", sz_title);
  
  for(i=0;i<size;i++) {
		if(i < (size - 1))
			TRACE("[ %f ] ", p_data[i]);
		else
			TRACE_CrLf("[ %f ]", p_data[i]);
	}	
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief use https://hg.sr.ht/~hyozd/serialplot/ https://github.com/hyOzd/serialplot
 *
 ***************************************************************************************/
void print_plotterFloat(float const * p_data, uint16_t size) {
  uint16_t i;
  for (i = 0; i<size; i++)
    TRACE_CrLf("%f", p_data[i]);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief use https://hg.sr.ht/~hyozd/serialplot/ https://github.com/hyOzd/serialplot
 *
 ***************************************************************************************/
void print_plotterUShort(uint16_t const * p_data, uint16_t size) {
  uint16_t i;
  for (i = 0; i<size; i++)
    TRACE_CrLf("%d", p_data[i]);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_fft_min(float * m_fft_output_f32, uint16_t data_size, float sample_res_hz) {
  float32_t min_value = 0;
  uint32_t  min_val_index = 0;
  
  uint32_t offset = 0;//10;
  arm_min_f32(&m_fft_output_f32[offset], data_size - offset, &min_value, &min_val_index);
  min_val_index += offset;
  
  TRACE_CrLf("[AUDIO] Frequency sample: %f Hz\r\n", (float32_t) sample_res_hz);
  TRACE_CrLf("[AUDIO] Min magnitude value: %f, index %d, frequency %f Hz\r\n", min_value, min_val_index, (float32_t) min_val_index * sample_res_hz);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_fft_max(float * m_fft_output_f32, uint16_t data_size, float sample_res_hz) {
  float32_t max_value = 0;
  uint32_t  max_val_index = 0;
  
  uint32_t offset = 0;//10;
  arm_max_f32(&m_fft_output_f32[offset], data_size - offset, &max_value, &max_val_index);
  max_val_index += offset;
  
  TRACE_CrLf("[AUDIO] frequency sample: %f Hz", (float32_t) sample_res_hz);
  TRACE_CrLf("[AUDIO] max magnitude value: %f, index %d, frequency %f Hz", max_value, max_val_index, (float32_t) max_val_index * sample_res_hz);
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void print_fft_result(FFT_RESULTS *p_fftResult, float sample_res_hz, uint32_t fft_size) {
  float32_t startFrequency = p_fftResult->binOffset * sample_res_hz;
  float32_t binFrequency = (float32_t) p_fftResult->binSize * sample_res_hz;
  uint16_t binEnd = p_fftResult->binOffset + (p_fftResult->binCount * p_fftResult->binSize);
  uint16_t i;

  if(binEnd >= fft_size)
    binEnd = fft_size;

  TRACE_CrLf("[AUDIO] fft result start: %u / %0.2f Hz, end: %u / %0.2f Hz, out bin freq: %0.2f Hz, out bin count: %u, in bin by out bin: %u", p_fftResult->binOffset, startFrequency,  binEnd, (float32_t) binEnd * sample_res_hz, binFrequency, p_fftResult->binCount, p_fftResult->binSize);

  for(i=0;i<p_fftResult->binCount;i++) {
    TRACE_CrLf("[AUDIO] %02d -> brut s_bin%f_%fHz,\t\ts_bin%04d_%04dHz = %d", i,
                                               (float32_t) (startFrequency + ((float32_t) i * binFrequency)), 
                                               (float32_t) (startFrequency + ((float32_t) i * binFrequency) + binFrequency),  
                                               (uint16_t) roundf((startFrequency + ((float32_t) i * binFrequency))), 
                                               (uint16_t) roundf((startFrequency + ((float32_t) i * binFrequency) + binFrequency)), 
                                               p_fftResult->values[i]);


  }
}

//https://github.com/sidneycadot/WindowFunctions
//https://github.com/kichiki/WaoN/blob/master/fft.c
//https://github.com/kfrlib/kfr

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief "Numerical Recipes in C" 2nd Ed. / Sec.13.4 - Data Windowing
 *
 ***************************************************************************************/
static double parzen (int i, int nn) {
  return (1.0 - fabs (((double)i-0.5*(double)(nn-1)) / (0.5*(double)(nn+1))));
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static double welch (int i, int nn) {
  return (1.0-(((double)i-0.5*(double)(nn-1)) / (0.5*(double)(nn+1))) * (((double)i-0.5*(double)(nn-1)) / (0.5*(double)(nn+1))));
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static double hanning (int i, int nn) {
  return ( 0.5 * (1.0 - cos (2.0*M_PI*(double)i/(double)(nn-1))) );
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief Reference: "Digital Filters and Signal Processing" 2nd Ed. / Sec.7.3 - Windows in Spectrum Analysis
 *
 ***************************************************************************************/
static double hamming (int i, int nn) {
  return ( 0.54 - 0.46 * cos (2.0*M_PI*(double)i/(double)(nn-1)) );
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static double blackman (int i, int nn) {
  return ( 0.42 - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1)) + 0.08 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
static double steeper (int i, int nn) {
  return ( 0.375 - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1)) + 0.125 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void windowing (int n, const float32_t *data, e_WindowType e_windowType, float32_t scale, float32_t *out) {
  int i;
  for (i = 0; i < n; i ++) {
    switch (e_windowType) {
	  case e_WINDOW_TYPE_PARZEN:
	    out [i] = data [i] * (float32_t) parzen (i, n) / scale;
	    break;

	  case e_WINDOW_TYPE_WELCH: 
	    out [i] = data [i] * (float32_t) welch (i, n) / scale;
	    break;

	  case e_WINDOW_TYPE_HANNING: 
	    out [i] = data [i] * (float32_t) hanning (i, n) / scale;
	    break;

	  case e_WINDOW_TYPE_HAMMING:
	    out [i] = data [i] * (float32_t) hamming (i, n) / scale;
	    break;

	  case e_WINDOW_TYPE_BLACKMAN:
	    out [i] = data [i] * (float32_t) blackman (i, n) / scale;
	    break;

	  case e_WINDOW_TYPE_STEEPER:
	    out [i] = data [i] * (float32_t) steeper (i, n) / scale;
	    break;
	  
	  case e_WINDOW_TYPE_NONE:
    default: 
	    out [i] = data [i] / scale;
	    break;
	  }
  }
}

/***************************************************************************************
 *
 *	\fn		int32_t sht3x_suspend(void)
 *	\brief 
 *
 ***************************************************************************************/
void fft_create_result(FFT_RESULTS *p_fftResult, float *p_fftValue, uint8_t binOutputCount /* 20 max */, uint16_t binOffset /* max 512 */, uint8_t binOutputSize, uint16_t fftSize, e_FFT_SAMPLE_TYPE e_sampleType) {
  uint16_t binsOutput, binInputCurrent, i;
  float32_t binOutputSum;
  uint16_t min = UINT16_MAX;
  bool end = FALSE;

  if((p_fftResult == NULL) || (p_fftValue == NULL) || !binOutputCount || !fftSize)
    return;

  if(binOutputCount > AUDIO_MAX_BINS)
    binOutputCount = AUDIO_MAX_BINS;

  p_fftResult->binOffset = binOffset; // start input bin
  p_fftResult->binSize = binOutputSize; // n input bin for one output bin

  for(binsOutput=0;binsOutput<binOutputCount;binsOutput++) {
    binOutputSum = 0.0;

    for(i=0;i<binOutputSize;i++) {
      binInputCurrent = binOffset + ((binsOutput * binOutputSize) + i);

      if(binInputCurrent >= fftSize) {
        end = TRUE;
        break;
      }  

      binOutputSum += p_fftValue[binInputCurrent];
    }

    if(e_sampleType == e_FFT_SAMBLE_TYPE_I2S)
      p_fftResult->values[binsOutput] = (uint16_t) floatMap(binOutputSum, 0, UINT32_MAX, 0, UINT16_MAX);
    else 
      p_fftResult->values[binsOutput] = constrain((uint16_t) binOutputSum, 0, UINT16_MAX);

    if((p_fftResult->values[binsOutput] != 0) && (p_fftResult->values[binsOutput] < min))
      min = p_fftResult->values[binsOutput];
  
    if(end)
      break;
  }

  p_fftResult->binCount = binsOutput; //n output bin computed

  /* noise filter */
  for(binsOutput=0;binsOutput<p_fftResult->binCount;binsOutput++) {
    if(p_fftResult->values[binsOutput] < min)
      p_fftResult->values[binsOutput] = 0;
    else
      p_fftResult->values[binsOutput] -= min;
  }  
}
