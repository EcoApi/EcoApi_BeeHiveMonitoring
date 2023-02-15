/************************************************************************************//**
 *
 *	\file		trace.cpp
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
#include "trace.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/


/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/
HardwareSerial *p_serial_ = NULL;

static bool enable_ = true;

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/***************************************************************************************
 *
 *	\fn		void trace_init(HardwareSerial *p_serial)
 *	\brief 
 *
 ***************************************************************************************/
void trace_init(HardwareSerial *p_serial) {
  if(p_serial == NULL)
    return;

  p_serial_ = p_serial;

  pinMode(TRACE_EN, INPUT_PULLUP);
  bool traceEnValue = (bool) digitalRead(TRACE_EN);
  pinMode(TRACE_EN, INPUT_ANALOG);

  trace_setState(traceEnValue); 
}

/***************************************************************************************
 *
 *	\fn		void trace_setState(uint8_t u8_enable)
 *	\brief 
 *
 ***************************************************************************************/
void trace_setState(bool enable) {
  enable_ = enable; 

  if(enable) {
    p_serial_->setTx(UART1_TX);
    p_serial_->setRx(UART1_RX);

    p_serial_->begin(115200); // Sets the speed needed to read from the serial port when connected
    while (!*p_serial_) ; // Loop that only continues once the serial port is active (true)

    setDebugSerial(p_serial_);
    TRACE_CrLf("##### START #####");
  } else {
    p_serial_->end();
  }
}

/***************************************************************************************
 *
 *	\fn		uint8_t trace_getState(void)
 *	\brief 
 *
 ***************************************************************************************/
bool trace_getState(void) {
  return enable_;
}  

/***************************************************************************************
 *
 *	\fn		void trace_callback(uint8_t withCrLf, const char * format, ... )
 *	\brief 
 *
 ***************************************************************************************/
void trace_callback(uint8_t withCrLf, const char * format, ... ) {
  va_list argptr;
    
  if(enable_ == false)
    return;

  va_start(argptr, format);
  vprintf(format, argptr);
  va_end(argptr);
  
  if(withCrLf)
    printf("\r\n");
}

/***************************************************************************************
 *
 *	\fn		void trace_display_buf(const char* msg, const uint8_t* p_buffer, uint32_t bufferSize)
 *	\brief 
 *
 ***************************************************************************************/
#define DUMP_BUFFER_SIZE 100

void trace_display_buf(const char* msg, const uint8_t* p_buffer, uint32_t bufferSize) {
	int aux_size = (DUMP_BUFFER_SIZE+1)*3;
	char aux[aux_size];
	int i;
	int counter = 0;

  if(enable_ == false)
    return;

	trace_callback(true, "%s (%d bytes)", msg, bufferSize);

	while(bufferSize > 0)
	{
		for(i = 0; i < DUMP_BUFFER_SIZE && i < bufferSize; i++)
			snprintf(aux + (3*i), aux_size, "%02X ", (int)p_buffer[i]);
		aux[3*i] = 0;

		trace_callback(true, "  %9d: %s", (counter * DUMP_BUFFER_SIZE), aux);

		counter++;
		bufferSize -= i;
		p_buffer += i;
	}
}