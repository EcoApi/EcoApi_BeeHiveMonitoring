#ifndef __DEBUG_H
#define __DEBUG_H

#include <Arduino.h>
#include <HardwareSerial.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

extern void display_mallinfo(void);
extern void setDebugSerial(HardwareSerial *Debug);

//#ifdef __cplusplus
//}
//#endif

#endif