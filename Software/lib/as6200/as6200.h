#ifndef AS6200_h
#define AS6200_h

#include <Arduino.h>
#include <Wire.h>

#define CFG_SLEEP_MODE      0x01
#define CFG_CONTINIOUS_MODE 0x02
#define CFG_POLARITY_LOW    0x04
#define CFG_POLARITY_HIGH   0x08
#define CFG_SINGLE_SHOT_EN  0x10
#define CFG_SINGLE_SHOT_DIS 0x20

#define AS6200_MIN_TEMP (-40.0)
#define AS6200_MAX_TEMP (125.0)
#define AS6200_MAX_CONVERSION_TIME (40) /* ms */

enum class AS6200_ADDRESS {
  ADDRESS_VSS = 0x48,
  ADDRESS_VDD = 0x49,
};

enum class CONVERSION_RATE {
  RATE_4S = 0x00,
  RATE_1S = 0x01,
  RATE_250MS = 0x02,
  RATE_125MS = 0x03
};

enum class SLEEP_MODE {
  CONTINIOUS_CONVERSION = 0x00,
  SLEEP = 0x01
};

enum class INTERRUPT_MODE {
  COMPARATOR = 0x00,
  INTERRUPT = 0x01
};

enum class POLARITY {
  ACTIVE_LOW = 0x00,
  ACTIVE_HIGH = 0x01
};

enum class CONSECUTIVE_FAULTS {
  FAULTS_1 = 0x00,
  FAULTS_2 = 0x01,
  FAULTS_4 = 0x02,
  FAULTS_6 = 0x03
};

enum class SINGLE_SHOT_CONVERSION {
  FINISH = 0x00,
  START = 0x01,
};

enum class THRESHOLD {
  TLOW = 0,
  THIGH = 1
};

#if 1 /* endianess */
typedef __attribute__ ((packed)) union t_AS6200_CONFIG_ {
  union {
    struct {
      uint8_t sleepMode:1;
      uint8_t interruptMode:1;
      uint8_t polarity:1;
      uint8_t consecutiveFault:2;
      uint8_t reserved1:2;
      uint8_t singleShot:1;

      uint8_t reserved0:5;
      uint8_t alert:1;
      uint8_t conversionRate:2;
      
      
    } st;
    uint8_t buffer[2];
  } u;
} t_AS6200_CONFIG;
#else
typedef __attribute__ ((packed)) union t_AS6200_CONFIG_ {
  union {
    struct {
      uint8_t reserved0:5;
      uint8_t alert:1;
      uint8_t conversionRate:2;
      uint8_t sleepMode:1;
      uint8_t interruptMode:1;
      uint8_t polarity:1;
      uint8_t consecutiveFault:2;
      uint8_t reserved1:2;
      uint8_t singleShot:1;
    } st;
    uint8_t buffer[2];
  } u;
} t_AS6200_CONFIG;
#endif

class AS6200
{
	public:
		AS6200(AS6200_ADDRESS address = AS6200_ADDRESS::ADDRESS_VSS); 
		void begin(TwoWire &inWire = Wire);

    int32_t configure(void);
    int32_t setAlertState(bool state);
		int32_t getTemperature(double *p_temperature);
    int32_t setTemperatureThreshold(THRESHOLD threshold, double temperature);
    int32_t getTemperatureThreshold(THRESHOLD threshold, double *p_temperature);
		int32_t setSingleShotMode(void);
		int32_t setContinuousMode(void);
		int32_t setPolarity(POLARITY polarity);
		int32_t waitConversion(void);
    int32_t waitAlert(bool state);

	private:
		TwoWire *myWire;
		AS6200_ADDRESS _address;

    int32_t setConfigurationBit(uint8_t configuration);
		int32_t getConfigurationRegister(t_AS6200_CONFIG *pt_config);
		int32_t setConfigurationRegister(t_AS6200_CONFIG *pt_config);
		int32_t getTempRegister(int registerAddress, uint16_t *pu16_tValData);
		int32_t setTempRegister(int registerAddress, uint16_t u16_tValData);
		double temperatureRegisterToDouble(unsigned short registerValue);
		unsigned short doubleToTemperatureRegister(double value);
};

#endif
