#include "as6200.h"
#include <Wire.h>

#define AS6200_TVAL_REGISTER 0x00
#define AS6200_CONFIG_REGISTER 0x01
#define AS6200_TLOW_REGISTER 0x02
#define AS6200_THIGH_REGISTER 0x03

#define AS6200_MIN_CONVERSION_DELAY	24
#define AS6200_MIN_WAIT_ALERT_DELAY 30
#define AS6200_TRY_MAX 3

AS6200::AS6200(AS6200_ADDRESS address) : _address(address) {

}

void AS6200::begin(TwoWire &inWire) {
    this->myWire = &inWire;
    this->myWire->begin();
}

//public methods
int32_t AS6200::configure(void)
{
  double thresholdMin = 150, thresholdMax = 150;

  getTemperatureThreshold(THRESHOLD::TLOW, &thresholdMin);
  if(thresholdMin != AS6200_MIN_TEMP) {
    setTemperatureThreshold(THRESHOLD::TLOW, AS6200_MIN_TEMP);
    getTemperatureThreshold(THRESHOLD::TLOW, &thresholdMin);
    if(thresholdMin != AS6200_MIN_TEMP)
      return -1;
  }

  getTemperatureThreshold(THRESHOLD::THIGH, &thresholdMax);
  if(thresholdMax != AS6200_MAX_TEMP) {
    setTemperatureThreshold(THRESHOLD::THIGH, AS6200_MAX_TEMP);
    getTemperatureThreshold(THRESHOLD::THIGH, &thresholdMax);
    if(thresholdMax != AS6200_MAX_TEMP)
      return -1;
  }

  return 0;  
}

int32_t AS6200::setAlertState(bool state)
{
  return setConfigurationBit((state == true) ? CFG_CONTINIOUS_MODE | CFG_POLARITY_HIGH : CFG_SLEEP_MODE | CFG_POLARITY_LOW);      
}

int32_t AS6200::getTemperature(double *p_temperature) {
  uint16_t data = 0;

  if(getTempRegister(AS6200_TVAL_REGISTER, &data) != 0)
    return -1;

  *p_temperature = temperatureRegisterToDouble(data);

  return 0;  
}

int32_t AS6200::setTemperatureThreshold(THRESHOLD threshold, double temperature)
{
  uint8_t registerAddress;
  
  if((threshold < THRESHOLD::TLOW) || (threshold > THRESHOLD::THIGH))
    return -1;

  registerAddress = (threshold == THRESHOLD::TLOW) ? AS6200_TLOW_REGISTER : AS6200_THIGH_REGISTER; 

  return setTempRegister(registerAddress, doubleToTemperatureRegister(temperature));
}

int32_t AS6200::getTemperatureThreshold(THRESHOLD threshold, double *p_temperature)
{
  uint16_t data = 0;
  uint8_t registerAddress;

  if(((threshold < THRESHOLD::TLOW) || (threshold > THRESHOLD::THIGH)) || (p_temperature == NULL))
    return -1;

  registerAddress = (threshold == THRESHOLD::TLOW) ? AS6200_TLOW_REGISTER : AS6200_THIGH_REGISTER; 

  if(getTempRegister(registerAddress, &data) != 0)
    return -1;

  *p_temperature = temperatureRegisterToDouble(data);

  return 0;
}

int32_t AS6200::setSingleShotMode(void)
{
  return setConfigurationBit(CFG_SINGLE_SHOT_EN); 
}

int32_t AS6200::setContinuousMode(void)
{
  return setConfigurationBit(CFG_CONTINIOUS_MODE); 
}

int32_t AS6200::waitConversion(void)
{
  t_AS6200_CONFIG t_config;

  delay(AS6200_MIN_CONVERSION_DELAY-1);

  do {
    delay(1);

    if(getConfigurationRegister(&t_config) != 0)
      return -1;
  } while(t_config.u.st.singleShot == (uint8_t) SINGLE_SHOT_CONVERSION::START);

  return 0;  
}

int32_t AS6200::waitAlert(bool state)
{
  t_AS6200_CONFIG t_config; 
  uint8_t u8_alarmLevel = state == true ? 1 : 0;

  delay(AS6200_MIN_WAIT_ALERT_DELAY-10);

  do {
    delay(10);
 
    if(getConfigurationRegister(&t_config) != 0)
      return -1;
  } while (t_config.u.st.alert == u8_alarmLevel);

  return 0;
}

//private methods
int32_t AS6200::setConfigurationBit(uint8_t configuration)
{
  t_AS6200_CONFIG t_config; 
  
  if(getConfigurationRegister(&t_config) != 0)
    return -1;

  if(configuration & CFG_SLEEP_MODE)
    t_config.u.st.sleepMode = (uint8_t) SLEEP_MODE::SLEEP;
  else if(configuration & CFG_CONTINIOUS_MODE)
    t_config.u.st.sleepMode = (uint8_t) SLEEP_MODE::CONTINIOUS_CONVERSION;

  if(configuration & CFG_POLARITY_LOW)
    t_config.u.st.polarity = (uint8_t) POLARITY::ACTIVE_LOW;
  else if(configuration & CFG_POLARITY_HIGH)
    t_config.u.st.polarity = (uint8_t) POLARITY::ACTIVE_HIGH;
  
  if(configuration & CFG_SINGLE_SHOT_EN)
    t_config.u.st.singleShot = (uint8_t) SINGLE_SHOT_CONVERSION::START;
  else if(configuration & CFG_SINGLE_SHOT_DIS)
    t_config.u.st.singleShot = (uint8_t) SINGLE_SHOT_CONVERSION::FINISH;

  if(setConfigurationRegister(&t_config) != 0)
    return -1;

  return 0;
}

int32_t AS6200::getConfigurationRegister(t_AS6200_CONFIG *pt_config)
{
  int ret = -1;

  if(pt_config == NULL)
    return -1;

  this->myWire->beginTransmission((uint8_t) this->_address);
  this->myWire->write(AS6200_CONFIG_REGISTER);
  this->myWire->endTransmission(false);
  this->myWire->requestFrom((uint8_t) this->_address, 2);
  if (this->myWire->available())
    pt_config->u.buffer[0] = this->myWire->read();
  
  if (this->myWire->available()) {
    pt_config->u.buffer[1] = this->myWire->read();
    ret = 0;
  }
  
  return ret;  
}

int32_t AS6200::setConfigurationRegister(t_AS6200_CONFIG *pt_config) {
  if(pt_config == NULL)
    return -1;

  this->myWire->beginTransmission((uint8_t) this->_address);
	this->myWire->write((byte) AS6200_CONFIG_REGISTER);//CONFIG
	this->myWire->write((byte) pt_config->u.buffer[0]);// SS, SM
	this->myWire->write((byte) pt_config->u.buffer[1]);
	this->myWire->endTransmission();

  return 0;
}

int32_t AS6200::getTempRegister(int registerAddress, uint16_t *pu16_tValData) {
  int ret = -1;

  if(pu16_tValData == NULL)
    return -1;

  this->myWire->beginTransmission((uint8_t) this->_address);
  this->myWire->write(registerAddress);
  this->myWire->endTransmission(false);
  this->myWire->requestFrom((uint8_t) this->_address, 2);
  
  if (this->myWire->available())
    *pu16_tValData = this->myWire->read();
  
  *pu16_tValData <<= 8;

  if (this->myWire->available()) {
    *pu16_tValData |= this->myWire->read();
    ret = 0;
  }

  *pu16_tValData >>= 4; 

  return ret;
}

int32_t AS6200::setTempRegister(int registerAddress, uint16_t u16_tValData) {
  byte buffer[2];

  u16_tValData <<= 4;

  buffer[1] = u16_tValData & 0x00FF;
  buffer[0] = (u16_tValData & 0xFF00) >> 8;
  
  this->myWire->beginTransmission((uint8_t) this->_address);
	this->myWire->write((byte) registerAddress);
	this->myWire->write(buffer[0]);
	this->myWire->write(buffer[1]);
	this->myWire->endTransmission();

  return 0;
}

double AS6200::temperatureRegisterToDouble(unsigned short registerValue) {
	double value;

  if(registerValue > 0x7ff) { // 2's complement (negative number)
    value = (double)(registerValue | ~((1 << 12) -1)); 
  } else // positive number
    value = registerValue; 
  
  value *= 0.0625; /* apply resolution factor */

  return value;
}

unsigned short AS6200::doubleToTemperatureRegister(double value) {
  int16_t s16_register = value / 0.0625;

  if (s16_register < 0) {
    s16_register *= -1;
    s16_register = (~(s16_register)) + 1;
    s16_register = s16_register & 0x0FFF;
  }

  return (uint16_t) s16_register;
}

