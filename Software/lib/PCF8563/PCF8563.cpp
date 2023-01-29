#include <Arduino.h>
#include <Wire.h>
#include "PCF8563.h"
#include <mktime.h>

/* the read and write values for pcf8563 rtcc */
/* these are adjusted for arduino */
#define RTCC_R      0xa3
#define RTCC_W      0xa2

#define RTCC_SEC        1
#define RTCC_MIN        2
#define RTCC_HR         3
#define RTCC_DAY        4
#define RTCC_WEEKDAY    5
#define RTCC_MONTH      6
#define RTCC_YEAR       7
#define RTCC_CENTURY    8

/* register addresses in the rtc */
#define RTCC_STAT1_ADDR     0x0
#define RTCC_STAT2_ADDR     0x01
#define RTCC_SEC_ADDR       0x02
#define RTCC_MIN_ADDR       0x03
#define RTCC_HR_ADDR        0x04
#define RTCC_DAY_ADDR       0x05
#define RTCC_WEEKDAY_ADDR   0x06
#define RTCC_MONTH_ADDR     0x07
#define RTCC_YEAR_ADDR      0x08
#define RTCC_ALRM_MIN_ADDR  0x09
#define RTCC_SQW_ADDR       0x0D
#define RTCC_TIMER1_ADDR    0x0E
#define RTCC_TIMER2_ADDR    0x0F

/* setting the alarm flag to 0 enables the alarm.
 * set it to 1 to disable the alarm for that value.
 */
#define RTCC_ALARM          0x80
#define RTCC_ALARM_AIE      0x02
#define RTCC_ALARM_AF       0x08
/* optional val for no alarm setting */
#define RTCC_NO_ALARM       99

#define RTCC_TIMER_TIE      0x01  // Timer Interrupt Enable

#define RTCC_TIMER_TF       0x04  // Timer Flag, read/write active state
                                  // When clearing, be sure to set RTCC_TIMER_AF
                                  // to 1 (see note above).
#define RTCC_TIMER_TI_TP    0x10  // 0: INT is active when TF is active
                                  //    (subject to the status of TIE)
                                  // 1: INT pulses active
                                  //    (subject to the status of TIE);
                                  // Note: TF stays active until cleared
                                  // no matter what RTCC_TIMER_TI_TP is.
#define RTCC_TIMER_TD10     0x03  // Timer source clock, TMR_1MIN saves power
#define RTCC_TIMER_TE       0x80  // Timer 1:enable/0:disable

/* Timer source-clock frequency constants */
#define TMR_4096HZ      B00000000
#define TMR_64Hz        B00000001
#define TMR_1Hz         B00000010
#define TMR_1MIN        B00000011

#define RTCC_CENTURY_MASK   0x80
#define RTCC_VLSEC_MASK     0x80

/* date format flags */
#define RTCC_DATE_WORLD     0x01
#define RTCC_DATE_ASIA      0x02
#define RTCC_DATE_US        0x04
/* time format flags */
#define RTCC_TIME_HMS       0x01
#define RTCC_TIME_HM        0x02

/* square wave contants */
#define SQW_DISABLE     B00000000
#define SQW_32KHZ       B10000000
#define SQW_1024HZ      B10000001
#define SQW_32HZ        B10000010
#define SQW_1HZ         B10000011

//extern Serial dbg;

PCF8563::PCF8563(void)
{
    //Wire.begin();
    //Rtcc_Addr = R8563_ADDR >> 1;
}

/**************************************************************************/
/*!
    @brief  Start I2C for the PCF8563 and test succesful connection
    @param  wireInstance pointer to the I2C bus
    @return True if Wire can find PCF8563 or false otherwise.
*/
/**************************************************************************/
bool PCF8563::begin(void) {
    time_t init_time;

    return begin(&init_time);
}

bool PCF8563::begin(time_t *p_init_time) {
  //Wire.begin();
  //Wire.setClock(400000);

  //delay(1);
  Rtcc_Addr = R8563_ADDR >> 1;

  if(!detected())
    return false;

  last_ts = now(); 

  *p_init_time = last_ts; 

  return true;
}

/*!
 *    @brief  Scans I2C for the address - note will give a false-positive
 *    if there's no pullups on I2C
 *    @return True if I2C initialized and a device with the addr found
 */
bool PCF8563::detected(void) {
  // A basic scanner, see if it ACK's
  Wire.beginTransmission(Rtcc_Addr);
  if (Wire.endTransmission() == 0) {
    return true;
  }

  return false;
}

/**************************************************************************/
/*!
    @brief  Check the status of the VL bit in the VL_SECONDS register.
    @details The PCF8563 has an on-chip voltage-low detector. When VDD drops
     below Vlow, bit VL in the VL_seconds register is set to indicate that
     the integrity of the clock information is no longer guaranteed.
    @return True if the bit is set (VDD droped below Vlow) indicating that
    the clock integrity is not guaranteed and false only after the bit is
    cleared using adjust()
*/
/**************************************************************************/
bool PCF8563::lostPower(void) {
#if 0
    char start = 0x02;
    char buffer;
    
    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.requestFrom(Rtcc_Addr, 1 /*7*/);
    buffer = Wire.read();

    return buffer >> 7;
#else
    return voltage_low;
#endif
}


bool PCF8563::get_time(struct tm *time)
{
    char start = 0x02;
    char buffer[7];
    
    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.available();
    
    Wire.requestFrom(Rtcc_Addr, 7);
    for (uint8_t i=0; i < 7; i++)
        buffer[i] = Wire.read();
    //Wire.endTransmission();

    voltage_low = buffer[0] >> 7;
	
    time->tm_sec = bcdToDecimal(buffer[0] /*& 0x7F*/ &~0x80); 
    time->tm_min = bcdToDecimal(buffer[1] & 0x7F);
    time->tm_hour = bcdToDecimal(buffer[2] & 0x3F);
    time->tm_mday = bcdToDecimal(buffer[3] & 0x3F);
    time->tm_wday = bcdToDecimal(buffer[4] & 0x07) + 1;
    time->tm_mon = bcdToDecimal(buffer[5] & 0x1F) - 1;
    time->tm_year = bcdToDecimal(buffer[6]);
    time->tm_yday = 0;
    time->tm_isdst = 0;

    return true;
}

time_t PCF8563::get_timestamp(void)
{
    struct tm now;
    time_t seconds;

    get_time(&now);

    if(false == _rtc_maketime(&now, &seconds, RTC_FULL_LEAP_YEAR_SUPPORT)) {
        return -1;
    }

    return seconds; 
}

time_t PCF8563::now(void)
{
    return get_timestamp(); 
}

bool PCF8563::set_time(struct tm time)
{
    char buffer[8];
    
    buffer[0] = 0x02; // memory address
    buffer[1] = decimalToBcd(time.tm_sec) & 0x7F; // VL = 0
    buffer[2] = decimalToBcd(time.tm_min) /*& 0x7F*/;
    buffer[3] = decimalToBcd(time.tm_hour) /*& 0x3F*/;
    buffer[4] = decimalToBcd(time.tm_mday) /*& 0x3F*/;
    buffer[5] = time.tm_wday - 1;
    buffer[6] = decimalToBcd(time.tm_mon + 1) /*& 0x1F*/;
    buffer[7] = decimalToBcd(time.tm_year);

    /* As per data sheet, have to set everything all in one operation */
    Wire.beginTransmission(Rtcc_Addr);    // Issue I2C start signal
    Wire.write(buffer, 8);
    Wire.endTransmission();

    return true;
}

bool PCF8563::set_timestamp(time_t time)
{
    struct tm now;
    
    _rtc_localtime(time, &now, RTC_FULL_LEAP_YEAR_SUPPORT);
    
    set_time(now);

    return true;
}

bool PCF8563::set_timeAlarm(struct tm time)
{
    char buffer[5];
    
    buffer[0] = 0x09; // memory address
    buffer[1] = decimalToBcd(time.tm_min);
    buffer[1] &= ~RTCC_ALARM;
    buffer[2] = decimalToBcd(time.tm_hour);
    buffer[2] &= ~RTCC_ALARM;
    buffer[3] = decimalToBcd(time.tm_mday);
    buffer[3] &= ~RTCC_ALARM;
    buffer[4] = (time.tm_wday - 1);
    buffer[4] &= ~RTCC_ALARM;

    // First set alarm values, then enable
    Wire.beginTransmission(Rtcc_Addr);    // Issue I2C start signal
    Wire.write(buffer, 5); 
    Wire.endTransmission();

    enableAlarm();
  
    return true;
}

bool PCF8563::set_timestampAlarm(time_t time)
{
    struct tm now;
    char buffer[5];
    
    _rtc_localtime(time, &now, RTC_FULL_LEAP_YEAR_SUPPORT);

    set_timeAlarm(now);
  
    return true;
}

bool PCF8563::get_timeAlarm(struct tm *time)
{
    char start = 0x09;
    char buffer[4];
    
    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.requestFrom(Rtcc_Addr, 4);
    for (uint8_t i=0; i < 4; i++)
        buffer[i] = Wire.read();

    time->tm_min = bcdToDecimal(buffer[0] & 0x7F);
    time->tm_hour = bcdToDecimal(buffer[1] & 0x3F);
    time->tm_mday = bcdToDecimal(buffer[2] & 0x3F);
    time->tm_wday = bcdToDecimal(buffer[3] & 0x07) + 1;
    
    time->tm_sec = 0;
    time->tm_mon = 0;
    time->tm_year = 0;
    time->tm_yday = 0;
    time->tm_isdst = 0;

    return true;
}

time_t PCF8563::get_timestampAlarm(void)
{
    struct tm now;
    time_t seconds;
    
    get_timeAlarm(&now);

    _rtc_maketime(&now, &seconds, RTC_FULL_LEAP_YEAR_SUPPORT);

    return seconds; 
}


byte PCF8563::getStatus1(void) {
    char start = RTCC_STAT1_ADDR;
    byte status;

    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.requestFrom(Rtcc_Addr, 1);
    status = Wire.read();

    return status;
}

byte PCF8563::getStatus2(void) {
    char start = RTCC_STAT2_ADDR;
    byte status;

    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.requestFrom(Rtcc_Addr, 1);
    status = Wire.read();

    return status;
}

byte PCF8563::getClkOut(void) {
    char start = RTCC_SQW_ADDR;
    byte clkout;

    Wire.flush();

    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte) start);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    Wire.requestFrom(Rtcc_Addr, 1);
    clkout = Wire.read();

    return clkout;

}

void PCF8563::setClkOut(byte value) {
    //enable the interrupt
    Wire.beginTransmission(Rtcc_Addr);  // Issue I2C start signal
    Wire.write((byte)RTCC_SQW_ADDR);
    Wire.write((byte)value);
    Wire.endTransmission();
}

void PCF8563::setRegister(byte register_, byte value) {
    //enable the interrupt
    Wire.beginTransmission(Rtcc_Addr);  // Issue I2C start signal
    Wire.write((byte)register_);
    Wire.write((byte)value);
    Wire.endTransmission(true);
}

/* enable alarm interrupt
 * whenever the clock matches these values an int will
 * be sent out pin 3 of the Pcf8563 chip
 */
void PCF8563::enableAlarm()
{
    char start = 0x01;
    byte status = getStatus2();
    
    //set status2 AF val to zero
    status &= ~RTCC_ALARM_AF;
    //set TF to 1 masks it from changing, as per data-sheet
    status |= RTCC_TIMER_TF;
    //enable the interrupt
    status |= RTCC_ALARM_AIE;

    //enable the interrupt
    Wire.beginTransmission(Rtcc_Addr);  // Issue I2C start signal
    Wire.write((byte)start);
    Wire.write((byte)status);
    Wire.endTransmission();
}

void PCF8563::clearAlarm()
{
    char start = 0x01;
    byte status = getStatus2();
    
    //set status2 AF val to zero to reset alarm
    status &= ~RTCC_ALARM_AF;
    //set TF to 1 masks it from changing, as per data-sheet
    status |= RTCC_TIMER_TF;
    //turn off the interrupt
    status &= ~RTCC_ALARM_AIE;

    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte)start);
    Wire.write((byte)status);
    Wire.endTransmission();
}

/**
* Reset the alarm leaving interrupt unchanged
*/
void PCF8563::resetAlarm()
{
    char start = 0x01;
    byte status = getStatus2();
    
    //set status2 AF val to zero to reset alarm
    status &= ~RTCC_ALARM_AF;
    //set TF to 1 masks it from changing, as per data-sheet
    status |= RTCC_TIMER_TF;

    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte)start);
    Wire.write((byte)status);
    Wire.endTransmission();
}

/*
* Returns true if AIE is on
*
*/
bool PCF8563::alarmEnabled()
{
    return getStatus2() & RTCC_ALARM_AIE;
}

/*
* Returns true if AF is on
*
*/
bool PCF8563::alarmActive()
{
    return getStatus2() & RTCC_ALARM_AF;
}

void PCF8563::initClock(void)
{
    Wire.beginTransmission(Rtcc_Addr);    // Issue I2C start signal
    Wire.write((byte)0x0);        // start address

    Wire.write((byte)0x0);     //control/status1
    Wire.write((byte)0x0);     //control/status2
    Wire.write((byte)0x00 /*0x81 &~ RTCC_VLSEC_MASK*/);     //set seconds & VL
    Wire.write((byte)0x00);    //set minutes
    Wire.write((byte)0x00);    //set hour
    Wire.write((byte)0x01);    //set day
    Wire.write((byte)0x00);    //set weekday
    Wire.write((byte)0x81);     //set month, century to 1
    Wire.write((byte)0x00);    //set year to 99
    Wire.write((byte)0x80);    //minute alarm value reset to 00
    Wire.write((byte)0x80);    //hour alarm value reset to 00
    Wire.write((byte)0x80);    //day alarm value reset to 00
    Wire.write((byte)0x80);    //weekday alarm value reset to 00
    Wire.write((byte)SQW_32HZ /*0x0*/);     //set SQW, see: setSquareWave
    Wire.write((byte)0x0);     //timer off
    Wire.endTransmission();
}

void PCF8563::zeroClock()
{
    Wire.beginTransmission(Rtcc_Addr);    // Issue I2C start signal
    Wire.write((byte)0x0);        // start address

    Wire.write((byte)0x0);     //control/status1
    Wire.write((byte)0x0);     //control/status2
    Wire.write((byte)0x00);    //set seconds to 0 & VL to 0
    Wire.write((byte)0x00);    //set minutes to 0
    Wire.write((byte)0x00);    //set hour to 0
    Wire.write((byte)0x01);    //set day to 1
    Wire.write((byte)0x00);    //set weekday to 0
    Wire.write((byte)0x81);    //set month to 1, century to 1900
    Wire.write((byte)0x00);    //set year to 0
    Wire.write((byte)0x80);    //minute alarm value reset to 00
    Wire.write((byte)0x80);    //hour alarm value reset to 00
    Wire.write((byte)0x80);    //day alarm value reset to 00
    Wire.write((byte)0x80);    //weekday alarm value reset to 00
    Wire.write((byte)B10000011); //set SQW to default, see: setSquareWave
    Wire.write((byte)0x0);     //timer off
    Wire.endTransmission();
}

/* Private internal functions, but useful to look at if you need a similar func. */
byte PCF8563::decToBcd(byte val)
{
    return ( (val/10*16) + (val%10) );
}

byte PCF8563::bcdToDec(byte val)
{
    return ( (val/16*10) + (val%16) );
}

void PCF8563::getDateTime(void)
{
    /* Start at beginning, read entire memory in one go */
    Wire.beginTransmission(Rtcc_Addr);
    Wire.write((byte)RTCC_STAT1_ADDR);
    Wire.endTransmission();

    /* As per data sheet, have to read everything all in one operation */
    uint8_t readBuffer[16] = {0};
    Wire.requestFrom(Rtcc_Addr, 16);
    for (uint8_t i=0; i < 16; i++)
        readBuffer[i] = Wire.read();

    // status bytes
    status1 = readBuffer[0];
    status2 = readBuffer[1];

    // time bytes
    //0x7f = 0b01111111
    volt_low = readBuffer[2] & RTCC_VLSEC_MASK;  //VL_Seconds
    sec = bcdToDec(readBuffer[2] & ~RTCC_VLSEC_MASK);
    minute = bcdToDec(readBuffer[3] & 0x7f);
    //0x3f = 0b00111111
    hour = bcdToDec(readBuffer[4] & 0x3f);

    // date bytes
    //0x3f = 0b00111111
    day = bcdToDec(readBuffer[5] & 0x3f);
    //0x07 = 0b00000111
    weekday = bcdToDec(readBuffer[6] & 0x07);
    //get raw month data byte and set month and century with it.
    month = readBuffer[7];
    if (month & RTCC_CENTURY_MASK)
        century = true;
    else
        century = false;
    //0x1f = 0b00011111
    month = month & 0x1f;
    month = bcdToDec(month);
    year = bcdToDec(readBuffer[8]);

    // alarm bytes
    alarm_minute = readBuffer[9];
    if(B10000000 & alarm_minute)
        alarm_minute = RTCC_NO_ALARM;
    else
        alarm_minute = bcdToDec(alarm_minute & B01111111);
    alarm_hour = readBuffer[10];
    if(B10000000 & alarm_hour)
        alarm_hour = RTCC_NO_ALARM;
    else
        alarm_hour = bcdToDec(alarm_hour & B00111111);
    alarm_day = readBuffer[11];
    if(B10000000 & alarm_day)
        alarm_day = RTCC_NO_ALARM;
    else
        alarm_day = bcdToDec(alarm_day  & B00111111);
    alarm_weekday = readBuffer[12];
    if(B10000000 & alarm_weekday)
        alarm_weekday = RTCC_NO_ALARM;
    else
        alarm_weekday = bcdToDec(alarm_weekday  & B00000111);

    // CLKOUT_control 0x03 = 0b00000011
    squareWave = readBuffer[13] & 0x03;

    // timer bytes
    timer_control = readBuffer[14] & 0x03;
    timer_value = readBuffer[15];  // current value != set value when running

    Serial.printf("RTC %d\r\n", sec);
}
