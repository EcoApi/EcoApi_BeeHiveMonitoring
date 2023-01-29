#ifndef PCF8563_H
#define PCF8563_H

#include <time.h>

#define W8563_ADDR     0xA2    // I2C address for write
#define R8563_ADDR     0xA3    // I2C address for read
#define PCF8563_FREQ     400000  // bus speed 400 kHz

class PCF8563
{
public:
    PCF8563(void);

    bool begin(void);
    bool begin(time_t *p_init_time);

    bool lostPower(void);

    bool get_time(struct tm *time);
    time_t get_timestamp(void);
    time_t now(void);
    
    bool set_time(struct tm time);
    bool set_timestamp(time_t time);
    
    bool set_timeAlarm(struct tm time);
    bool set_timestampAlarm(time_t time);
    bool get_timeAlarm(struct tm *time);
    time_t get_timestampAlarm(void);
    
    void clearAlarm(void);
    void resetAlarm(void);

    bool alarmEnabled(void);
    bool alarmActive(void);

    byte getStatus1(void);
    byte getStatus2(void);
    byte getClkOut(void);
    void setClkOut(byte value);
    void setRegister(byte register_, byte value);
    void initClock(void);
    void zeroClock(void);
     void getDateTime(); 
     void clearVoltLow(void); /* Only clearing is possible */
    /**
     */
    
private:
    int Rtcc_Addr;

    bool detected(void);

    /*static int bcdToDecimal(int bcd) {
        return ((bcd & 0xF0) >> 4) * 10 + (bcd & 0x0F);
    }*/

    /*static int decimalToBcd(int dec) {
        return (dec % 10) + ((dec / 10) << 4);
    }*/

    /* Private internal functions, but useful to look at if you need a similar func. */
    static int decimalToBcd(int dec) {
        return ( (dec/10*16) + (dec%10) );
    }

    static int bcdToDecimal(int bcd) {
        return ( (bcd/16*10) + (bcd%16) );
    }

    byte decToBcd(byte value);
    byte bcdToDec(byte value);

    void enableAlarm(void);

    byte hour;
    byte minute;
    bool volt_low;
    byte sec;
    byte day;
    byte weekday;
    byte month;
    byte year;
    /* alarm */
    byte alarm_hour;
    byte alarm_minute;
    byte alarm_weekday;
    byte alarm_day;
    /* CLKOUT */
    byte squareWave;
    /* timer */
    byte timer_control;
    byte timer_value;
    /* support */
    byte status1;
    byte status2;
    bool century;
    bool voltage_low;
    time_t last_ts;

};

#endif
