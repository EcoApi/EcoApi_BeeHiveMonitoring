/************************************************************************************//**
 *
 *	\file		utils.cpp
 *
 *	\brief
 *
 *	\date		22 jan. 2022
 *
 *	\author		ecoapi
 *
 ***************************************************************************************/

/***************************************************************************************/
/*	Includes																		                
/***************************************************************************************/
#include "utils.h"
#include <Wire.h>

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/
#define I2C_BITBANG_DELAY_US 5

/***************************************************************************************/
/*	Defines		  	 	 															                                     
/***************************************************************************************/

/***************************************************************************************/
/*	Local variables                                                                    
/***************************************************************************************/

/***************************************************************************************/
/*	Local Functions prototypes                                                         
/***************************************************************************************/

/*
 * Unwedge the i2c bus for the given port.
 *
 * Some devices on our i2c busses keep power even if we get a reset.  That
 * means that they could be partway through a transaction and could be
 * driving the bus in a way that makes it hard for us to talk on the bus.
 * ...or they might listen to the next transaction and interpret it in a
 * weird way.
 *
 * Note that devices could be in one of several states:
 * - If a device got interrupted in a write transaction it will be watching
 *   for additional data to finish its write.  It will probably be looking to
 *   ack the data (drive the data line low) after it gets everything.  Ideally
 *   we'd like to abort right away so we don't write bogus data.
 * - If a device got interrupted while responding to a register read, it will
 *   be watching for clocks and will drive data out when it sees clocks.  At
 *   the moment it might be trying to send out a 1 (so both clock and data
 *   may be high) or it might be trying to send out a 0 (so it's driving data
 *   low). Ideally we want to finish reading the current byte and then nak to
 *   abort everything.
 *
 * We attempt to unwedge the bus by doing:
 * - If possible, send a pseudo-"stop" bit.  We can only do this if nobody
 *   else is driving the clock or data lines, since that's the only way we
 *   have enough control.  The idea here is to abort any writes that might
 *   be in progress.  Note that a real "stop" bit would actually be a "low to
 *   high transition of SDA while SCL is high".  ...but both must be high for
 *   us to be in control of the bus.  Thus we _first_ drive SDA low so we can
 *   transition it high.  This first transition looks like a start bit.  In any
 *   case, the hope here is that it will look enough like an error condition
 *   that slaves will abort.
 * - If we failed to send the pseudo-stop bit, try one clock and try again.
 *   I've seen a reset happen while the device was waiting for us to clock out
 *   its ack of the address.  That should be the only time that the other side
 *   is driving things in the case of a write, so only 1 clock is enough.
 * - Try to clock 9 times, if we can.  This should finish reading out any data
 *   and then should nak.
 * - Send one last pseudo-stop bit, just for good measure.
 *
 * @param  port  The i2c port to unwedge.
 */

/***************************************************************************************
 *
 *	\fn		void unwedge_i2c_bus(uint32_t scl_pin, uint32_t sda_pin)
 *	\brief 
 *
 ***************************************************************************************/
void unwedge_i2c_bus(uint32_t scl_pin, uint32_t sda_pin) {
	int i;

	/*
	 * Reconfigure ports as general purpose open-drain outputs, initted
	 * to high.
	 *
	 * We manually set the level first in addition to using GPIO_HIGH
	 * since gpio_set_flags() behaves strangely in the case of a warm boot.
	 */

    pinMode(scl_pin, INPUT);
    pinMode(sda_pin, INPUT);

    /* Try to send out pseudo-stop bit.  See function description */
	if ((digitalRead(scl_pin) == HIGH) && (digitalRead(sda_pin) == HIGH)) {
        digitalWrite(scl_pin, HIGH);
        digitalWrite(sda_pin, HIGH);

        pinMode(scl_pin, OUTPUT);
        pinMode(sda_pin, OUTPUT);

	    digitalWrite(sda_pin, LOW);
	    delayMicroseconds(I2C_BITBANG_DELAY_US);
	    digitalWrite(sda_pin, HIGH);
	    delayMicroseconds(I2C_BITBANG_DELAY_US);
	} else {
        digitalWrite(scl_pin, HIGH);

        pinMode(scl_pin, OUTPUT);

		/* One more clock in case it was trying to ack its address */
		digitalWrite(scl_pin, LOW);
		delayMicroseconds(I2C_BITBANG_DELAY_US);
		digitalWrite(scl_pin, HIGH);
        delayMicroseconds(I2C_BITBANG_DELAY_US);

        pinMode(scl_pin, INPUT);

		if ((digitalRead(scl_pin) == HIGH) && (digitalRead(sda_pin) == HIGH)) {
            digitalWrite(sda_pin, HIGH);

            pinMode(sda_pin, OUTPUT);

		    digitalWrite(sda_pin, LOW);
		    delayMicroseconds(I2C_BITBANG_DELAY_US);
		    digitalWrite(sda_pin, HIGH);
		    delayMicroseconds(I2C_BITBANG_DELAY_US);
		}
	}

	/*
	 * Now clock 9 to read pending data; one of these will be a NAK.
	 *
	 * Don't bother even checking if scl is high--we can't do anything about
	 * it anyway.
	 */
    digitalWrite(scl_pin, HIGH);
    pinMode(scl_pin, OUTPUT);

	for (i = 0; i < 20 /*9*/; i++) {
		digitalWrite(scl_pin, LOW);
		delayMicroseconds(I2C_BITBANG_DELAY_US);
		digitalWrite(scl_pin, HIGH);
		delayMicroseconds(I2C_BITBANG_DELAY_US);
	}

	/* One last try at a pseudo-stop bit */
	if ((digitalRead(scl_pin) == HIGH) && (digitalRead(sda_pin) == HIGH)) {
        digitalWrite(sda_pin, HIGH);

        pinMode(sda_pin, OUTPUT);

	    digitalWrite(sda_pin, LOW);
		delayMicroseconds(I2C_BITBANG_DELAY_US);
	    digitalWrite(sda_pin, HIGH);
		delayMicroseconds(I2C_BITBANG_DELAY_US);
    }

    /*
	 * Set things back to quiescent.
	 *
	 * We rely on board_i2c_post_init() to actually reconfigure pins to
	 * be special function.
	 */
    digitalWrite(scl_pin, HIGH);
    digitalWrite(sda_pin, HIGH);

    pinMode(scl_pin, INPUT);
    pinMode(sda_pin, INPUT);
}

/***************************************************************************************
 *
 *	\fn		uint32_t time_lapse(uint32_t u32_start, uint32_t u32_end)
 *	\brief 
 *
 ***************************************************************************************/
uint32_t time_lapse(uint32_t u32_start, uint32_t u32_end) {
	if(u32_end >= u32_start)
		return u32_end - u32_start;

	return 0xFFFFFFFF - u32_end + u32_start;
}