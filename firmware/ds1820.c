#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "ds1820.h"

static uint8_t ds1820_state = DS1820_IDLE;
static uint8_t ds1820[9];

extern uint8_t temperature[];

/* NOTE: can't disable interrupts for more than a few microseconds, or USB won't work
   properly. All delays here may be interrupted by USB, breaking the 1-wire protocol.
   Therefore, discard data on CRC mismatch */

static uint8_t ds1820_reset()
{
	uint8_t err = 0;

	DS1820_OUT &= ~DS1820_1;
	DS1820_DDR |= DS1820_1;
	_delay_us(500);

	DS1820_OUT |= DS1820_1;
	DS1820_DDR &= ~DS1820_1;
	_delay_us(65);
	err = DS1820_IN & DS1820_1;		// no presence detect

	_delay_us(490 - 65);
	if ((DS1820_IN & DS1820_1) == 0)		// short circuit
		err = 1;

	return err;
}

static uint8_t ds1820_bit_io(uint8_t b)
{
	DS1820_OUT &= ~DS1820_1;
	DS1820_DDR |= DS1820_1;
	_delay_us(2);
	if (b) {
		DS1820_DDR &= ~DS1820_1;
		DS1820_OUT |= DS1820_1; 	     // pullup
	}
	_delay_us(15-2);
	if ((DS1820_IN & DS1820_1) == 0 )
		b = 0;
	_delay_us(60-15);
	DS1820_DDR &= ~DS1820_1;
	DS1820_OUT |= DS1820_1;

	return b;
}

static uint8_t ds1820_write(uint8_t b)
{
	uint8_t i = 8, j;
	do {
		j = ds1820_bit_io(b & 1);
		b >>= 1;
		if (j)
			b |= 0x80;
	} while (--i);
	return b;
}


static uint8_t ds1820_read()
{
	return ds1820_write(0xFF);
}

static void ds1820_read_scratchpad()
{
        uint8_t i, crc;

	ds1820_reset();
        ds1820_write(DS1820_CMD_SKIP_ROM);
        ds1820_write(DS1820_CMD_READ);

	crc = 0;
        for (i = 0; i < 9; i++) {
                ds1820[i] = ds1820_read();
		crc = _crc_ibutton_update(crc, ds1820[i]);
	}

	/* only update temperature if CRC ok */
	if (!crc) {
		temperature[0] = ds1820[0];
		temperature[1] = ds1820[1];
	}
}

void ds1820_state_machine()
{
	switch (ds1820_state) {

	case DS1820_IDLE:
		if (ds1820_reset())
			return;

		ds1820_write(DS1820_CMD_SKIP_ROM);
		ds1820_write(DS1820_CMD_CONVERT_T);

		ds1820_state = DS1820_CONVERT;
		break;

	case DS1820_CONVERT:
		if (ds1820_read() == 0xFF)
			ds1820_state = DS1820_READ;
		break;

	case DS1820_READ:
		ds1820_read_scratchpad();
		ds1820_state = DS1820_IDLE;
		break;
	}
}
