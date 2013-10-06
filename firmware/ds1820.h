#ifndef __DS1820_H
#define __DS1820_H

enum {
	DS1820_IDLE = 1,
	DS1820_CONVERT,
	DS1820_READ,
};

/* DS1820 pins */
#define DS1820_1 _BV(PD4)

#define DS1820_OUT PORTD
#define DS1820_IN  PIND
#define DS1820_DDR DDRD

#define DS1820_CMD_SKIP_ROM        0xCC
#define DS1820_CMD_CONVERT_T       0x44
#define DS1820_CMD_READ            0xBE

extern void ds1820_state_machine();

#endif
