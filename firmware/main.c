
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds1820.h"
#include "usbdrv.h"

#include "focuser.h"

#define DEFAULT_PWM_MOVE 255
#define DEFAULT_PWM_HOLD 128

/* steps per second */
#define STEP_HZ 500
#define TICKS_PER_STEP ((F_CPU / 64) / STEP_HZ)

uint32_t motor_position = (1UL << 31);
uint32_t motor_target   = (1UL << 31);

uint8_t move_power = DEFAULT_PWM_MOVE;
uint8_t hold_power = DEFAULT_PWM_HOLD;

uint8_t move_dir = 1;
uint8_t movement = 0;

uint8_t temperature[2];

#define COIL_1A PB6
#define COIL_1B PB1
#define COIL_2A PB0
#define COIL_2B PB7

uint8_t motor_phases[8] = {
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(1 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
};

uint8_t current_phase = 0;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (usbRequest_t *) data;

	switch (rq->bRequest) {
	case CMD_GET_ENCODER:
		usbMsgPtr = (void *) &motor_position;
		return 4;

	case CMD_SET_ENCODER:
		//motor_position = ((uint32_t) rq->wValue.word << 16) + rq->wIndex.word;
		((uint8_t *) &motor_position)[0] = rq->wIndex.bytes[0];
		((uint8_t *) &motor_position)[1] = rq->wIndex.bytes[1];

		((uint8_t *) &motor_position)[2] = rq->wValue.bytes[0];
		((uint8_t *) &motor_position)[3] = rq->wValue.bytes[1];
		break;

	case CMD_GET_TARGET:
		usbMsgPtr = (void *) &motor_target;
		return 4;

	case CMD_SET_TARGET:
		//motor_target = ((uint32_t) rq->wValue.word << 16) + rq->wIndex.word;
		((uint8_t *) &motor_target)[0] = rq->wIndex.bytes[0];
		((uint8_t *) &motor_target)[1] = rq->wIndex.bytes[1];

		((uint8_t *) &motor_target)[2] = rq->wValue.bytes[0];
		((uint8_t *) &motor_target)[3] = rq->wValue.bytes[1];
		break;

	case CMD_GET_TEMPERATURE:
		usbMsgPtr = (void *) &temperature;
		return 2;

	case CMD_STOP:
		movement = 0;
		break;

	case CMD_MOVE:
		move_dir = rq->wValue.bytes[0];
		movement = 1;
		OCR0A = move_power;
		break;

	case CMD_SET_PWM:
		move_power = rq->wValue.bytes[0];
		hold_power = rq->wValue.bytes[1];
		break;

		/* debug only, F_CPU/64 ticks per step */
	case CMD_SET_SPEED:
		OCR1A = rq->wValue.word;
		break;
	}
	return 0;
}

/* NOTE: ISR_NOBLOCK is required for USB */
ISR (TIMER1_COMPA_vect, ISR_NOBLOCK)
{
	if (!movement || (motor_position == motor_target)) {
		movement = 0;
		OCR0A = hold_power;
		return;
	}

	if (move_dir) {
		motor_position++;
		current_phase++;
	} else {
		motor_position--;
		current_phase--;
	}

	current_phase &= 7;
	PORTB = motor_phases[current_phase];
}


int main()
{
	usbDeviceDisconnect();

	/* ds1820 on PORTD */
	DS1820_DDR &= ~DS1820_1;
	DS1820_OUT |= DS1820_1;

	/* motor phase + PWM outputs */
	DDRB  = _BV(PB0) | _BV(PB1) | _BV(PB6) | _BV(PB7) | _BV(PB2);
	PORTB = motor_phases[0];

	/* motor enable by PWM on OC0A/PB2.
	   Timer0 in Fast PWM mode, non-inverting output, clk_io/64 (~976Hz PWM) */
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) |
		 (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << WGM02) |
		 (0 << CS02) | (1 << CS01) | (1 << CS00);
	OCR0A  = DEFAULT_PWM_MOVE;

	/* motor phasing in Timer1 overflow interrupt.
	   Timer1 in CTC mode, clk_io/64, every 500 => 500Hz, maybe increase with tests
	 */
	OCR1A = TICKS_PER_STEP;
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) |
		 (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << WGM13) | (1 << WGM12) |
		 (0 << CS12) | (1 << CS11) | (1 << CS10);
	TIMSK  = (1 << OCIE1A);


        /* start usb, enable ints */
        _delay_ms(200);
        usbDeviceConnect();
        usbInit();

        sei();

	for (;;) {
		usbPoll();
		ds1820_state_machine();
	}


	return 0;
}
