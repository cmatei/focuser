
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "ds1820.h"
#include "usbdrv.h"

#include "focuser.h"

#define COIL_1A PB6
#define COIL_1B PB1
#define COIL_2A PB0
#define COIL_2B PB7


PROGMEM const uint8_t phases_fullstep_wave[4] = {
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
};

PROGMEM const uint8_t phases_fullstep[4] = {
	(1 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
};

PROGMEM const uint8_t phases_halfstep[8] = {
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(1 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (1 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (0 << COIL_2B),
	(0 << COIL_1A) | (1 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
	(0 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
	(1 << COIL_1A) | (0 << COIL_1B) | (0 << COIL_2A) | (1 << COIL_2B),
};

/* wave drive, or single phase on */
#define DEFAULT_PWM_MOVE 255
#define DEFAULT_PWM_HOLD 0

static const uint8_t *motor_phases = phases_fullstep_wave;
static uint8_t num_phases = 4;

static uint8_t move_power = DEFAULT_PWM_MOVE;
static uint8_t hold_power = DEFAULT_PWM_HOLD;


/* steps per second */
#define STEP_HZ 250
#define TICKS_PER_STEP ((F_CPU / 64) / STEP_HZ)

/* in miliseconds */
#define POWER_CHANGE_DELAY 10

static uint16_t positions[2] = { 32768, 32768 };
#define motor_encoder (positions[0])
#define motor_target  (positions[1])

uint8_t temperature[2];

static uint8_t version = TINYFOCUSER_VERSION;


/* http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_regbind */
register uint8_t execute   asm("r2");
register uint8_t moving    asm("r3");
register uint8_t direction asm("r4");

register uint8_t current_phase asm("r5");

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (usbRequest_t *) data;

	switch (rq->bRequest) {
	case CMD_GET_VERSION:
		usbMsgPtr = (void *) &version;
		return 1;

	case CMD_GET_TEMPERATURE:
		usbMsgPtr = (void *) &temperature;
		return 2;

	case CMD_SET_SPEED:
		OCR1A = rq->wValue.word;
		break;

	case CMD_SET_PWM:
		move_power = rq->wValue.bytes[0];
		hold_power = rq->wValue.bytes[1];
		break;

	case CMD_SET_STEPPING:
		switch (rq->wValue.bytes[0]) {
		case 1:
			motor_phases = phases_fullstep;
			num_phases = 4;
			break;
		case 2:
			motor_phases = phases_halfstep;
			num_phases = 8;
			break;
		case 0:
		default:
			motor_phases = phases_fullstep_wave;
			num_phases = 4;
			break;
		}

		execute = 0;
		current_phase = 0;
		break;

	case CMD_GET_POSITIONS:
		usbMsgPtr = (void *) positions;
		return 4;

	case CMD_SET_POSITIONS:
		motor_encoder = rq->wIndex.word;
		motor_target  = rq->wValue.word;

		/* will need an explicit move command after positions set */
		execute = 0;
		break;

	case CMD_EXECUTE:
		execute = rq->wValue.bytes[0];
		break;
	}
	return 0;
}


/* NOTE: ISR_NOBLOCK is required for USB */
ISR (TIMER1_COMPA_vect, ISR_NOBLOCK)
{

	if (!execute || !moving || (motor_encoder == motor_target))
		return;

	if (direction) {
		motor_encoder++;
		current_phase++;
	} else {
		motor_encoder--;
		current_phase--;
	}

	if (current_phase >= num_phases)
		current_phase = 0;

	PORTB = pgm_read_byte_near(motor_phases + current_phase);
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
	   Timer1 in CTC mode, clk_io/64, e.g. every 500 => 500Hz, maybe increase with tests
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

		if (!execute || (motor_encoder == motor_target)) {
			moving = 0;

			_delay_ms(POWER_CHANGE_DELAY);
			OCR0A = hold_power;

			continue;
		}

		direction = (motor_encoder < motor_target);

		OCR0A = move_power;
		_delay_ms(POWER_CHANGE_DELAY);

		moving = 1;
	}


	return 0;
}
