#ifndef __FOCUSER_H
#define __FOCUSER_H

enum {
	CMD_GET_ENCODER = 1,	/* get current encoder value */
	CMD_SET_ENCODER,	/* set current encoder value */

	CMD_GET_TARGET,		/* get target encoder value */
	CMD_SET_TARGET,		/* set target encoder value */

	CMD_SET_PWM,		/* set movement/hold pwm */

	CMD_GET_TEMPERATURE,	/* get current temperature */

	CMD_MOVE,		/* start moving in specified direction */
	CMD_STOP,		/* abort movement */

	CMD_SET_SPEED,          /* steps per tick */
};

#endif
