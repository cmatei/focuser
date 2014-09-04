#ifndef __FOCUSER_H
#define __FOCUSER_H

#define TINYFOCUSER_VERSION 2

enum {
	CMD_GET_ENCODER = 1,		     /* not used in V2 */
	CMD_SET_ENCODER,		     /* not used in V2 */
	CMD_GET_TARGET,                      /* not used in V2 */
	CMD_SET_TARGET,                      /* not used in V2 */

	CMD_SET_PWM,		             /* set hold/move PWM */

	CMD_GET_TEMPERATURE,	             /* get current temperature */

	CMD_MOVE,			     /* not used in V2 */
	CMD_STOP,			     /* not used in V2 */

	CMD_SET_SPEED,                       /* F_CPU/64 ticks per motor step */

	CMD_GET_POSITIONS,	             /* get encoder/target values */
	CMD_SET_POSITIONS,	             /* set encoder/target values */
	CMD_EXECUTE,	                     /* stop/run */
	CMD_SET_STEPPING,		     /* set stepping mode */

	CMD_GET_VERSION = 255,	             /* get version to adjust command set */
};

#endif
