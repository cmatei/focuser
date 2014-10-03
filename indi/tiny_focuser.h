#ifndef __TINY_FOCUSER_H
#define __TINY_FOCUSER_H

#include <libusb-1.0/libusb.h>

#include <libindi/defaultdevice.h>
#include <libindi/indifocuser.h>
#include <libindi/indifocuserinterface.h>

class TinyFocuser : public INDI::Focuser
{

public:
	TinyFocuser();
	~TinyFocuser();

	virtual bool Connect();
	virtual bool Disconnect();

	const char *getDefaultName();

	bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);

	bool initProperties();
	bool updateProperties();

	bool Abort();
	int  MoveAbs(int ticks);
	int  MoveRel(FocusDirection dir, unsigned int ticks);

	void TimerHit();

private:
	libusb_device_handle *handle;

	double lastPos, targetPos;
	double lastTemp;

	INumber TemperatureN[1];
	INumberVectorProperty TemperatureNP;

	INumber PWMN[2];
	INumberVectorProperty PWMNP;

	ISwitch SteppingS[3];
	ISwitchVectorProperty SteppingSP;

	INumber SpeedN[1];
	INumberVectorProperty SpeedNP;

	INumber EncoderN[1];
	INumberVectorProperty EncoderNP;

	bool updateTemperature();
	bool updatePosition();

	void getCurrentParams();

	// hardware stuff

	static const int REQUEST_READ  = 0xC0;
	static const int REQUEST_WRITE = 0x40;

	static const int CMD_GET_ENCODER     = 1;		    // not used in V2
	static const int CMD_SET_ENCODER     = 2;		    // not used in V2
	static const int CMD_GET_TARGET      = 3;		    // not used in V2
	static const int CMD_SET_TARGET      = 4;		    // not used in V2
	static const int CMD_SET_PWM         = 5;		    // set hold/move PWM
	static const int CMD_GET_TEMPERATURE = 6;		    // get current temperature
	static const int CMD_MOVE            = 7;		    // not used in V2
	static const int CMD_STOP            = 8;		    // not used in V2
	static const int CMD_SET_SPEED       = 9;	            // F_CPU/64 ticks per motor step
	static const int CMD_GET_POSITIONS   = 10;		    // get encoder/target values
	static const int CMD_SET_POSITIONS   = 11;		    // set encoder/target values
	static const int CMD_EXECUTE         = 12;		    // stop/run
	static const int CMD_SET_STEPPING    = 13;		    // set stepping mode
	static const int CMD_GET_VERSION     = 255;                 // version 2

	static const int STEPPING_FULL_ONEPHASE = 0;
	static const int STEPPING_FULL_TWOPHASE = 1;
	static const int STEPPING_HALF          = 2;

	int hw_get_temperature(double *temp);

	int hw_get_positions(int *current, int *target);
	int hw_set_positions(int current, int target);

	int hw_execute(int on);

	int hw_set_pwm(int hold, int move);
	int hw_set_speed(int speed);
	int hw_set_stepping(int mode);

	int hw_get_version();
};


#endif
