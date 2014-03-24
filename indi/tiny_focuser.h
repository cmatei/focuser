#ifndef __TINY_FOCUSER_H
#define __TINY_FOCUSER_H

#include <indidevapi.h>
#include <indicom.h>
#include <defaultdevice.h>

#include <libusb-1.0/libusb.h>

class TinyFocuser : public INDI::DefaultDevice
{

public:
	TinyFocuser();
	~TinyFocuser();

	virtual bool Connect();
	virtual bool Disconnect();

	virtual const char *getDefaultName();

	virtual void ISGetProperties (const char *dev);

	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);

	void ISPoll();

private:
	libusb_device_handle *handle;

	INumberVectorProperty *FocusPositionNP;
	INumberVectorProperty *FocusTargetNP;

	INumberVectorProperty *FocusPWMNP;
	INumberVectorProperty *FocusSpeedNP;
	INumberVectorProperty *FocusTemperatureNP;

};


#endif
