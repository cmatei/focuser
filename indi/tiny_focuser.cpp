#define _BSD_SOURCE

#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <endian.h>

#include "tiny_focuser.h"
#include "config.h"

#define DEV_MANUFACTURER "mconovici@gmail.com"
#define DEV_PRODUCT "Focuser"

#define INDI_NAME "TinyFocuser"

static TinyFocuser *focuser = NULL;
static int initialized = 0;

const int POLLMS = 1000;

#define REQUEST_READ  0xC0
#define REQUEST_WRITE 0x40

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

typedef union {
	uint32_t u32;
	uint16_t u16;
	int16_t  i16;

	unsigned char arr[4];
} byte_shuffle_t;

static int tiny_get_encoder(libusb_device_handle *handle, unsigned int *val)
{
	byte_shuffle_t x;

	if (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_ENCODER,
				    0, 0, x.arr, 4, 0) != 4)
		return 1;

	*val = le32toh(x.u32);
	return 0;
}

static int tiny_set_encoder(libusb_device_handle *handle, unsigned int val)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_ENCODER,
				    (val & 0xffff0000) >> 16, val & 0xffff,
				    NULL, 0, 0) != 0)
		return 1;

	return 0;
}

static int tiny_get_target(libusb_device_handle *handle, unsigned int *val)
{
	byte_shuffle_t x;

	if (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_TARGET,
				    0, 0, x.arr, 4, 0) != 4)
		return 1;

	*val = le32toh(x.u32);
	return 0;
}

static int tiny_set_target(libusb_device_handle *handle, unsigned int val)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_TARGET,
				    (val & 0xffff0000) >> 16, val & 0xffff,
				    NULL, 0, 0) != 0)
		return 1;

	return 0;
}

static int tiny_get_temperature(libusb_device_handle *handle, double *temperature)
{
	byte_shuffle_t x;

	if (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_TEMPERATURE,
				    0, 0, x.arr, 2, 0) != 2)
		return 1;

	x.u16 = le16toh(x.u16);
	*temperature = x.i16 / 16.0;

	return 0;
}

static int tiny_set_speed(libusb_device_handle *handle, int ticks_per_step)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_SPEED,
				    ticks_per_step, 0, NULL, 0, 0) != 0)
		return 1;

	return 0;
}

static int tiny_set_pwm(libusb_device_handle *handle, int hold_pwm, int move_pwm)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_PWM,
				    (hold_pwm << 8) + move_pwm, 0, NULL, 0, 0) != 0)
		return 1;

	return 0;
}

static int tiny_move(libusb_device_handle *handle, int dir)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_MOVE,
				    dir, 0, NULL, 0, 0) != 0)
		return 1;

	return 0;
}

static int tiny_stop(libusb_device_handle *handle)
{
	if (libusb_control_transfer(handle, REQUEST_WRITE, CMD_STOP,
				    0, 0, NULL, 0, 0) != 0)
		return 1;

	return 0;
}


void ISPoll(void *ptr)
{
	focuser->ISPoll();
	IEAddTimer(POLLMS, ISPoll, NULL);
}

static void initialize()
{
	if (!initialized) {
		initialized = 1;
		libusb_init (NULL);

		focuser = new TinyFocuser();
		IEAddTimer(POLLMS, ISPoll, NULL);
	}
}



void ISGetProperties(const char *dev)
{
	if (dev && strcmp(dev, INDI_NAME))
		return;

	initialize();
	focuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (dev && strcmp(dev, INDI_NAME))
		return;

	initialize();
	focuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	if (dev && strcmp(dev, INDI_NAME))
		return;

	initialize();
	focuser->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	if (dev && strcmp(dev, INDI_NAME))
		return;

	initialize();
	focuser->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
	INDI_UNUSED(dev);
	INDI_UNUSED(name);
	INDI_UNUSED(sizes);
	INDI_UNUSED(blobsizes);
	INDI_UNUSED(blobs);
	INDI_UNUSED(formats);
	INDI_UNUSED(names);
	INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
	INDI_UNUSED(root);
}



TinyFocuser::TinyFocuser()
{
	char skeleton[PATH_MAX];
	struct stat st;
	char *skel = getenv("INDISKEL");

	snprintf(skeleton, PATH_MAX, "%s/indi_tinyfocuser_sk.xml", INDI_DATA_DIR);

	if (skel)
		buildSkeleton(skel);
	else if (stat(skeleton, &st) == 0)
		buildSkeleton(skeleton);
	else
		IDLog("No skeleton file was found.\n");

	FocusPositionNP    = getNumber("FOCUS_POSITION");
	FocusTargetNP      = getNumber("FOCUS_POSITION_REQUEST");
	FocusTemperatureNP = getNumber("FOCUS_TEMPERATURE");
	FocusPWMNP         = getNumber("FOCUS_POWER");
	FocusSpeedNP       = getNumber("FOCUS_SPEED");

	handle = NULL;
}

TinyFocuser::~TinyFocuser()
{
	Disconnect();

	delete FocusPositionNP;
	delete FocusTargetNP;
	delete FocusTemperatureNP;
	delete FocusPWMNP;
	delete FocusSpeedNP;
}

const char *TinyFocuser::getDefaultName()
{
	return (char *) INDI_NAME;
}


bool TinyFocuser::Connect()
{
        libusb_device **devices;
        libusb_device *dev;
        struct libusb_device_descriptor desc;
        int i, n;
        uint32_t devID;
        char manufacturer[32];
        char product[32];

        Disconnect();

        n = libusb_get_device_list (NULL, &devices);

        for (i = 0; i < n; i++) {
                dev = devices[i];
                if (libusb_get_device_descriptor(dev, &desc) < 0)
                        continue;


                /* voti.nl USB VID/PID for vendor class devices */
                devID = (desc.idVendor << 16) + desc.idProduct;
                if (devID != 0x16C005DC)
                        continue;

                if (libusb_open(dev, &handle) < 0)
                        continue;

                if ((libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, (unsigned char *) manufacturer, 32) < 0) ||
                    (libusb_get_string_descriptor_ascii(handle, desc.iProduct, (unsigned char *) product, 32) < 0)) {
                        libusb_close(handle);
                        continue;
                }

                if (strcmp(manufacturer, DEV_MANUFACTURER) || strcmp(product, DEV_PRODUCT)) {
                        libusb_close(handle);
                        continue;
                }


                libusb_free_device_list (devices, 1);

                /* found it, keep the handle open */
		IDMessage(getDeviceName(), "Connected.");

                return true;
        }

        libusb_free_device_list (devices, 1);
        handle = NULL;

        return false;
}

bool TinyFocuser::Disconnect()
{
	if (!isConnected())
		return true;

	libusb_close(handle);
	handle = NULL;

	IDMessage(getDeviceName(), "Disconnected.");

	return true;
}

void TinyFocuser::ISGetProperties (const char *dev)
{
	INDI::DefaultDevice::ISGetProperties(dev);
}

bool TinyFocuser::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	int dir;

	if (INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n))
		return true;

	if (!isConnected()) {
		resetProperties();
		IDMessage(getDeviceName(), "Connect before issuing commands.");
		return false;
	}

	if (!strcmp(name, "FOCUS_POSITION")) {
		IUUpdateNumber(FocusPositionNP, values, names, n);

		if (!tiny_set_encoder(handle, FocusPositionNP->np[0].value))
			FocusPositionNP->s = IPS_OK;
		else
			FocusPositionNP->s = IPS_ALERT;

		IDSetNumber(FocusPositionNP, NULL);

		return true;
	}

	if (!strcmp(name, "FOCUS_POSITION_REQUEST")) {
		IUUpdateNumber(FocusTargetNP, values, names, n);

		dir = (FocusTargetNP->np[0].value > FocusPositionNP->np[0].value) ? 1 : 0;
		if (tiny_set_target(handle, FocusTargetNP->np[0].value) ||
		    tiny_move(handle, dir)) {

			FocusTargetNP->s = IPS_ALERT;
		} else {
			FocusTargetNP->s = IPS_BUSY;
		}

		IDSetNumber(FocusTargetNP, NULL);

		return true;
	}

	if (!strcmp(name, "FOCUS_POWER")) {
		IUUpdateNumber(FocusPWMNP, values, names, n);

		if (!tiny_set_pwm(handle, FocusPWMNP->np[0].value / 100.0 * 255, FocusPWMNP->np[1].value / 100.0 * 255))
			FocusPWMNP->s = IPS_OK;
		else
			FocusPWMNP->s = IPS_ALERT;

		IDSetNumber(FocusPWMNP, NULL);

		return true;
	}

	if (!strcmp(name, "FOCUS_SPEED")) {
		IUUpdateNumber(FocusSpeedNP, values, names, n);

		if (!tiny_set_speed(handle, FocusSpeedNP->np[0].value))
			FocusSpeedNP->s = IPS_OK;
		else
			FocusSpeedNP->s = IPS_ALERT;

		IDSetNumber(FocusSpeedNP, NULL);

		return true;
	}

	return false;
}

bool TinyFocuser::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n))
		return true;

	return false;
}


void TinyFocuser::ISPoll()
{
	unsigned int pos, target;
	double d;

	if (!isConnected())
		return;

	// encoder
	if (!tiny_get_encoder(handle, &pos)) {
		FocusPositionNP->np[0].value = pos;
		FocusPositionNP->s = IPS_OK;
	} else {
		FocusPositionNP->s = IPS_ALERT;
	}
	IDSetNumber(FocusPositionNP, NULL);

	// target
	if (!tiny_get_target(handle, &target)) {
		FocusTargetNP->np[0].value = target;
		FocusTargetNP->s = (target == pos) ? IPS_OK : IPS_BUSY;
	} else {
		FocusTargetNP->s = IPS_ALERT;
	}
	IDSetNumber(FocusTargetNP, NULL);

	// temperature
	if (!tiny_get_temperature(handle, &d)) {
		FocusTemperatureNP->np[0].value = d;
		FocusTemperatureNP->s = IPS_OK;
	} else {
		FocusTemperatureNP->s = IPS_ALERT;
	}
	IDSetNumber(FocusTemperatureNP, NULL);
}
