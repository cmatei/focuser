
#include <memory>

#include "tiny_focuser.h"

#ifndef HUGE
#define HUGE 1e20
#endif

#define DEV_MANUFACTURER "mconovici@gmail.com"
#define DEV_PRODUCT "Focuser"

#define INDI_NAME "TinyFocuser"

#define POLLMS 1000

#define currentPWMHold     PWMN[0].value
#define currentPWMMove     PWMN[1].value
#define currentSpeed       SpeedN[0].value
#define currentEncoder     EncoderN[0].value
#define currentTemperature TemperatureN[0].value
#define currentPosition    FocusAbsPosN[0].value
#define currentStepping    SteppingS[0].s

std::auto_ptr<TinyFocuser> focuser(0);

void ISInit()
{
	static int isInit = false;

	if (isInit)
		return;

	isInit = true;
	if (focuser.get() == 0)
		focuser.reset(new TinyFocuser());
}

void ISGetProperties(const char *dev)
{
	ISInit();
	focuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	ISInit();
	focuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	ISInit();
	focuser->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	ISInit();
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

void ISSnoopDevice(XMLEle *root)
{
	ISInit();
	focuser->ISSnoopDevice(root);
}


TinyFocuser::TinyFocuser()
{
	handle = NULL;

	// canAbsMove, canRelMove, canAbort, variableSpeed
	setFocuserFeatures(true, true, true, false);
}

TinyFocuser::~TinyFocuser()
{
	if (handle)
		libusb_close(handle);
}

const char *TinyFocuser::getDefaultName()
{
	return INDI_NAME;
}

bool TinyFocuser::initProperties()
{
	INDI::Focuser::initProperties();

	/* Relative and absolute movement */
	FocusRelPosN[0].min = -65535.;
	FocusRelPosN[0].max = 65535.;
	FocusRelPosN[0].value = 0;
	FocusRelPosN[0].step = 1;

	FocusAbsPosN[0].min = 0.;
	FocusAbsPosN[0].max = 65535.;
	FocusAbsPosN[0].value = 0;
	FocusAbsPosN[0].step = 1;

	/* Temperature */
	IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Degrees (C)", "%6.2f", -HUGE, HUGE, 0., 0.);
	IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	/* Current position encoder */
	IUFillNumber(&EncoderN[0], "CURRENT_POSITION", "Value", "%.f", 0., 65535., 1., 0.);
	IUFillNumberVector(&EncoderNP, EncoderN, 1, getDeviceName(), "CURRENT_POSITION", "Position Encoder", OPTIONS_TAB, IP_WO, 0, IPS_IDLE);

	/* Stepping mode */
	IUFillSwitch(&SteppingS[0], "MOTOR_ONEPHASE", "One phase", ISS_ON);
	IUFillSwitch(&SteppingS[1], "MOTOR_TWOPHASE", "Two phase", ISS_OFF);
	IUFillSwitch(&SteppingS[2], "MOTOR_HALFSTEP", "Halfstep",  ISS_OFF);
	IUFillSwitchVector(&SteppingSP, SteppingS, 3, getDeviceName(), "MOTOR_MODE", "Stepping mode", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	/* Stepping speed */
	IUFillNumber(&SpeedN[0], "MOTOR_SPEED", "Speed (%)", "%.f", 1., 100., 1., 1.);
	IUFillNumberVector(&SpeedNP, SpeedN, 1, getDeviceName(), "MOTOR_SPEED", "Stepping speed", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	/* PWM for hold/move */
	IUFillNumber(&PWMN[0], "MOTOR_PWM_HOLD", "Hold (%)", "%.f", 0., 100., 1., 0.);
	IUFillNumber(&PWMN[1], "MOTOR_PWM_MOVE", "Move (%)", "%.f", 0., 100., 1., 100.);
	IUFillNumberVector(&PWMNP, PWMN, 2, getDeviceName(), "MOTOR_POWER", "Motor Power", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
}

bool TinyFocuser::updateProperties()
{

	INDI::Focuser::updateProperties();

	if (isConnected()) {
		defineNumber(&TemperatureNP);
		defineNumber(&EncoderNP);
		defineSwitch(&SteppingSP);
		defineNumber(&SpeedNP);
		defineNumber(&PWMNP);

		updateFocuserData();
	} else {
		deleteProperty(TemperatureNP.name);
		deleteProperty(EncoderNP.name);
		deleteProperty(SteppingSP.name);
		deleteProperty(SpeedNP.name);
		deleteProperty(PWMNP.name);
	}
}

void TinyFocuser::updateFocuserData()
{
	int current = 0, target = 0;
	double temp = 0.0;
	int statuschange, valuechange;
	IPState s;

	/* temperature */
	statuschange = 0; valuechange = 0;
	if ((s = hw_get_temperature(&temp) ? IPS_ALERT : IPS_OK) == IPS_OK) {
		if (currentTemperature != temp) {
			currentTemperature = temp;
			valuechange = 1;
		}
	}
	if (TemperatureNP.s != s)
		statuschange = 1;
	TemperatureNP.s = s;
	if (statuschange || valuechange)
		IDSetNumber(&TemperatureNP, NULL);

	/* position */
	statuschange = 0; valuechange = 0;
	if ((s = hw_get_positions(&current, &target) ? IPS_ALERT : IPS_OK) == IPS_OK) {
		if (current != target) {
			s = IPS_BUSY;
		} else {
			FocusRelPosN[0].value = 0;
		}

		if (current != currentPosition) {
			valuechange = 1;
			currentPosition == current;
		}
	}
	if (FocusAbsPosNP.s != s || FocusRelPosNP.s != s)
		statuschange = 1;
	FocusAbsPosNP.s = FocusRelPosNP.s = s;
	if (statuschange || valuechange) {
		IDSetNumber(&FocusAbsPosNP, NULL);
		IDSetNumber(&FocusRelPosNP, NULL);
	}
}

bool TinyFocuser::Abort()
{
	int current, target;

	if (hw_execute(0) || hw_get_positions(&current, &target) || hw_set_positions(current, current))
		return false;

	return true;
}

// -1 error
// 0 moved
// 1 moving
int TinyFocuser::MoveAbs(int ticks)
{
	int current = 0, target;

	if (hw_execute(0) || hw_get_positions(&current, &target))
		return -1;

	if (current == ticks)
		return 0;

	if (hw_set_positions(current, ticks) || hw_execute(1))
		return -1;

	return 1;
}

bool TinyFocuser::Connect()
{
	libusb_context *ctx;
        libusb_device **devices;
        libusb_device *dev;
        struct libusb_device_descriptor desc;
        int i, n;
        uint32_t devID;
        char manufacturer[32];
        char product[32];

	if (isConnected()) {
		IDMessage(getDeviceName(), "Already connected.");
		return true;
	}

	if (libusb_init(NULL)) {
		DEBUG(INDI::Logger::DBG_ERROR, "libusb_init failed!");
		return false;
	}

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

		SetTimer(POLLMS);

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

	if (handle)
		libusb_close(handle);
	handle = NULL;

	IDMessage(getDeviceName(), "Disconnected.");
	return true;
}


bool TinyFocuser::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	if (!strcmp(dev, getDeviceName())) {

		if (!strcmp(name, PWMNP.name)) {
			if (IUUpdateNumber(&PWMNP, values, names, n) < 0)
				return false;

			if (hw_execute(0) || hw_set_pwm(currentPWMHold, currentPWMMove))
				PWMNP.s = IPS_ALERT;
			else
				PWMNP.s = IPS_OK;

			IDSetNumber(&PWMNP, NULL);
			return true;
		}

		if (!strcmp(name, SpeedNP.name)) {
			if (IUUpdateNumber(&SpeedNP, values, names, n) < 0)
				return false;

			/* ticks per step @ 250kHz
			    125  => 2000 steps/sec = 10 rot/sec
			    2500 => 100 steps/sec  = 0.5 rot/sec */

			if (hw_execute(0) || hw_set_speed(125 + (100.0 - currentSpeed) * 23.75))
				SpeedNP.s = IPS_ALERT;
			else
				SpeedNP.s = IPS_OK;

			IDSetNumber(&SpeedNP, NULL);
			return true;
		}

		if (!strcmp(name, EncoderNP.name)) {
			if (IUUpdateNumber(&EncoderNP, values, names, n) < 0)
				return false;

			Abort();

			if (hw_set_positions(currentEncoder, currentEncoder))
				EncoderNP.s = IPS_ALERT;
			else
				EncoderNP.s = IPS_OK;

			IDSetNumber(&EncoderNP, NULL);
			return true;
		}
	}

	return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool TinyFocuser::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (!strcmp(dev, getDeviceName())) {
		if (!strcmp(name, SteppingSP.name)) {
			if (IUUpdateSwitch(&SteppingSP, states, names, n) < 0)
				return false;

			if (hw_execute(0) || hw_set_stepping(currentStepping))
				SteppingSP.s = IPS_ALERT;
			else
				SteppingSP.s = IPS_OK;

			IDSetSwitch(&SteppingSP, NULL);
			return true;
		}
	}

	return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}


void TinyFocuser::TimerHit()
{
	if (!isConnected())
		return;

	DEBUG(INDI::Logger::DBG_DEBUG, "TimerHit!");

	updateFocuserData();

	SetTimer(POLLMS);
}


int TinyFocuser::hw_get_version()
{
	unsigned char data[1];

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_VERSION, 0, 0, data, 1, 0) != 1))
		return -1;

	return data[0];
}

int TinyFocuser::hw_get_temperature(double *temp)
{
	unsigned char data[2];

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_TEMPERATURE, 0, 0, data, 2, 0) != 2))
		return -1;

	// two temp bytes coming from DS1820, signed 16bit

	//       x.u16 = le16toh(x.u16);
	//       *temperature = x.i16 / 16.0;


	return 0;
}

int TinyFocuser::hw_get_positions(int *current, int *target)
{
	unsigned char data[4];

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_POSITIONS, 0, 0, data, 4, 0) != 4))
		return -1;

	// static uint16_t positions[2] = { 32768, 32768 };
        // #define motor_encoder (positions[0])
        // #define motor_target  (positions[1])

	*current = (data[1] << 8) + data[0];
	*target  = (data[3] << 8) + data[2];

	return 0;
}

int TinyFocuser::hw_set_positions(int current, int target)
{
	// motor_encoder = rq->wIndex.word;
	// motor_target  = rq->wValue.word;

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_SET_POSITIONS, htole16(target), htole16(current), NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int TinyFocuser::hw_execute(int on)
{
	// execute = rq->wValue.bytes[0];

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_EXECUTE, htole16(on), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int TinyFocuser::hw_set_pwm(int hold, int move)
{
	int val = (hold << 8) + move;

	// move_power = rq->wValue.bytes[0];
	// hold_power = rq->wValue.bytes[1];

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_PWM, htole16(val), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int TinyFocuser::hw_set_speed(int speed)
{
	// OCR1A = rq->wValue.word;

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_SPEED, htole16(speed), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int TinyFocuser::hw_set_stepping(int mode)
{
	// switch (rq->wValue.bytes[0]) {

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_SPEED, (mode & 0x03), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}
