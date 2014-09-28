#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <getopt.h>

#include <libusb-1.0/libusb.h>

#ifndef HUGE
#define HUGE 1e20
#endif

#define DEV_MANUFACTURER "mconovici@gmail.com"
#define DEV_PRODUCT "Focuser"

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
static const int CMD_GET_VERSION     = 255;		    // get version

static const int STEPPING_FULL_ONEPHASE = 0;
static const int STEPPING_FULL_TWOPHASE = 1;
static const int STEPPING_HALF          = 2;

static libusb_device_handle *handle = NULL;

int hw_connect()
{
        libusb_device **devices;
        libusb_device *dev;
        struct libusb_device_descriptor desc;
        int i, n;
        uint32_t devID;
        char manufacturer[32];
        char product[32];

	if (libusb_init(NULL))
		return -1;

	handle = NULL;

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

                /* found it, keep the handle open */
        }

        libusb_free_device_list (devices, 1);

	return handle ? 0 : -1;
}

int hw_disconnect()
{
	if (handle)
		libusb_close(handle);
	handle = NULL;

	return 0;
}

int hw_get_temperature(double *temp)
{
	unsigned char data[2];

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_TEMPERATURE, 0, 0, data, 2, 0) != 2))
		return -1;

	//printf("data[0] = %02x, data[1] = %02x\n", data[0], data[1]);

	// two temp bytes coming from DS1820, signed 16bit

	//       x.u16 = le16toh(x.u16);
	//       *temperature = x.i16 / 16.0;
	*temp = *(int16_t *) data / 16.0;

	return 0;
}

int hw_get_positions(int *current, int *target)
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

int hw_set_positions(int current, int target)
{
	// motor_encoder = rq->wIndex.word;
	// motor_target  = rq->wValue.word;

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_SET_POSITIONS, htole16(target), htole16(current), NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int hw_execute(int on)
{
	// execute = rq->wValue.bytes[0];

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_EXECUTE, htole16(on), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int hw_set_pwm(int hold, int move)
{
	int val = (hold << 8) + move;

	// move_power = rq->wValue.bytes[0];
	// hold_power = rq->wValue.bytes[1];

	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_PWM, htole16(val), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int hw_set_speed(int speed)
{
	// OCR1A = rq->wValue.word;
	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_SPEED, htole16(speed), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int hw_set_stepping(int mode)
{
	// switch (rq->wValue.bytes[0]) {
	if (!handle || (libusb_control_transfer(handle, REQUEST_WRITE, CMD_SET_STEPPING, (mode & 0x03), 0, NULL, 0, 0) != 0))
		return -1;

	return 0;
}

int hw_get_version()
{
	unsigned char data[1];

	if (!handle || (libusb_control_transfer(handle, REQUEST_READ, CMD_GET_VERSION, 0, 0, data, 1, 0) != 1))
		return -1;

	return data[0];
}

void focuser_get_info()
{
	double temp = 0.0;
	int current = 0, target = 0;
	int r;

	printf("TinyFocuser ");
	printf("version %d, ", hw_get_version());

	r = hw_get_temperature(&temp);
	printf("temp %f (%d), ", temp, r);

	r = hw_get_positions(&current, &target);
	printf("current %d (%d), ", current, r);
	printf("target %d (%d)\n", target, r);
}


#define DEFAULT_MODE -1
#define DEFAULT_SPEED 1000
#define DEFAULT_PWM_HOLD -1
#define DEFAULT_PWM_MOVE -1
#define DEFAULT_ENCODER 32768

int main(int argc, char **argv)
{
	int c;
	int mode     = DEFAULT_MODE,
	    speed    = DEFAULT_SPEED,
	    pwm_hold = DEFAULT_PWM_HOLD,
	    pwm_move = DEFAULT_PWM_MOVE,
	    encoder  = DEFAULT_ENCODER,
	    target   = DEFAULT_ENCODER;

	if (hw_connect()) {
		fprintf(stderr, "Can't find focuser\n");
		return 1;
	}

	while ((c = getopt(argc, argv, "m:p:P:s:e:t:")) != -1) {
		switch (c) {
		case 'm':
			mode = atoi(optarg);
			break;
		case 'p':
			pwm_hold = atoi(optarg);
			break;
		case 'P':
			pwm_move = atoi(optarg);
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'e':
			encoder = atoi(optarg);
			break;
		case 't':
			target = atoi(optarg);
			break;
		}
	}

	if ((mode != DEFAULT_MODE) && hw_set_stepping(mode)) {
		fprintf(stderr, "Failed to set stepping mode %d\n", mode);
		goto out;
	}

	if ((speed != DEFAULT_SPEED) && hw_set_speed(speed)) {
		fprintf(stderr, "Failed to set speed %d\n", speed);
		goto out;
	}

	if (((pwm_hold != DEFAULT_PWM_HOLD) || (pwm_move != DEFAULT_PWM_MOVE)) &&
	    hw_set_pwm(pwm_hold, pwm_move)) {
		fprintf(stderr, "Failed to set PWM hold %d, move %d\n", pwm_hold, pwm_move);
		goto out;
	}

	if ((encoder != DEFAULT_ENCODER) && hw_set_positions(encoder, encoder)) {
		fprintf(stderr, "Failed to set position %d\n", encoder);
		goto out;
	}

	if (target != DEFAULT_ENCODER) {
		int dummy;
		if (hw_get_positions(&encoder, &dummy) ||
		    hw_set_positions(encoder, target) ||
		    hw_execute(1)) {
			fprintf(stderr, "Failed to move to position %d\n", target);
			goto out;
		}
	}

	focuser_get_info();

out:
	hw_disconnect();

	return 0;
}
