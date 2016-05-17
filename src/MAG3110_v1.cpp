// Do not remove the include below
#include "MAG3110.h"
#include "printf.h"

extern HardwareSerial Serial;


// minimum and maximum values seen during calibration time
//int16_t mag_low_x, mag_high_x, mag_low_y, mag_high_y, mag_low_z, mag_high_z;

//uint8_t mag_calibrate = 1;	  // turn on during calibration
//uint16_t mag_num_calib = 0;  // number of readings when calibrating

// scale factors for heading
//float mag_x_scale = 1.853, mag_y_scale = 1.91, mag_z_scale = 100;

// offsets for non-RAW mode
//int16_t mag_x_offset = 3720, mag_y_offset = -2557, mag_z_offset = -245;

mag_values_t mag_values;


void print_heading(int16_t x, int16_t y, int16_t z) {

	Serial.print("x=");
	Serial.print(x);
	Serial.print(" y=");
	Serial.print(y);
	Serial.print(" z=");
	Serial.println(z);

	float mag_x_scale = 1.0 / (mag_high_x - mag_low_x);
	float mag_y_scale = 1.0 / (mag_high_y - mag_low_y);

	float heading = atan2(-y * mag_y_scale, x * mag_x_scale);
	if (heading < 0) {
		heading += 2 * PI;  // correct for when the heading is negative
	}
	float headingDegrees = heading * degrees_per_radian;  // convert to degrees

	Serial.print("heading: ");
	Serial.print(headingDegrees);
	Serial.println();
}





void print_registers(void) {
	Serial.println("MAG3110 registers:");

	for (int i = 0; i < 0x12; i++) {
		Serial.print(i, HEX);
		Serial.print(": 0x");
		Serial.println(mag_read_reg(i), HEX);
		delay(2);
	}
}

void setup(void) {
	Serial.begin(115200);
	Serial.println("MAG3110");
	i2cInit();
	i2cSetBitrate(100);  // try 100kHz
	delay(20);
	mag_setup();
	delay(50);

	start_mag_calibrate();


	print_registers();

	while (!Serial.available())
		;

}

void loop(void) {
	int16_t mag_x, mag_y, mag_z;
	mag_read_xyz(&mag_x, &mag_y, &mag_z);  // read from sensor
	print_heading(mag_x, mag_y, mag_z);  // print the heading
	delay(1000);
}
