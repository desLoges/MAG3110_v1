#include <stdint.h>

#include "mag3110.h"
#include "i2c.h" // Use I2C library found here: http://users.soe.ucsc.edu/~karplus/Arduino/libraries/i2c/


#define mag_write_reg(r,v)   (i2cWriteRegister(MAG_3110_I2C,r,v))
#define mag_read_reg(r)      (i2cReadRegister(MAG_3110_I2C,r))

inline uint8_t mag_data_ready(void) {
	return mag_read_reg(MAG_3110_DR_STATUS) & MAG_3110_ZYXDR;
}

void mag_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
	while (!mag_data_ready()) {
	} // wait for new set of data

	static uint8_t data[6];
	i2cReadRegisters(MAG_3110_I2C, MAG_3110_OUT_X_MSB, 6, data);
	*x = (data[0] << 8) + data[1];
	*y = (data[2] << 8) + data[3];
	*z = (data[4] << 8) + data[5];
}

void mag_set_offsets(void) {
	int16_t mag_x_offset = (mag_low_x + mag_high_x) / 2;
	int16_t mag_y_offset = (mag_low_y + mag_high_y) / 2;
	int16_t mag_z_offset = (mag_low_z + mag_high_z) / 2;
	static uint8_t data[6];

	data[0] = mag_x_offset >> 7;
	data[1] = (mag_x_offset << 1) & 0xFF;
	data[2] = mag_y_offset >> 7;
	data[3] = (mag_y_offset << 1) & 0xFF;
	data[4] = mag_z_offset >> 7;
	data[5] = (mag_z_offset << 1) & 0xFF;

	i2cWriteRegisters(MAG_3110_I2C, MAG_3110_OFF_X_MSB, 6, data);
}

void mag_setup(void) {
	mag_write_reg(MAG_3110_CTRL_REG2, MAG_3110_AUTO_MRST_EN);

	mag_write_reg(MAG_3110_CTRL_REG1,
			MAG_3110_SAMPLE80+MAG_3110_OVERSAMPLE1+MAG_3110_ACTIVE);

	mag_set_offsets();
}

void mag_hardIron_calibrate(void) {

	Serial.print("Calibration");
	Serial.println();

	mag_write_reg(MAG_3110_CTRL_REG2,
			mag_read_reg(MAG_3110_CTRL_REG2)| MAG_3110_RAW);

	mag_low_x = 32767;
	mag_high_x = 0x8000;
	mag_low_y = 32767;
	mag_high_y = 0x8000;
	mag_low_z = 32767;
	mag_high_z = 0x8000;
	mag_x_scale = 1;
	mag_y_scale = 1;
	mag_z_scale = 1;
	static uint16_t mag_num_calib = 0;

	int16_t x, y, z;
	mag_read_xyz(&x, &y, &z);  // discard a read from magnetometer (not RAW)

	while (mag_num_calib < MAG_CALIB_NUM_LIMIT) {

		if (mag_data_ready()) {

			mag_read_xyz(&x, &y, &z);

			if (x < mag_low_x)
				mag_low_x = x;
			if (x > mag_high_x)
				mag_high_x = x;
			if (y < mag_low_y)
				mag_low_y = y;
			if (y > mag_high_y)
				mag_high_y = y;
			if (z < mag_low_z)
				mag_low_z = z;
			if (z > mag_high_z)
				mag_high_z = z;

			mag_num_calib++;
		}
	}

	mag_num_calib = 0;

	delay(100);

	mag_calibrate = 0;
	// set the offsets to the middle of the x,y,z range
	Serial.println("Calibration done:");

	mag_x_offset = (mag_low_x + mag_high_x) / 2;
	Serial.print(" mag_low_x=");
	Serial.print(mag_low_x);
	Serial.print(" mag_high_x=");
	Serial.print(mag_high_x);
	Serial.print(" x_off=");
	Serial.println(mag_x_offset);
	mag_x_scale = 1.0 / (mag_high_x - mag_low_x);

	mag_y_offset = (mag_low_y + mag_high_y) / 2;
	Serial.print(" mag_low_y=");
	Serial.print(mag_low_y);
	Serial.print(" mag_high_y=");
	Serial.print(mag_high_y);
	Serial.print(" y_off=");
	Serial.println(mag_y_offset);
	mag_y_scale = 1.0 / (mag_high_y - mag_low_y);

	mag_z_offset = (mag_low_z + mag_high_z) / 2;
	Serial.print(" mag_low_z=");
	Serial.print(mag_low_z);
	Serial.print(" mag_high_z=");
	Serial.print(mag_high_z);
	Serial.print(" z_off=");
	Serial.println(mag_z_offset);
	mag_z_scale = 1.0 / (mag_high_z - mag_low_z);

	Serial.println();
	Serial.print("  mag_x_scale=");
	Serial.print(mag_x_scale * 10000);
	Serial.print("E-4 mag_y_scale=");
	Serial.print(mag_y_scale * 10000);
	Serial.print("E-4 mag_z_scale=");
	Serial.print(mag_z_scale * 10000);
	Serial.println("E-4");

	mag_set_offsets();

	// stop using raw mode
	mag_write_reg(MAG_3110_CTRL_REG2,
			mag_read_reg(MAG_3110_CTRL_REG2) &~ MAG_3110_RAW);

	mag_write_reg(MAG_3110_CTRL_REG1, mag_read_reg(MAG_3110_CTRL_REG1) &~ MAG_3110_ACTIVE);

	mag_write_reg(MAG_3110_CTRL_REG1,
				MAG_3110_SAMPLE1_25+MAG_3110_OVERSAMPLE1);

	mag_write_reg(MAG_3110_CTRL_REG1, mag_read_reg(MAG_3110_CTRL_REG1) | MAG_3110_ACTIVE);

	int16_t x, y, z;
	mag_read_xyz(&x, &y, &z);  // discard a read from magnetometer (was RAW)


}

void stop_mag_hardIron_calibrate(void) {
	mag_calibrate = 0;
	// set the offsets to the middle of the x,y,z range
	Serial.println("Calibration done:");

	mag_x_offset = (mag_low_x + mag_high_x) / 2;
	Serial.print(" mag_low_x=");
	Serial.print(mag_low_x);
	Serial.print(" mag_high_x=");
	Serial.print(mag_high_x);
	Serial.print(" x_off=");
	Serial.println(mag_x_offset);
	mag_x_scale = 1.0 / (mag_high_x - mag_low_x);

	mag_y_offset = (mag_low_y + mag_high_y) / 2;
	Serial.print(" mag_low_y=");
	Serial.print(mag_low_y);
	Serial.print(" mag_high_y=");
	Serial.print(mag_high_y);
	Serial.print(" y_off=");
	Serial.println(mag_y_offset);
	mag_y_scale = 1.0 / (mag_high_y - mag_low_y);

	mag_z_offset = (mag_low_z + mag_high_z) / 2;
	Serial.print(" mag_low_z=");
	Serial.print(mag_low_z);
	Serial.print(" mag_high_z=");
	Serial.print(mag_high_z);
	Serial.print(" z_off=");
	Serial.println(mag_z_offset);
	mag_z_scale = 1.0 / (mag_high_z - mag_low_z);

	Serial.println();
	Serial.print("  mag_x_scale=");
	Serial.print(mag_x_scale * 10000);
	Serial.print("E-4 mag_y_scale=");
	Serial.print(mag_y_scale * 10000);
	Serial.print("E-4 mag_z_scale=");
	Serial.print(mag_z_scale * 10000);
	Serial.println("E-4");

	mag_set_offsets();

	// stop using raw mode
	mag_write_reg(MAG_3110_CTRL_REG2,
			mag_read_reg(MAG_3110_CTRL_REG2) &~ MAG_3110_RAW);

	mag_write_reg(MAG_3110_CTRL_REG1, mag_read_reg(MAG_3110_CTRL_REG1) &~ MAG_3110_ACTIVE);

	mag_write_reg(MAG_3110_CTRL_REG1,
				MAG_3110_SAMPLE1_25+MAG_3110_OVERSAMPLE1);

	mag_write_reg(MAG_3110_CTRL_REG1, mag_read_reg(MAG_3110_CTRL_REG1) | MAG_3110_ACTIVE);

	int16_t x, y, z;
	mag_read_xyz(&x, &y, &z);  // discard a read from magnetometer (was RAW)
}
