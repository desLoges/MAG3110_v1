#include <stdint.h>

// I2C 7-bit address of the magnetometer
#define MAG_3110_I2C  0x0E

// registers on the magnetometer
#define MAG_3110_DR_STATUS 0x00

// add 1 for LSB
#define MAG_3110_OUT_X_MSB 0x01
#define MAG_3110_OUT_Y_MSB 0x03
#define MAG_3110_OUT_Z_MSB 0x05

// add 1 for LSB  // user offset
#define MAG_3110_OFF_X_MSB 0x09
#define MAG_3110_OFF_Y_MSB 0x0B
#define MAG_3110_OFF_Z_MSB 0x0D

#define MAG_3110_CTRL_REG1 0x10
#define MAG_3110_CTRL_REG2 0x11

// don't subtract user offsets
#define MAG_3110_RAW 0x20

// Fields in registers
// CTRL_REG1: dr2,dr1,dr0  os1,os0  fr tm ac

// Sampling rate
#define MAG_3110_SAMPLE80 0
#define MAG_3110_SAMPLE1_25 0xC0
#define MAG_3110_SAMPLE0_08 0xE0

// How many samples to average (lowers data rate)
#define MAG_3110_OVERSAMPLE1 0
#define MAG_3110_OVERSAMPLE2 0x08
#define MAG_3110_OVERSAMPLE3 0x10
#define MAG_3110_OVERSAMPLE4 0x18

// put in active mode
#define MAG_3110_ACTIVE 0x01

// CTRL_REG2: AUTO_MRST_EN  _ RAW MAG_RST _ _ _ _ _
// reset sensor after each reading
#define MAG_3110_AUTO_MRST_EN 0x80

// DR_STATUS Register ZYXOW ZOW YOW XOW ZYXDR ZDR YDR XDR
#define MAG_3110_ZYXDR	0x08

#define MAG_CALIB_NUM_LIMIT 2000

#define degrees_per_radian  (180./3.14159265358979)

typedef struct{
  int16_t mag_low_x;
  int16_t  mag_high_x;
  int16_t  mag_low_y;
  int16_t  mag_high_y;
  int16_t  mag_low_z;
  int16_t  mag_high_z;
  int16_t mag_x_offset;
  int16_t mag_y_offset;
  int16_t mag_z_offset;
  float mag_x_scale;
  float mag_y_scale;
  float mag_z_scale;
}mag_values_t;

//inline uint8_t mag_data_ready(void);
void mag_read_xyz(int16_t *x, int16_t *y, int16_t *z);
void mag_set_offsets(void);
void mag_setup(void);
void start_mag_hardIron_calibrate(void);
void stop_mag_hardIron_calibrate(void);
