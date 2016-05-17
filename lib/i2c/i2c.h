//      This library provides the high-level functions needed to use the I2C
//	serial interface supported by the hardware of several AVR processors.
#ifndef I2C_HEADER
#define I2C_HEADER

#include <Arduino.h>

// TWSR values (not bits)
// (taken from avr-libc twi.h - thank you Marek Michalkiewicz)
// Master
#define TW_START					0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
// Slave Transmitter
#define TW_ST_SLA_ACK				0xA8
#define TW_ST_ARB_LOST_SLA_ACK		0xB0
#define TW_ST_DATA_ACK				0xB8
#define TW_ST_DATA_NACK				0xC0
#define TW_ST_LAST_DATA				0xC8
// Slave Receiver
#define TW_SR_SLA_ACK				0x60
#define TW_SR_ARB_LOST_SLA_ACK		0x68
#define TW_SR_GCALL_ACK				0x70
#define TW_SR_ARB_LOST_GCALL_ACK	0x78
#define TW_SR_DATA_ACK				0x80
#define TW_SR_DATA_NACK				0x88
#define TW_SR_GCALL_DATA_ACK		0x90
#define TW_SR_GCALL_DATA_NACK		0x98
#define TW_SR_STOP					0xA0
// Misc
#define TW_NO_INFO					0xF8
#define TW_BUS_ERROR				0x00

// defines and constants
#define TWCR_CMD_MASK		0x0F
#define TWSR_STATUS_MASK	0xF8

// return values
#define I2C_OK				0x00
#define I2C_ERROR_NODEV		0x01

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))
#define BV(bit)			(1<<(bit))


// functions

//! Initialize I2C (TWI) interface
//	Must be done before anything else is called in this package
void i2cInit(void);

//! Set the I2C transaction bitrate (in KHz)
void i2cSetBitrate(unsigned short bitrateKHz);

// High-level interactions:
// Read one register 
uint8_t i2cReadRegister(uint8_t i2c_7bit_address, uint8_t address);
// Read num_to_read registers starting at address
void i2cReadRegisters(uint8_t i2c_7bit_address, uint8_t address, uint8_t num_to_read, uint8_t * dest);

// Write one register
void i2cWriteRegister(uint8_t i2c_7bit_address, uint8_t address, uint8_t data);
// Write num_to_write registers, starting at address
void i2cWriteRegisters(uint8_t i2c_7bit_address, uint8_t address, uint8_t num_to_write, const uint8_t *data);




// Low-level I2C transaction commands 
//! Send an I2C start condition in Master mode
void i2cSendStart(void);

//! Send an I2C stop condition in Master mode
void i2cSendStop(void);

//! Wait for current I2C operation to complete (not used for STOP)
// Return 0 if times out rather than completing.
//	Already included in operations, so not needed externally.
boolean i2cWaitForComplete(uint16_t num_tries=8000);

//! Send an (address|R/W) combination or a data byte over I2C
//	Deprecated: use i2cSendWriteAddress, i2cSendReadAddress, i2cSendData instead
void i2cSendByte(uint8_t data);

void i2cSendWriteAddress(uint8_t  i2c_7bit_address);
void i2cSendReadAddress(uint8_t  i2c_7bit_address);
void i2cSendData(uint8_t data);	// used for both data and register addresses

//! Receive a data byte over I2C  
// ackFlag = 1 if recevied data should be ACK'ed
// ackFlag = 0 if recevied data should be NACK'ed
void i2cReceiveByte(uint8_t ackFlag=1);
//! Pick up the data that was received with i2cReceiveByte()
uint8_t i2cGetReceivedByte(void);
//! Get current I2c bus status from TWSR
uint8_t i2cGetStatus(void);

// high-level I2C transaction commands

//! send I2C data to a device on the bus (non-interrupt based)
uint8_t i2cMasterSendNI(uint8_t deviceAddr, uint8_t length, uint8_t* data);
//! receive I2C data from a device on the bus (non-interrupt based)
uint8_t i2cMasterReceiveNI(uint8_t deviceAddr, uint8_t length, uint8_t *data);

#endif
