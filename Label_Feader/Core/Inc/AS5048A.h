/*
 * AS5048A.h
 *
 *  Created on: 31 sie 2022
 *      Author: mprzybyl
 */

#ifndef AS5048A_HPP_
#define AS5048A_HPP_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <Math.h>

static const uint16_t AS5048A_CLEAR_ERROR_FLAG = 0x0001;
static const uint16_t AS5048A_PROGRAMMING_CONTROL = 0x0003;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH = 0x0016;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW = 0x0017;
static const uint16_t AS5048A_DIAG_AGC = 0x3FFD;
static const uint16_t AS5048A_MAGNITUDE = 0x3FFE;
static const uint16_t AS5048A_ANGLE = 0x3FFF;

static const uint8_t AS5048A_AGC_FLAG = 0xFF;
static const uint8_t AS5048A_ERROR_PARITY_FLAG = 0x04;
static const uint8_t AS5048A_ERROR_COMMAND_INVALID_FLAG = 0x02;
static const uint8_t AS5048A_ERROR_FRAMING_FLAG = 0x01;

static const uint16_t AS5048A_DIAG_COMP_HIGH = 0x2000;
static const uint16_t AS5048A_DIAG_COMP_LOW = 0x1000;
static const uint16_t AS5048A_DIAG_COF = 0x0800;
static const uint16_t AS5048A_DIAG_OCF = 0x0400;

static const double AS5048A_MAX_VALUE = 8191.0;



typedef struct {

	SPI_HandleTypeDef *hspi;
	bool errorFlag;
	bool ocfFlag;
	uint16_t position;
	bool debug;
	uint8_t esp32_delay;
	uint16_t spi_timeout;

} AS5048A_TypeDef;

typedef enum {
	AS5048A_COMP_HIGH,
	AS5048A_COMP_LOW,
	AS5048A_COF,
	AS5048A_OCF
} AS5048A_EnumDIAG;

typedef enum {
	AS5048A_PARITY_FLAG,
	AS5048A_COMMAND_INVALID_FLAG,
	AS5048A_FRAMING_FLAG
} AS5048A_EnumERROR;


uint8_t _AS5048A_spiCalcEvenParity(uint16_t value);

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit uint16_t
 * Returns the value of the register
 */
uint16_t _AS5048A_read(AS5048A_TypeDef* hASA, uint16_t registerAddress);

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t _AS5048A_write(AS5048A_TypeDef* hASA, uint16_t registerAddress, uint16_t data);

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A_getRotation(AS5048A_TypeDef* hASA);

/*
 * Check if an error has been encountered.
 */
bool AS5048A_error(AS5048A_TypeDef* hASA);



/** Constructor**/
void AS5048A_Init(AS5048A_TypeDef* hASA, bool debug);

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A_ConnectToSPI(AS5048A_TypeDef* hASA, SPI_HandleTypeDef* hspi);

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A_getRawRotation(AS5048A_TypeDef* hASA);

/**
 * Get the rotation of the sensor relative to the zero position in degrees.
 *
 * @return {double} between 0 and 360
 */
double AS5048A_getRotationInDegrees(AS5048A_TypeDef* hASA);

/**
 * Get the rotation of the sensor relative to the zero position in radians.
 *
 * @return {double} between 0 and 2 * PI
 */
double AS5048A_getRotationInRadians(AS5048A_TypeDef* hASA);

/**
 * returns the value of the state register
 * @return 16 bit uint16_t containing flags
 */
uint16_t AS5048A_getState(AS5048A_TypeDef* hASA);

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain(AS5048A_TypeDef* hASA);

/*
 * Get and clear the error register by reading it
 */
AS5048A_EnumERROR AS5048A_getErrors(AS5048A_TypeDef* hASA);

/**
 * Get diagnostic
 */
AS5048A_EnumDIAG AS5048A_getDiagnostic(AS5048A_TypeDef* hASA);

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(AS5048A_TypeDef* hASA, uint16_t position);

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition(AS5048A_TypeDef* hASA);

#endif
