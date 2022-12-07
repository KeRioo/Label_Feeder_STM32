/*
 * AS5048A.cpp
 *
 *  Created on: 31 sie 2022
 *      Author: mprzybyl
 */

#include "AS5048A.h"


/**
 * Constructor
 */
void AS5048A_Init(AS5048A_TypeDef* hASA, bool debug) {
	hASA->errorFlag 	= false;
	hASA->ocfFlag 		= false;
	hASA->position 		= 0;
	hASA->debug 		= debug;
	hASA->spi_timeout	= UINT16_MAX;

}

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A_ConnectToSPI(AS5048A_TypeDef* hASA, SPI_HandleTypeDef* hspi)
{
	hASA->hspi = hspi;
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer
 */
uint8_t _AS5048A_spiCalcEvenParity(uint16_t value)
{
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A_getRotation(AS5048A_TypeDef* hASA)
{
	uint16_t data;
	int16_t rotation;

	data = AS5048A_getRawRotation(hASA);
	rotation = (int16_t)(data) - (int16_t)(hASA->position);
	if (rotation > AS5048A_MAX_VALUE)
		rotation = -((0x3FFF) - rotation); //more than -180

	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A_getRawRotation(AS5048A_TypeDef* hASA)
{
	return _AS5048A_read(hASA, AS5048A_ANGLE);
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double} between 0 and 360
  */

double AS5048A_getRotationInDegrees(AS5048A_TypeDef* hASA)
{
	int16_t rotation = AS5048A_getRotation(hASA);
	double degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / (AS5048A_MAX_VALUE * 2.0);
	return degrees;
}

/**
  * Get the rotation of the sensor relative to the zero position in radians.
  *
  * @return {double} between 0 and 2 * PI
  */

double AS5048A_getRotationInRadians(AS5048A_TypeDef* hASA)
{
	int16_t rotation = AS5048A_getRotation(hASA);
	double radians = M_PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
	return radians;
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A_getState(AS5048A_TypeDef* hASA)
{
	return _AS5048A_read(hASA, AS5048A_DIAG_AGC);
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain(AS5048A_TypeDef* hASA)
{
	uint16_t data = AS5048A_getState(hASA);
	return (uint8_t)(data & AS5048A_AGC_FLAG);
}

/**
 * Get diagnostic
 */
AS5048A_EnumDIAG AS5048A_getDiagnostic(AS5048A_TypeDef* hASA)
{
	uint16_t data = AS5048A_getState(hASA);
	if (data & AS5048A_DIAG_COMP_HIGH)
	{
		return AS5048A_COMP_HIGH;
	}
	if (data & AS5048A_DIAG_COMP_LOW)
	{
		return AS5048A_COMP_LOW;
	}
	if (data & AS5048A_DIAG_COF)
	{
		return AS5048A_COF;
	}
	if (data & AS5048A_DIAG_OCF && hASA->ocfFlag == false)
	{
		hASA->ocfFlag = true;
		return AS5048A_OCF;
	}
	return 0;
}

/*
 * Get and clear the error register by reading it
 */
AS5048A_EnumERROR AS5048A_getErrors(AS5048A_TypeDef* hASA)
{
	uint16_t error = _AS5048A_read(hASA, AS5048A_CLEAR_ERROR_FLAG);
	if (error & AS5048A_ERROR_PARITY_FLAG)
	{
		return AS5048A_PARITY_FLAG;
	}
	if (error & AS5048A_ERROR_COMMAND_INVALID_FLAG)
	{
		return AS5048A_COMMAND_INVALID_FLAG;
	}
	if (error & AS5048A_ERROR_FRAMING_FLAG)
	{
		return AS5048A_FRAMING_FLAG;
	}
	return 0;
}

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(AS5048A_TypeDef* hASA, uint16_t position)
{
	hASA->position = position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition(AS5048A_TypeDef* hASA)
{
	return hASA->position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A_error(AS5048A_TypeDef* hASA)
{
	return hASA->errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t _AS5048A_read(AS5048A_TypeDef* hASA, uint16_t registerAddress)
{
	uint16_t command = 0x4000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= (uint16_t)(_AS5048A_spiCalcEvenParity(command) << 0xF);

	if (hASA->debug)
	{
//		Serial.print("Read (0x");
//		Serial.print(registerAddress, HEX);
//		Serial.print(") with command: 0b");
//		Serial.println(command, BIN);
	}

	uint8_t buffer[2];
	buffer[0] = command >> 8;
	buffer[1] = command;

	//Send the command
	HAL_SPI_Transmit(hASA->hspi, (uint8_t*) buffer, 16, hASA->spi_timeout);

	buffer[0] = 0x0;
	buffer[1] = 0x0;

	//Now read the response
	HAL_SPI_TransmitReceive(hASA->hspi, 0x00, (uint8_t*) buffer, 16, hASA->spi_timeout);

	uint16_t response = ((uint16_t)buffer[0] << 8 ) | ((uint16_t) buffer[1]);

	if (hASA->debug)
	{
//		Serial.print("Read returned: ");
//		Serial.println(response, BIN);
	}

	//Check if the error bit is set
	if (response & 0x4000)
	{
		if (hASA->debug)
		{
//			Serial.println("Setting error bit");
		}
		hASA->errorFlag = true;
	}
	else
	{
		hASA->errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t _AS5048A_write(AS5048A_TypeDef* hASA, uint16_t registerAddress, uint16_t data)
{

	uint16_t command = 0x0000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= (uint16_t)(_AS5048A_spiCalcEvenParity(command) << 0xF);

	if (hASA->debug)
	{
//		Serial.print("Write (0x");
//		Serial.print(registerAddress, HEX);
//		Serial.print(") with command: 0b");
//		Serial.println(command, BIN);
	}

	//Start the write command with the target address
	uint8_t buffer[2];
		buffer[0] = command >> 8;
		buffer[1] = command;

	//Send the command
	HAL_SPI_Transmit(hASA->hspi, (uint8_t*) buffer, 16, hASA->spi_timeout);


	uint16_t dataToSend = 0x0000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= (uint16_t)(_AS5048A_spiCalcEvenParity(dataToSend) << 0xF);

	if (hASA->debug)
	{
//		Serial.print("Sending data to write: ");
//		Serial.println(dataToSend, BIN);
	}

	buffer[0] = dataToSend >> 8;
	buffer[1] = dataToSend;

	//Now read the response
	HAL_SPI_Transmit(hASA->hspi, (uint8_t*) buffer, 16, hASA->spi_timeout);

	buffer[0] = 0x0;
	buffer[1] = 0x0;

	HAL_SPI_TransmitReceive(hASA->hspi, 0x00, (uint8_t*) buffer, 16, hASA->spi_timeout);

	uint16_t response = ((uint16_t)buffer[0] << 8 ) | ((uint16_t) buffer[1]);

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}
