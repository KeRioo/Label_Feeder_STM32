/*
 * util.h
 *
 *  Created on: Sep 6, 2022
 *      Author: mprzybyl
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include <string.h>
#include <i2c.h>

typedef struct CONFIG {
	uint16_t PROBE_DEPLOY_WAIT;
	uint16_t MAX_DISTANCE;
	uint16_t CLAMP_SMALL_DEPLOY_WAIT;
	uint16_t CLAMP_BIG_DEPLOY_WAIT;
	int16_t  ARM_PICK_ANGLE;
	int16_t  ARM_PLACE_ANGLE;
	uint16_t ARM_PID_POSITION_ACCURACY;
	uint16_t ARM_PID_POSITION_TIME;
	int16_t  ARM_MEASURE_ANGLE;
	uint16_t ARM_MAX_FORCE;
	uint16_t VACCUM_MIN_VALUE;


} CONFIG_TypeDef;

#define CONFIG_NB_VAR (sizeof(struct CONFIG) / sizeof(uint16_t))


typedef enum ACTION_ENUM {
	SET_LABEL,
	NEXT_LABEL,
	REFILL,
	GET_CONFIG,
	SET_CONFIG,
	CALIBRATE_ARM,
	ERROR_UNKNOW_COMMAND
} ACTION_ENUM_TypeDef;

const static struct  {
	ACTION_ENUM_TypeDef val;
	const char *str;
} conversion[] =
{
		{ SET_LABEL, "SET_LABEL" },
		{ NEXT_LABEL, "NEXT_LABEL" },
		{ REFILL, "REFILL" },
		{ GET_CONFIG, "GET_CONFIG" },
		{ SET_CONFIG,"SET_CONFIG" },
		{ CALIBRATE_ARM, "CALIBRATE_ARM" }
 };


void UTIL_update_power_counter(I2C_HandleTypeDef *hi2c, bool onlyRead);
void UTIL_increment_power_counter(I2C_HandleTypeDef *hi2c);

ACTION_ENUM_TypeDef UTIL_get_action_from_cdc(const uint8_t *rx_buffer);

void UTIL_read_EEPROM( I2C_HandleTypeDef *hi2c, CONFIG_TypeDef *config);

void UTIL_send_CDC(char* message);

void UTIL_wait_PID_stable(double* PIDAngle, double* PIDSetpoint, uint16_t* armPIDPositionTime, uint16_t* armPIDPositionAccuracy);

#endif /* UTIL_H_ */
