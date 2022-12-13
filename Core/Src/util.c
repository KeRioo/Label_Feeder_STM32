/*
 * util.cpp
 *
 *  Created on: Sep 6, 2022
 *      Author: mprzybyl
 */

#include "util.h"

#include "usbd_cdc_if.h"

uint32_t power_counter;


ACTION_ENUM_TypeDef UTIL_get_action_from_cdc(const uint8_t *rx_buffer) {
	int j;
	for (j = 0; j < sizeof(conversion) / sizeof(conversion[0]); ++j) {
		if (!strcmp((char*) rx_buffer, conversion[j].str)) {
			return conversion[j].val;
		}
	}
	return ERROR_UNKNOW_COMMAND;
}

void UTIL_update_power_counter(I2C_HandleTypeDef *hi2c, bool onlyRead){

	if (HAL_I2C_Mem_Read(hi2c, 0b1010000, 0, 4, (uint8_t *)&power_counter, sizeof(power_counter), 10) != HAL_OK) {
		Error_Handler();
	}

	if(onlyRead) return;

	UTIL_increment_power_counter(hi2c);
}

void UTIL_increment_power_counter(I2C_HandleTypeDef *hi2c) {

	power_counter++;
	if (HAL_I2C_Mem_Write(hi2c, 0b1010000, 0, 4, (uint8_t *)&power_counter, sizeof(power_counter), 10) != HAL_OK) {
		Error_Handler();
	}


}

void UTIL_read_EEPROM(I2C_HandleTypeDef *hi2c, CONFIG_TypeDef *config) {

	if (HAL_I2C_Mem_Read(hi2c, 0b1010000, 4, sizeof(struct CONFIG), (uint8_t *)&config, sizeof(struct CONFIG), 100) != HAL_OK) {
			  Error_Handler();
	}

}

void UTIL_send_CDC(char* message) {
	CDC_Transmit_FS((uint8_t*) message, sizeof(message));
}


void UTIL_wait_PID_stable(double* PIDAngle, double* PIDSetpoint, uint16_t *armPIDPositionTime, uint16_t *armPIDPositionAccuracy) {
	uint accuracyCounter = 0;
	while (accuracyCounter < *armPIDPositionTime) {
		fabs(&PIDAngle - &PIDSetpoint) > (double) *armPIDPositionAccuracy ? accuracyCounter = 0 : accuracyCounter++;
	};
}



