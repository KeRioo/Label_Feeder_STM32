/*
 * util.cpp
 *
 *  Created on: Sep 6, 2022
 *      Author: mprzybyl
 */

#include "util.h"

#include "eeprom.h"

#include "usbd_cdc_if.h"

ACTION_ENUM_TypeDef UTIL_get_action_from_cdc(const uint8_t *rx_buffer) {
	int j;
	for (j = 0; j < sizeof(conversion) / sizeof(conversion[0]); ++j) {
		if (!strcmp((char*) rx_buffer, conversion[j].str)) {
			return conversion[j].val;
		}
	}
	return ERROR_UNKNOW_COMMAND;
}

void UTIL_read_EEPROM(uint16_t* virtVar, CONFIG_TypeDef *config) {

	uint16_t readTab[NB_OF_VAR];

	for (uint16_t i = 0; i < NB_OF_VAR; i++)
	  {
		  if(EE_ReadVariable(virtVar[i],  &readTab[i]) != HAL_OK)
		  {
			  Error_Handler();
		  }
	  }

	config->PROBE_DEPLOY_WAIT 			= readTab[0];
	config->MAX_DISTANCE 				= readTab[1];
	config->CLAMP_SMALL_DEPLOY_WAIT		= readTab[2];
	config->CLAMP_BIG_DEPLOY_WAIT		= readTab[3];
	config->ARM_PICK_ANGLE				= readTab[4];
	config->ARM_MEASURE_ANGLE			= readTab[5];
	config->ARM_PLACE_ANGLE				= readTab[6];
	config->ARM_PID_POSITION_ACCURACY 	= readTab[7];
	config->ARM_PID_POSITION_TIME		= readTab[8];
	config->ARM_MAX_FORCE				= readTab[9];
	config->VACCUM_MIN_VALUE			= readTab[10];
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



