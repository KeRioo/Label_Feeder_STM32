/*
 * Motor.h
 *
 *  Created on: Aug 29, 2022
 *      Author: mprzybyl
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_


#include "stm32f4xx_hal.h"
#include "micros.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"


typedef struct {
	unsigned int _L_EN;
	unsigned int _R_EN;
	TIM_HandleTypeDef *_htim;
	bool is_running;
}Motor_TypeDef;


void Motor_Init(Motor_TypeDef* hMOT, TIM_HandleTypeDef* htim);

void Motor_Enable(Motor_TypeDef* hMOT);
void Motor_Disable(Motor_TypeDef* hMOT);

void Motor_Turn(Motor_TypeDef* hMOT, int8_t);
void Motor_Stop(Motor_TypeDef* hMOT);

bool Motor_isRunning(Motor_TypeDef* hMOT);



#endif /* SRC_MOTOR_H_ */
