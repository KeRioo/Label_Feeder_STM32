/*
 * Motor.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: mprzybyl
 */

#include <Motor.h>


void Motor_Init(Motor_TypeDef* hMOT, TIM_HandleTypeDef* htim)
{
  hMOT->_htim = htim;
  hMOT->_L_EN = 7;
  hMOT->_R_EN = 6;
  hMOT->is_running = false;
}


void Motor_Turn(Motor_TypeDef* hMOT, int8_t pwm){
  if (!hMOT->is_running) {
	  Motor_Enable(hMOT);
  }

  switch (pwm) {
    case INT8_MIN ... -1:
      TIM2->CCR2 = 0;
      delay_us(100);
      TIM2->CCR3 = abs(pwm);
      break;

    case 0:
      TIM2->CCR2 = 0;
      TIM2->CCR3 = 0;
      break;

    case 1 ... INT8_MAX:
      TIM2->CCR3 = 0;
      delay_us(100);
      TIM2->CCR2 = abs(pwm);
      break;
  }
}

void Motor_Enable(Motor_TypeDef* hMOT) {
  if (!hMOT->is_running) {
	  hMOT->is_running = true;
      HAL_GPIO_WritePin(GPIOB, hMOT->_L_EN, GPIO_PIN_SET);
      if(hMOT->_R_EN != 0) HAL_GPIO_WritePin(GPIOB, hMOT->_R_EN, GPIO_PIN_SET);
      HAL_Delay(10);
      TIM2->CCR2 = 0;
      HAL_TIM_PWM_Start(hMOT->_htim, TIM_CHANNEL_2);
      TIM2->CCR3 = 0;
      HAL_TIM_PWM_Start(hMOT->_htim, TIM_CHANNEL_3);
  }
}

void Motor_Disable(Motor_TypeDef* hMOT){
  if (hMOT->is_running) {
	  hMOT->is_running = false;
      HAL_GPIO_WritePin(GPIOB, hMOT->_L_EN, GPIO_PIN_RESET);
      if(hMOT->_R_EN != 0) HAL_GPIO_WritePin(GPIOB, hMOT->_R_EN, GPIO_PIN_RESET);
  }
}

void Motor_Stop(Motor_TypeDef* hMOT) {
  if (hMOT->is_running) {
      TIM2->CCR2 = 0;
      TIM2->CCR2 = 0;
      Motor_Disable(hMOT);
  }
}

bool Motor_isRunning(Motor_TypeDef* hMOT) {
  return hMOT->is_running;
}


