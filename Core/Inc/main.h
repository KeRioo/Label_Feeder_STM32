/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "AS5048A.h"
#include "Motor.h"
#include "Stepper.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_LED_Pin GPIO_PIN_13
#define BOARD_LED_GPIO_Port GPIOC
#define L_PWM_Pin GPIO_PIN_1
#define L_PWM_GPIO_Port GPIOA
#define R_PWM_Pin GPIO_PIN_2
#define R_PWM_GPIO_Port GPIOA
#define Pressure_Sensor_Pin GPIO_PIN_3
#define Pressure_Sensor_GPIO_Port GPIOA
#define Probe_Pin GPIO_PIN_0
#define Probe_GPIO_Port GPIOB
#define Probe_Deploy_Pin GPIO_PIN_1
#define Probe_Deploy_GPIO_Port GPIOB
#define ENA_Pin GPIO_PIN_12
#define ENA_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_13
#define DIR_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_14
#define STEP_GPIO_Port GPIOB
#define Vaccum_Generator_Pin GPIO_PIN_8
#define Vaccum_Generator_GPIO_Port GPIOA
#define Clamp_Big_Pin GPIO_PIN_9
#define Clamp_Big_GPIO_Port GPIOA
#define Clamp_Small_Pin GPIO_PIN_10
#define Clamp_Small_GPIO_Port GPIOA
#define Endstop_R_Pin GPIO_PIN_4
#define Endstop_R_GPIO_Port GPIOB
#define Endstop_L_Pin GPIO_PIN_5
#define Endstop_L_GPIO_Port GPIOB
#define R_EN_Pin GPIO_PIN_6
#define R_EN_GPIO_Port GPIOB
#define L_EN_Pin GPIO_PIN_7
#define L_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
