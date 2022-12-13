/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "util.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PID_TypeDef hPID;
double PIDAngle, PIDOut, PIDSetpoint;

uint16_t vaccumSensorValue;

AS5048A_TypeDef hASA;

Motor_TypeDef hMOT;

STEPPER_TypeDef hSTEP;

CONFIG_TypeDef config;

uint8_t CDC_TX_buffer[APP_TX_DATA_SIZE];
uint8_t CDC_RX_buffer[APP_RX_DATA_SIZE];
uint32_t Len;
uint8_t cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &vaccumSensorValue, sizeof(vaccumSensorValue));

	AS5048A_Init(&hASA, false);
	AS5048A_ConnectToSPI(&hASA, &hspi1);

	Motor_Init(&hMOT, &htim2);
	Motor_Enable(&hMOT);

	PID_Init(&hPID);
	PID(&hPID, &PIDAngle, &PIDOut, &PIDSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&hPID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&hPID, 200);
	PID_SetOutputLimits(&hPID, -config.ARM_MAX_FORCE, config.ARM_MAX_FORCE);

	PIDAngle = (double) AS5048A_getRotation(&hASA);
#ifdef DEBUG
	sprintf((char*)CDC_TX_buffer, "%f", PIDAngle);
	UTIL_send_CDC((char*)CDC_TX_buffer);
#endif


	HAL_TIM_Base_Start_IT(&htim3);

	Stepper_Init(&hSTEP, 13, 14, true);
	Stepper_setEnablePin(&hSTEP, 16);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//UTIL_send_CDC("WAITING_FOR_COMMAND");
		cnt = VCP_retrieveInputData(CDC_RX_buffer, &Len);
		if (Len > 0) {
			Len = 0;

			switch (UTIL_get_action_from_cdc(CDC_RX_buffer)) {

			case SET_LABEL:
				Stepper_move(&hSTEP, mm2steps(-3)); // TODO: Check if endstop triggered

				HAL_GPIO_WritePin(Probe_Deploy_GPIO_Port, Probe_Deploy_Pin, GPIO_PIN_SET); // Deploy probe
				HAL_Delay(config.PROBE_DEPLOY_WAIT);

				// TODO: Stepper home to probe, if not CDC send "ERROR_BOARD_CARRAIGE_EMPTY"

				if(Stepper_goToEndstop(&hSTEP, Probe_GPIO_Port, Probe_Pin, mm2steps(config.MAX_DISTANCE)) != true) {
					UTIL_send_CDC("ERROR_BOARD_CARRIAGE_EMPTY");
					break;
				}

				// Retract probe
				HAL_GPIO_WritePin(Probe_Deploy_GPIO_Port, Probe_Deploy_Pin, GPIO_PIN_RESET);
				HAL_Delay(config.PROBE_DEPLOY_WAIT);

				// Clamp Small deploy
				HAL_GPIO_WritePin(Clamp_Small_GPIO_Port, Clamp_Small_Pin, GPIO_PIN_SET);
				HAL_Delay(config.CLAMP_SMALL_DEPLOY_WAIT);

				// Clamp Big deploy
				HAL_GPIO_WritePin(Clamp_Big_GPIO_Port, Clamp_Big_Pin, GPIO_PIN_SET);
				HAL_Delay(config.CLAMP_BIG_DEPLOY_WAIT);

				// Retract clamps
				HAL_GPIO_WritePin(Clamp_Small_GPIO_Port, Clamp_Small_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Clamp_Big_GPIO_Port, Clamp_Big_Pin, GPIO_PIN_RESET);
				HAL_Delay(config.CLAMP_BIG_DEPLOY_WAIT);

				UTIL_send_CDC("OK");
				break;
			case NEXT_LABEL: {
				bool isLabelPicked = false;
				uint8_t attempts = 3;

				Motor_Enable(&hMOT);

				while(!isLabelPicked && attempts > 0) {
					PIDSetpoint = (double) config.ARM_PICK_ANGLE;
					UTIL_wait_PID_stable(&PIDAngle, &PIDSetpoint, &config.ARM_PID_POSITION_ACCURACY, &config.ARM_PID_POSITION_TIME);

					HAL_GPIO_WritePin(Vaccum_Generator_GPIO_Port, Vaccum_Generator_Pin, GPIO_PIN_SET);

					PIDSetpoint = (double) config.ARM_MEASURE_ANGLE;
					UTIL_wait_PID_stable(&PIDAngle, &PIDSetpoint, &config.ARM_PID_POSITION_ACCURACY, &config.ARM_PID_POSITION_TIME);

					if(vaccumSensorValue > config.VACCUM_MIN_VALUE) {
						isLabelPicked = true; break;
					}

					attempts--;
				}

				if (attempts == 0) {
					UTIL_send_CDC("ERROR_UNABLE_TO_PICK_LABEL");
					break;
				}
				PIDSetpoint = (double) config.ARM_PLACE_ANGLE;
				UTIL_wait_PID_stable(&PIDAngle, &PIDSetpoint, &config.ARM_PID_POSITION_ACCURACY, &config.ARM_PID_POSITION_TIME);

				HAL_GPIO_WritePin(Vaccum_Generator_GPIO_Port, Vaccum_Generator_Pin, GPIO_PIN_RESET);

				HAL_Delay(200);

				Motor_Disable(&hMOT);
				break;
			}

			case REFILL:

				break;
			case GET_CONFIG:

				break;
			case CALIBRATE_ARM:

				break;
			case SET_CONFIG:

				break;
			case ERROR_UNKNOW_COMMAND:
				memset(CDC_TX_buffer, '\0', 64);
				memcpy("UNKNOW COMMAND.", CDC_TX_buffer, 15);
				CDC_Transmit_FS(CDC_TX_buffer, sizeof(CDC_TX_buffer));
				break;
			default:
				break;
			}
		}


	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == TIM3) {
	  if(Motor_isRunning(&hMOT)) {
		  PIDAngle = (double) AS5048A_getRotation(&hASA);
		  PID_Compute(&hPID);
		  Motor_Turn(&hMOT, PIDOut);
	  }
  }



  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();



	while (1) {
		     HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
		     HAL_Delay(100);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
