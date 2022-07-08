/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stm_hal_serial.h"
#include "button_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_ENABLE				0

#define READ_GPIO_TIMEOUT			button_update_timeout_ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sModeGpioInput_t input;
sModeGpioOutput_t output;

uint8_t rxPBuffer[RING_BUFFER_RX_SIZE];
uint8_t txPBuffer[RING_BUFFER_TX_SIZE];
TSerial panel;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void stateInit();
static void stateRead();
static uint32_t parsingCommand(const char *str);
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
	char inCommand[RING_BUFFER_RX_SIZE / 2];
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	serial_init(&panel, (char*) &rxPBuffer, sizeof(rxPBuffer), (char*) &txPBuffer,
			sizeof(txPBuffer), &huart1);

#if DEBUG_ENABLE==1
	bufLen = sprintf(buf, "Button - RCWS firmware!\r\n");
	serial_write_str(&panel, buf, bufLen);

	char s[32];
	sprintf(s, "$PNL,F*");
	uint16_t ret = parsingCommand(s);
	bufLen = sprintf(buf, "ret= 0x%04X\r\n", ret);
	serial_write_str(&panel, buf, bufLen);

#endif	//if DEBUG_ENABLE==1

	stateInit();
//#if DEBUG_ENABLE==1
//	HAL_GPIO_WritePin(OUT_POWER_LED_GPIO_Port, OUT_POWER_LED_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(OUT_TRIG_LED_GPIO_Port, OUT_TRIG_LED_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(OUT_LRF_EN_GPIO_Port, OUT_LRF_EN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(OUT_CAM_SELECT_GPIO_Port, OUT_CAM_SELECT_Pin, GPIO_PIN_SET);
//#endif	//if DEBUG_ENABLE==1

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t readTimer = 0;
	while (1) {
		uint8_t inCommandCompleted = 0;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (HAL_GetTick() >= readTimer) {
			readTimer = HAL_GetTick() + READ_GPIO_TIMEOUT;

			stateRead();

#if DEBUG_ENABLE==1
			bufLen = sprintf(buf, "input= 0x%04X\r\n", *(uint16_t*) &input);
			serial_write_str(&panel, buf, bufLen);
//			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
#endif	//if DEBUG_ENABLE==1

			bufLen = sprintf(buf, "$BTN,%04X*\r\n", *(uint16_t*) &input);
			serial_write_str(&panel, buf, bufLen);
		}

		if (serial_available(&panel) > 0) {
			char c[2] = { 0, 0 };
			c[0] = serial_read(&panel);

			if (c[0] == '$')
				memset(inCommand, 0, RING_BUFFER_RX_SIZE / 2);
			else if (c[0] == '*')
				inCommandCompleted = 1;
			strcat(inCommand, c);
		}

		if (inCommandCompleted != 0) {
			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
#if DEBUG_ENABLE==1
			bufLen = sprintf(buf, "[%ld]cmd: %s\r\n", HAL_GetTick(), inCommand);
			serial_write_str(&panel, buf, bufLen);
//			serial_write_str(&panel, inCommand, strlen(inCommand));

			*(uint8_t*) &output = parsingCommand(inCommand);

			bufLen = sprintf(buf, "out: %X\r\n", *(uint8_t*) &output);
			serial_write_str(&panel, buf, bufLen);
#else
			*(uint8_t*) &output = parsingCommand(inCommand);
#endif	//if DEBUG_ENABLE==1
			HAL_GPIO_WritePin(OUT_POWER_LED_GPIO_Port, OUT_POWER_LED_Pin, output.power_state);
			HAL_GPIO_WritePin(OUT_TRIG_LED_GPIO_Port, OUT_TRIG_LED_Pin, output.trig_enable);
			HAL_GPIO_WritePin(OUT_LRF_EN_GPIO_Port, OUT_LRF_EN_Pin, output.lrf_enable);
			HAL_GPIO_WritePin(OUT_CAM_SELECT_GPIO_Port, OUT_CAM_SELECT_Pin, output.cam_select);
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* TODO User Functions*/
void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&panel);
}

static void stateInit()
{
	*(uint16_t*) &input = 0;
	*(uint8_t*) &output = 0;
}

static void stateRead()
{
	if (HAL_GPIO_ReadPin(IN_TRIG_GPIO_Port, IN_TRIG_Pin) == GPIO_PIN_RESET)
		input.trig_enable = TRIG_ENABLE;
	else
		input.trig_enable = TRIG_DISABLE;

	if (HAL_GPIO_ReadPin(IN_FIRING0_GPIO_Port, IN_FIRING0_Pin) == GPIO_PIN_RESET)
		input.firing_mode = FIRING_MODE_1;
	else if (HAL_GPIO_ReadPin(IN_FIRING1_GPIO_Port, IN_FIRING1_Pin) == GPIO_PIN_RESET)
		input.firing_mode = FIRING_MODE_CONT;
	else
		input.firing_mode = FIRING_MODE_3;

	if (HAL_GPIO_ReadPin(IN_CAM_SELECT_GPIO_Port, IN_CAM_SELECT_Pin) == GPIO_PIN_RESET)
		input.cam_select = CAM_SELECT_THERMAL;
	else
		input.cam_select = CAM_SELECT_DAY;

	if (HAL_GPIO_ReadPin(IN_SPD0_GPIO_Port, IN_SPD0_Pin) == GPIO_PIN_RESET)
		input.spd_select = SPD_SELECT_MIN;
	else if (HAL_GPIO_ReadPin(IN_SPD1_GPIO_Port, IN_SPD1_Pin) == GPIO_PIN_RESET)
		input.spd_select = SPD_SELECT_MAX;
	else
		input.spd_select = SPD_SELECT_MID;

	if (HAL_GPIO_ReadPin(IN_FOCUS_IN_GPIO_Port, IN_FOCUS_IN_Pin) == GPIO_PIN_RESET)
		input.focus_mode = FOCUS_IN;
	else if (HAL_GPIO_ReadPin(IN_FOCUS_OUT_GPIO_Port, IN_FOCUS_OUT_Pin) == GPIO_PIN_RESET)
		input.focus_mode = FOCUS_OUT;
	else
		input.focus_mode = FOCUS_STOP;

	if (HAL_GPIO_ReadPin(IN_LRF_EN_GPIO_Port, IN_LRF_EN_Pin) == GPIO_PIN_RESET)
		input.lrf_enable = LRF_ENABLE;
	else
		input.lrf_enable = LRF_DISABLE;

	if (HAL_GPIO_ReadPin(IN_LRF_START_GPIO_Port, IN_LRF_START_Pin) == GPIO_PIN_RESET)
		input.lrf_start = LRF_ENABLE;
	else
		input.lrf_start = LRF_DISABLE;

	if (HAL_GPIO_ReadPin(IN_LRF_UP_GPIO_Port, IN_LRF_UP_Pin) == GPIO_PIN_RESET)
		input.lrf_man_mode = LRF_MAN_UP;
	else if (HAL_GPIO_ReadPin(IN_LRF_DOWN_GPIO_Port, IN_LRF_DOWN_Pin) == GPIO_PIN_RESET)
		input.lrf_man_mode = LRF_MAN_DOWN;
	else
		input.lrf_man_mode = LRF_MAN_NONE;

	if (HAL_GPIO_ReadPin(IN_TARGET_NEXT_GPIO_Port, IN_TARGET_NEXT_Pin) == GPIO_PIN_RESET)
		input.target_select = TARGET_NEXT;
	else if (HAL_GPIO_ReadPin(IN_TARGET_PREV_GPIO_Port, IN_TARGET_PREV_Pin) == GPIO_PIN_RESET)
		input.target_select = TARGET_PREV;
	else
		input.target_select = TARGET_NONE;
}

static uint32_t parsingCommand(const char *str)
{
	uint32_t len;
	uint8_t startHeader = 0, startContent = 0;
	char hBuf[8] = { 0 }, cBuf[16] = { 0 };
	char c[2] = { 0, 0 };
	uint32_t ret = 0;

	len = strlen(str);
	for ( int i = 0; i < len; i++ ) {
		c[0] = str[i];

		if (c[0] == '*')
			break;
		if (startHeader > 0) {
			if (startContent == 0) {
				if (c[0] == ',')
					startContent = 1;
				else
					strcat(hBuf, c);
			}
			else
				strcat(cBuf, c);
		}
		else {
			if (c[0] == '$')
				startHeader = 1;
		}
	}

	int cmp = strcmp("PNL", hBuf);
	if (cmp == 0)
		ret = strtol(cBuf, NULL, 16);

	return ret;
}

/* USER CODE END 4 */

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

