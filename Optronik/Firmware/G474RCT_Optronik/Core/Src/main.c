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

#include "stm_hal_serial.h"
#include "libVisca.h"
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
uint8_t rxDBuffer[RING_BUFFER_RX_SIZE];
uint8_t txDBuffer[RING_BUFFER_TX_SIZE];
TSerial debug;

uint8_t rxSonyBuffer[RING_BUFFER_RX_SIZE];
uint8_t txSonyBuffer[RING_BUFFER_TX_SIZE];
TSerial sony;

VISCAInterface_t sonyIface;
VISCACamera_t sonyCamera;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void usart2ChangeBaud(uint32_t baud);
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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Init */
	serial_init(&debug, (char*) &rxDBuffer, sizeof(rxDBuffer), (char*) &txDBuffer,
			sizeof(txDBuffer), &huart1);

	bufLen = sprintf(buf, "Optronik Firmware!\r\n");
	serial_write_str(&debug, buf, bufLen);

	bufLen = sprintf(buf, "BR= %ld\r\n", huart2.Init.BaudRate);
	serial_write_str(&debug, buf, bufLen);

	sonyIface.port_fd = &sony;
	sonyIface.broadcast = 0;
	sonyCamera.address = 1;
	serial_init(&sony, (char*) &rxSonyBuffer, sizeof(rxSonyBuffer), (char*) &txSonyBuffer,
			sizeof(txSonyBuffer), &huart2);

//	HAL_Delay(3000);
//	if (VISCA_set_zoom_value(&sonyIface, &sonyCamera, 0x2000) == VISCA_SUCCESS) {
//		bufLen = sprintf(buf, "sony success!\r\n");
//	}
//	else
//		bufLen = sprintf(buf, "sony failed!\r\n");
//	serial_write_str(&debug, buf, bufLen);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t ledTimer = 0;
	while (1) {
		char c;
		uint8_t sonyCommand[32];
		uint32_t u32;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/

		if (serial_available(&sony) > 0) {
			c = serial_read(&sony);
			bufLen = sprintf(buf, "0x%02X ", c);
			serial_write_str(&debug, buf, bufLen);
			if (c == 0xFF)
				serial_write_str(&debug, "\r\n", 2);
		}

		if (serial_available(&debug) > 0) {
			c = serial_read(&debug);
			if (c == 'R') {
//				bufLen = sprintf(buf, "Camera Reset= %d\r\n",
//						HAL_GPIO_ReadPin(SONY_RESET_GPIO_Port, SONY_RESET_Pin));
//				serial_write_str(&debug, buf, bufLen);
//
//				HAL_GPIO_WritePin(SONY_RESET_GPIO_Port, SONY_RESET_Pin, GPIO_PIN_RESET);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(SONY_RESET_GPIO_Port, SONY_RESET_Pin, GPIO_PIN_SET);

			}
			else if (c == 't') {
				bufLen = sprintf(buf, "get camera version\r\n");
				serial_write_str(&debug, buf, bufLen);
				/* version inquiry */
				memset(sonyCommand, 0, 32);
				sonyCommand[0] = 0x81;
				sonyCommand[1] = 0x09;
				sonyCommand[2] = 0x00;
				sonyCommand[3] = 0x02;
				sonyCommand[4] = 0xFF;
				serial_write_str(&sony, sonyCommand, 5);
			}
			else if (c == 'P') {
				bufLen = sprintf(buf, "Power On\r\n");
				serial_write_str(&debug, buf, bufLen);
				/* version inquiry */
				memset(sonyCommand, 0, 32);
				sonyCommand[0] = 0x81;
				sonyCommand[1] = 0x01;
				sonyCommand[2] = 0x04;
				sonyCommand[3] = 0x00;
				sonyCommand[4] = 0x02;
				sonyCommand[5] = 0xFF;
				serial_write_str(&sony, sonyCommand, 6);
			}
//			else if (c == 'z') {
//
//				u32 = 0x2000;
//
//				bufLen = sprintf(buf, "zoom= 0x%lX\t%d\r\n", u32,
//						VISCA_set_zoom_value(&sonyIface, &sonyCamera, u32));
//				serial_write_str(&debug, buf, bufLen);
//			}
//			else if (c == 'A') {
//				bufLen = sprintf(buf, "set address to default!\r\n");
//				serial_write_str(&debug, buf, bufLen);
//				memset(sonyCommand, 0, 32);
//				sonyCommand[0] = 0x88;
//				sonyCommand[1] = 0x30;
//				sonyCommand[2] = 0x01;
//				sonyCommand[3] = 0xFF;
//				serial_write_str(&sony, (const char*) sonyCommand, 4);
//			}
//			else if (c == 'F') {
//				bufLen = sprintf(buf, "IF_Clear broadcast\r\n");
//				serial_write_str(&debug, buf, bufLen);
//				memset(sonyCommand, 0, 32);
//				sonyCommand[0] = 0x88;
//				sonyCommand[1] = 0x01;
//				sonyCommand[2] = 0x00;
//				sonyCommand[3] = 0x01;
//				sonyCommand[4] = 0xFF;
//				serial_write_str(&sony, (const char*) sonyCommand, 5);
//			}
//			else if (c == '9') {
//				usart2ChangeBaud(9600);
//				serial_init(&sony, (char*) &rxSonyBuffer, sizeof(rxSonyBuffer),
//						(char*) &txSonyBuffer, sizeof(txSonyBuffer), &huart2);
//				serial_read_flush(&sony);
//			}
//			else if (c == '1') {
//				usart2ChangeBaud(19200);
//				serial_init(&sony, (char*) &rxSonyBuffer, sizeof(rxSonyBuffer),
//						(char*) &txSonyBuffer, sizeof(txSonyBuffer), &huart2);
//				serial_read_flush(&sony);
//			}
//			else if (c == '3') {
//				usart2ChangeBaud(38400);
//				serial_init(&sony, (char*) &rxSonyBuffer, sizeof(rxSonyBuffer),
//						(char*) &txSonyBuffer, sizeof(txSonyBuffer), &huart2);
//				serial_read_flush(&sony);
//			}
//			else if (c == '\r')
//				serial_write(&debug, '\r');
//			else if (c == '\n')
//				serial_write(&debug, '\n');

		}

		if (HAL_GetTick() >= ledTimer) {
			ledTimer = HAL_GetTick() + 100;

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//			bufLen = sprintf(buf, "led= %d;sony_rst= %d;B0= %d\r\n",
//					HAL_GPIO_ReadPin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin),
//					HAL_GPIO_ReadPin(SONY_RESET_GPIO_Port, SONY_RESET_Pin),
//					HAL_GPIO_ReadPin(Boot0_GPIO_Port, Boot0_Pin));
			bufLen = sprintf(buf, "led= %d;B0= %d\r\n",
					HAL_GPIO_ReadPin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin),
					HAL_GPIO_ReadPin(Boot0_GPIO_Port, Boot0_Pin));
			serial_write_str(&debug, buf, bufLen);
		}

		/* TODO END LOOP*/
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* TODO USER FUNCTION*/
void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&sony);
}

static void usart2ChangeBaud(uint32_t baud)
{
	uint16_t bufLen = 0;
	char buf[RING_BUFFER_TX_SIZE];

	huart2.Init.BaudRate = baud;
	if (HAL_UART_Init(&huart2) == HAL_OK) {
		bufLen = sprintf(buf, "baud= %ld\r\n", huart2.Init.BaudRate);
		serial_write_str(&debug, buf, bufLen);
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
