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
#include "cmsis_os.h"
#include "fdcan.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "app/common.h"
#include "driver/uartTerminal/uartTerminal.h"
#include "driver/bus_fdcan/bus_fdcan.h"
#include "driver/ingenia/ingenia.h"
#include "driver/weapon/trigger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if DEBUG_ENABLE==1
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)

#endif	//if DEBUG_ENABLE==1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
reset_cause_e g_reset_cause = RESET_CAUSE_UNKNOWN;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static reset_cause_e get_reset_source()
{
	reset_cause_e ret = RESET_CAUSE_UNKNOWN;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
		ret = RESET_CAUSE_LOW_POWER_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
		ret = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		ret = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		/* This reset is induced by calling the ARM CMSIS */
		/* `NVIC_SystemReset()` function! */
		ret = RESET_CAUSE_SOFTWARE_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
		ret = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;

	return ret;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	g_reset_cause = get_reset_source();
	__HAL_RCC_CLEAR_RESET_FLAGS();
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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	retarget_init();

	LOG("====\r\n");
	LOG("MotorController firmware!\r\n");
	LOG("====\r\n");

	if ((g_reset_cause == RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET) || (g_reset_cause == RESET_CAUSE_SOFTWARE_RESET)) {
		LOG("system is reset from IWDG or s/w reset!\r\n");
	}

	if (g_reset_cause == RESET_CAUSE_EXTERNAL_RESET_PIN_RESET)
		LOG("do you reset this chip?\r\n");

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* TODO User Functions */
uint32_t HAL_GetTick(void)
{
	return osKernelSysTick();
}

void HAL_Delay(uint32_t Delay)
{
	osDelay(Delay);
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == T_JS_PULSE_Pin) {
//		trig_pulse_on();
//		t_js_counter++;
//	}
//}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{

//	if (htim->Instance == htim15.Instance) {
//		trig_start();
////		HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_SET);
////		MAIL_Weapon_t *pTMail;
////		// allocate memory; receiver must be free it
////		pTMail = osMailAlloc(mtr_get_mail(T_Weapon_id), 0);
////		pTMail->sender_id = Weapon_Sender_Sensor_Trigger_id;
////		pTMail->param.sensor.trigger.pulse_on = 1;
////
////		osMailPut(mtr_get_mail(T_Weapon_id), pTMail);
//	}

//	if (htim->Instance == htim15.Instance) {
//		if (firing_mode != FIRE_MODE_INF) {
//			__HAL_TIM_SET_COUNTER(&htim16, 0);
//			HAL_TIM_Base_Start_IT(&htim16);
//		}
//		HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_SET);
//		LOG("Trigger start!\r\n");
//	}

}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[64];
	BUS_Rx_Buffer_t bus_buffer;
	CAN_Data_t ingenia_buffer;

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		if (hfdcan->Instance == hfdcan2.Instance) {
//			Ingenia_IRQHandler(hfdcan);
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if (RxHeader.IdType == FDCAN_STANDARD_ID) {
					ingenia_buffer.id = RxHeader.Identifier;
					ingenia_buffer.len = FDCAN_Convert_Datalength(RxHeader.DataLength);
					memcpy(ingenia_buffer.rxData, RxData, ingenia_buffer.len);

					Ingenia_rx_callback(&ingenia_buffer);
				}
			}
		}
		else if (hfdcan->Instance == hfdcan1.Instance) {
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if (RxHeader.IdType == FDCAN_STANDARD_ID) {
					bus_buffer.lastTimestamp = HAL_GetTick();
					bus_buffer.id = RxHeader.Identifier;
					bus_buffer.len = FDCAN_Convert_Datalength(RxHeader.DataLength);
					memcpy(bus_buffer.data, RxData, bus_buffer.len);
					bus_buffer.counter++;

					bus_rx_callback(&bus_buffer);
				}
			}
		}
	}

}

/**
 * @brief  Rx FIFO 1 callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hfdcan);
	UNUSED(RxFifo1ITs);

	/* NOTE : This function Should not be modified, when the callback is needed,
	 the HAL_FDCAN_RxFifo1Callback could be implemented in the user file
	 */
}

/**
 * @brief  Error status callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  ErrorStatusITs indicates which Error Status interrupts are signaled.
 *         This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
 * @retval None
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	/* Prevent unused argument(s) compilation warning */
	if (hfdcan->Instance == hfdcan1.Instance) {
		MX_FDCAN1_Init();
		bus_init();
		LOG("\r\n===BUS ERROR!===\r\n\r\n");
	}
	else if (hfdcan->Instance == hfdcan2.Instance)
		mtr_error_callback();

	/* NOTE : This function Should not be modified, when the callback is needed,
	 the HAL_FDCAN_ErrorStatusCallback could be implemented in the user file
	 */
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

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
