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
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "usart.h"
#include "VL6180X.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_LOW_POWER_RESET,
	RESET_CAUSE_WINDOW_WATCHDOG_RESET,
	RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
	RESET_CAUSE_SOFTWARE_RESET,
	RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
//	RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
//	RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_e;

typedef struct
{
	uint8_t sensor_active :2;
	uint8_t s1_val :1;
	uint8_t s2_val :1;
	uint8_t s1_prev :1;
	uint8_t s2_prev :1;
	uint8_t reserved :2; /* Reserved. */
} __attribute__ ((packed)) Sensor_Change_t;

typedef struct
{
	Sensor_Change_t change;
	uint32_t s1_counter;
	uint32_t s2_counter;
	uint32_t counter;
} Sensor_t;

#define INTER_MEASUREMENT_PERIOD	20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_ENABLE			1
#if DEBUG_ENABLE==1
#define LOG(str, ...) printf("[%05ld] " str, HAL_GetTick()%100000,  ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOG(str, ...)
#define LOG_E(str, ...)
#endif	//if DEBUG_ENABLE==1

#if defined(__IWDG_H__)
#define reset_wdt()		HAL_IWDG_Refresh(&hiwdg)
#else
#define reset_wdt()
#endif	//if defined(__IWDG_H__)

#define SENSOR_MIN_DISTANCE		1
#define SENSOR_MAX_DISTANCE		30
//#define SENSOR_MAX_DISTANCE		50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static reset_cause_e g_reset_cause;
VL6180X_t tof1;
VL6180X_t tof2;
Sensor_t sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t sensor_read(VL6180X_t *sensor, uint8_t *status, uint8_t *range);
static void sensor_update();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void reset_source_init()
{
	g_reset_cause = RESET_CAUSE_UNKNOWN;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
		g_reset_cause = RESET_CAUSE_LOW_POWER_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
		g_reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		g_reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		/* This reset is induced by calling the ARM CMSIS */
		/* `NVIC_SystemReset()` function! */
		g_reset_cause = RESET_CAUSE_SOFTWARE_RESET;
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
		g_reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;

	__HAL_RCC_CLEAR_RESET_FLAGS();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	reset_source_init();
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	debug_uart_init();
	retarget_init(&debug);

	LOG("Munition Counter (VL6180X) firmware!\r\n");

	HAL_Delay(100);
	reset_wdt();
	*(uint8_t*) &sensor.change = 0;
	sensor.counter = 0;
	sensor.s1_counter = sensor.s2_counter = 0;

	int8_t sensor_init;
	LOG("init module 1 ...");
	sensor_init = VL6180X_begin(&tof1, &hi2c1);
	if (sensor_init == -1)
		while (1)
			;
	else if (sensor_init == 1) {
		LOG("fresh restart\r\n");
		VL6180X_set_default_configuration(&tof1);

		VL6180X_range_config_interrupt(&tof1, VL6180X_NEW_SAMPLE_READY);
		VL6180X_range_set_inter_measurement_period(&tof1, INTER_MEASUREMENT_PERIOD);
		VL6180X_range_start_continous_mode(&tof1);
	}
	else if (sensor_init == 0)
		VL6180X_set_fresh_reset(&tof1, 1);

#if DEBUG_ENABLE==1
	printf("done!\r\n");
#endif	//if DEBUG_ENABLE==1

	HAL_Delay(100);
	reset_wdt();

	LOG("init module 2 ...");
	sensor_init = VL6180X_begin(&tof2, &hi2c2);
	if (sensor_init == -1)
		while (1)
			;
	else if (sensor_init == 1) {
		LOG("fresh restart\r\n");
		VL6180X_set_default_configuration(&tof2);

		VL6180X_range_config_interrupt(&tof2, VL6180X_NEW_SAMPLE_READY);
		VL6180X_range_set_inter_measurement_period(&tof2, INTER_MEASUREMENT_PERIOD);
		VL6180X_range_start_continous_mode(&tof2);
	}
	else if (sensor_init == 0)
		VL6180X_set_fresh_reset(&tof2, 1);

#if DEBUG_ENABLE==1
	printf("done!\r\n");
#endif	//if DEBUG_ENABLE==1

	HAL_Delay(100);
	reset_wdt();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t debug_adv = 1;

	uint8_t start_to_shutdown = 0;
	uint32_t display_timer = 0;
	while (1) {
		uint8_t status = VL6180X_MAX_S_N_ERR, range = 0;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		reset_wdt();

		if (start_to_shutdown == 0) {
			status = VL6180X_MAX_S_N_ERR, range = 0;
			if (sensor_read(&tof1, &status, &range) == 1) {
				if (debug_adv == 1)
					LOG("1,%d,%d\r\n", status, range);
				if (status == VL6180X_NO_ERR) {
					sensor.change.sensor_active = 1;
					if (range >= SENSOR_MIN_DISTANCE && range <= SENSOR_MAX_DISTANCE)
						sensor.change.s1_val = 1;
					else
						sensor.change.s1_val = 0;

					sensor_update();
				}
			}

			status = VL6180X_MAX_S_N_ERR, range = 0;
			if (sensor_read(&tof2, &status, &range) == 1) {
				if (debug_adv == 1)
					LOG("2,%d,%d\r\n", status, range);
				if (status == VL6180X_NO_ERR) {
					sensor.change.sensor_active = 2;
					if (range >= SENSOR_MIN_DISTANCE && range <= SENSOR_MAX_DISTANCE)
						sensor.change.s2_val = 1;
					else
						sensor.change.s2_val = 0;

					sensor_update();
				}
			}
		}

		if (debug_adv == 0) {
			if (HAL_GetTick() >= display_timer) {
				display_timer = HAL_GetTick() + 100;

				LOG("mc=%ld,%ld,%ld\r\n", sensor.counter, sensor.s1_counter, sensor.s2_counter);
			}
		}

		if (serial_available(&debug) > 0) {
			char c = serial_read(&debug);

			if (c == 'R')
				NVIC_SystemReset();
			else if (c == '0')
				sensor.counter = sensor.s1_counter = sensor.s2_counter = 0;
			else if (c == 's') {
				start_to_shutdown = 1;
				VL6180X_set_fresh_reset(&tof1, 1);
				VL6180X_set_fresh_reset(&tof2, 1);
			}
			else if (c == 'd') {
				if (debug_adv == 0)
					debug_adv = 1;
				else
					debug_adv = 0;
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
static uint8_t sensor_read(VL6180X_t *sensor, uint8_t *status, uint8_t *range)
{
	if (VL6180X_range_get_interrupt_status(sensor) == VL6180X_NEW_SAMPLE_READY) {

		/*Get the measured distance data*/
		*range = VL6180X_range_get_measurement(sensor);
		/*Get the judged result of the range value*/
		*status = VL6180X_get_range_result(sensor);
		/*Clear interrupts generated by measuring range*/
		VL6180X_clear_range_interrupt(sensor);

//		printf("%ld,%d,%d\r\n", HAL_GetTick(), status, range);

		return 1;
	}
	return 0;
}

static void sensor_update()
{
	if (sensor.change.sensor_active != 0) {
		if (sensor.change.sensor_active == 1) {
			if (sensor.change.s1_val != sensor.change.s1_prev) {

				if (sensor.change.s2_prev == 1) {
					if (sensor.change.s1_val == 1)
						sensor.counter++;
//					else
//						sensor.counter--;
				}

				if (sensor.change.s1_val == 0)
					sensor.s1_counter++;

				sensor.change.s1_prev = sensor.change.s1_val;
			}
		}
		else if (sensor.change.sensor_active == 2) {
			if (sensor.change.s2_val != sensor.change.s2_prev) {

				if (sensor.change.s1_prev == 1)
					if (sensor.change.s2_val == 1 && sensor.counter > 0)
						sensor.counter--;

				if (sensor.change.s2_val == 0)
					sensor.s2_counter++;

				sensor.change.s2_prev = sensor.change.s2_val;
			}
		}

		sensor.change.sensor_active = 0;
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
