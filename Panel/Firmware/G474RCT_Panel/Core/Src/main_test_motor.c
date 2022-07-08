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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "bus_fdcan.h"
#include "Button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MTR_AZ_ENABLE			0
#define MTR_EL_ENABLE			1

#define MTR_AZ_SPEED_1			50
#define MTR_AZ_SPEED_2			200
#define MTR_AZ_SPEED_3			RWS_MOTOR_PAN_MAX_SPEED

#define MTR_EL_SPEED_1			100
#define MTR_EL_SPEED_2			10000
#define MTR_EL_SPEED_3			RWS_MOTOR_TILT_MAX_SPEED

typedef struct
{
	uint8_t motorStatus;
	uint8_t weaponStatus;
	uint16_t munitionCounter;
	int32_t panPosition;
	int32_t tiltPosition;
	int32_t panVelocity;
	int32_t tiltVelocity;
} Motor_Weapon_Status_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* TODO Global Variables*/
Bus_Tx_Buffer_t busSendPanel;
Bus_Rx_Buffer_t busRecvMotor;
Bus_Rx_Buffer_t busRecvOptronik;
Bus_Rx_Buffer_t busRecvImu;

Motor_Weapon_Status_t g_motor_weapon_status;
volatile uint8_t g_fdcan_bus_busOff_error = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void PVD_Config(void);
static void busInit(void);
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
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_LPUART1_UART_Init();
	MX_FDCAN1_Init();
	MX_TIM2_Init();
	MX_DMA_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Begin Init*/
	HAL_TIM_PWM_Start(&htim4, LED_CH);
	HAL_TIM_PWM_Start(&htim4, TRIG_CH);
	tim4_out_on(LED_CH);
	tim4_out_off(TRIG_CH);

	uartInitAll();
	bufLen = sprintf(buf, "Main Panel Firmware!\r\n");
	serial_write_str(&debug, buf, bufLen);

	bufLen = sprintf(buf, "to PC!\r\n");
	serial_write_str(&pc, buf, bufLen);

	button_init(&button);
	busInit();

	PVD_Config();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t panelSendTimer = 0;
	uint8_t motorDirection = 0;

	uint8_t motor_counter = 0;
	uint8_t motor_debug_on = 0;
	uint32_t motor_debug_timer = 0;
	while (1) {
		char c;
		Rws_Union_u _val;
		Rws_Union_u mtr[4];
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/
		if (g_fdcan_bus_busOff_error != 0) {
			MX_FDCAN1_Init();
			busInit();

			g_fdcan_bus_busOff_error = 0;
		}

		if (motor_counter != busRecvMotor.counter) {
			motor_counter = busRecvMotor.counter;

			g_motor_weapon_status.motorStatus = busRecvMotor.data[0];
			g_motor_weapon_status.weaponStatus = busRecvMotor.data[1];
			g_motor_weapon_status.munitionCounter = ((uint16_t) busRecvMotor.data[3] << 8)
					| busRecvMotor.data[2];
			for ( int _i = 0; _i < 4; _i++ ) {
				mtr[0].u8[_i] = busRecvMotor.data[4 + _i];
				mtr[1].u8[_i] = busRecvMotor.data[8 + _i];
				mtr[2].u8[_i] = busRecvMotor.data[12 + _i];
				mtr[3].u8[_i] = busRecvMotor.data[16 + _i];
			}
			g_motor_weapon_status.panPosition = mtr[0].i32;
			g_motor_weapon_status.tiltPosition = mtr[1].i32;
			g_motor_weapon_status.panVelocity = mtr[2].i32;
			g_motor_weapon_status.tiltVelocity = mtr[3].i32;

			tim4_out_toggle(LED_CH);
		}

		if (motor_debug_on != 0) {
			if (HAL_GetTick() >= motor_debug_timer) {
				motor_debug_timer = HAL_GetTick() + 500;

				bufLen = sprintf(buf, "(MTR)P:V= %ld %ld\r\n", g_motor_weapon_status.tiltPosition,
						g_motor_weapon_status.tiltVelocity);
				serial_write_str(&debug, buf, bufLen);
			}
		}

#if MTR_AZ_ENABLE==1
		if (serial_available(&debug) > 0) {
			c = serial_read(&debug);

			if (c == 'e') {
				if (!bitRead(busRecvMotor.data[0], 0)) {
					bufLen = sprintf(buf, "Start to enable pan motor!\r\n");
					bitSet(busSendPanel.data[0], 0);
				}
				else {
					bufLen = sprintf(buf, "Start to disable pan motor!\r\n");
					bitClear(busSendPanel.data[0], 0);
				}
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '0') {
				_val.i32 = 0;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 4] = _val.u8[i];
			}
			else if (c == '1') {
				if (motorDirection == 0)
					_val.i32 = MTR_AZ_SPEED_1;
				else
					_val.i32 = -MTR_AZ_SPEED_1;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 4] = _val.u8[i];
			}
			else if (c == '2') {
				if (motorDirection == 0)
					_val.i32 = MTR_AZ_SPEED_2;
				else
					_val.i32 = -MTR_AZ_SPEED_2;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 4] = _val.u8[i];
			}
			else if (c == '3') {
				if (motorDirection == 0)
					_val.i32 = MTR_AZ_SPEED_3;
				else
					_val.i32 = -MTR_AZ_SPEED_3;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 4] = _val.u8[i];
			}
			else if (c == 'R')
				motorDirection = 0;
			else if (c == 'L')
				motorDirection = 1;
		}
#elif MTR_EL_ENABLE==1
		if (serial_available(&debug) > 0) {
			c = serial_read(&debug);

			if (c == 'e') {
				if (!bitRead(g_motor_weapon_status.motorStatus, 1)) {
					bufLen = sprintf(buf, "Start to enable tilt motor!\r\n");
					bitSet(busSendPanel.data[0], 1);
				}
				else {
					bufLen = sprintf(buf, "Start to disable tilt motor!\r\n");
					bitClear(busSendPanel.data[0], 1);
				}
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'D') {
				if (motor_debug_on == 0)
					motor_debug_on = 1;
				else
					motor_debug_on = 0;
			}
			else if (c == '0') {
				_val.i32 = 0;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 8] = _val.u8[i];
				bufLen = sprintf(buf, "mtr Tilt Stop!\r\n");
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '1') {
				if (motorDirection == 0)
					_val.i32 = MTR_EL_SPEED_1;
				else
					_val.i32 = -MTR_EL_SPEED_1;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 8] = _val.u8[i];

				bufLen = sprintf(buf, "tiltSpeed= %ld!\r\n", _val.i32);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '2') {
				if (motorDirection == 0)
					_val.i32 = MTR_EL_SPEED_2;
				else
					_val.i32 = -MTR_EL_SPEED_2;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 8] = _val.u8[i];

				bufLen = sprintf(buf, "tiltSpeed= %ld!\r\n", _val.i32);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '3') {
				if (motorDirection == 0)
					_val.i32 = MTR_EL_SPEED_3;
				else
					_val.i32 = -MTR_EL_SPEED_3;
				for ( int i = 0; i < 4; i++ )
					busSendPanel.data[i + 8] = _val.u8[i];

				bufLen = sprintf(buf, "tiltSpeed= %ld!\r\n", _val.i32);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'U') {
				motorDirection = 0;

				bufLen = sprintf(buf, "mtr Tilt Upward movement!\r\n");
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'D') {
				motorDirection = 1;

				bufLen = sprintf(buf, "mtr Tilt Downward movement!\r\n");
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'L') {
				if (button.panelButton.output.cam_select == 0)
					button.panelButton.output.cam_select = 1;
				else
					button.panelButton.output.cam_select = 0;
				bufLen = sprintf(buf, "led cam=%d\r\n", button.panelButton.output.cam_select);
				serial_write_str(&debug, buf, bufLen);
			}
		}
#endif	//if MTR_AZ_ENABLE==1

		button_handler(&button);

		if (HAL_GetTick() >= panelSendTimer) {
			panelSendTimer = HAL_GetTick() + 50;

			/* Start the Transmission process */
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &busSendPanel.txHeader, busSendPanel.data);
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
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* TODO USER FUNCTIONS*/
static void busInit(void)
{
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

	bufLen = sprintf(buf, "fdcan bus init ...\r\n");
	serial_write_str(&debug, buf, bufLen);

	FDCAN_RX_Filter_Motor(&hfdcan1, 0);
	busRecvMotor.id = RWS_MOTOR_ID;
	FDCAN_RX_Filter_Optronik(&hfdcan1, 1);
	busRecvOptronik.id = RWS_OPTRONIK_ID;
	FDCAN_RX_Filter_Imu(&hfdcan1, 2);
	busRecvImu.id = RWS_IMU_ID;
	FDCAN_TX_Config(&busSendPanel.txHeader, RWS_PANEL_ID, RWS_PANEL_DATA_LENGTH);
	FDCAN_Config(&hfdcan1);

	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
}

void Bus_Notification_Callback(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *data)
{
	if (rxHeader->IdType == FDCAN_STANDARD_ID) {
		if (rxHeader->Identifier == RWS_MOTOR_ID) {
			busRecvMotor.lastTimestamp = HAL_GetTick();
			busRecvMotor.rxHeader = *rxHeader;
			memcpy(busRecvMotor.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
			busRecvMotor.counter++;
		}
		else if (rxHeader->Identifier == RWS_OPTRONIK_ID) {
			busRecvOptronik.lastTimestamp = HAL_GetTick();
			busRecvOptronik.rxHeader = *rxHeader;
			memcpy(busRecvOptronik.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
			busRecvOptronik.counter++;
		}
		else if (rxHeader->Identifier == RWS_IMU_ID) {
			busRecvImu.lastTimestamp = HAL_GetTick();
			busRecvImu.rxHeader = *rxHeader;
			memcpy(busRecvImu.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
			busRecvImu.counter++;
		}
	}
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	g_fdcan_bus_busOff_error = 1;
}

/**
 * @brief  Configures the PVD resources.
 * @param  None
 * @retval None
 */
static void PVD_Config(void)
{
	PWR_PVDTypeDef sConfigPVD;

	/*##-1- Enable Power Clock #################################################*/
	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/*##-2- Configure the NVIC for PVD #########################################*/
	HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);

	/* Configure the PVD Level to 3 and generate an interrupt on rising and falling
	 edges(PVD detection level set to 2.5V, refer to the electrical characteristics
	 of you device datasheet for more details) */
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
//	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; /* trap only when power is down */
	HAL_PWR_ConfigPVD(&sConfigPVD);

	/* Enable the PVD Output */
	HAL_PWR_EnablePVD();
}

/**
 * @brief  PWR PVD interrupt callback
 * @param  none
 * @retval none
 */
void HAL_PWR_PVDCallback(void)
{
	HAL_ADC_MspDeInit(&hadc2);
	HAL_TIM_Base_MspDeInit(&htim2);
	HAL_FDCAN_MspDeInit(&hfdcan1);
	HAL_UART_MspDeInit(&hlpuart1);
	HAL_UART_MspDeInit(&huart2);
	HAL_UART_MspDeInit(&huart1);

	tim4_out_on(LED_CH);
}

void PVD_PVM_IRQHandler(void)
{
	/* Check PWR Exti flag */
	if (__HAL_PWR_PVD_EXTI_GET_FLAG() != RESET) {
		/* PWR PVD interrupt user callback */
		HAL_PWR_PVDCallback();

		/* Clear PWR Exti pending bit */
		__HAL_PWR_PVD_EXTI_CLEAR_FLAG();
	}
}

/* TODO end user function*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
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

