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
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "stm_hal_serial.h"
#include "Ingenia_FdcanServoDriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union
{
	float f;
	uint32_t u32;
	int32_t i32;
	uint8_t b[4];
} Union_u;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define MTR_AZ_ID				0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxDBuffer[RING_BUFFER_RX_SIZE];
uint8_t txDBuffer[RING_BUFFER_TX_SIZE];
TSerial debug;

Servo_t mtrAzi;
volatile uint32_t countTPDO4 = 0;
FDCAN_HandleTypeDef *motorBus = &hfdcan2;

int posActual, veloActual;
uint8_t tpdo4Recv = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef motorInit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FDCAN_TxHeaderTypeDef TxHeader;
/**
 * @brief  Configures the FDCAN.
 * @param  None
 * @retval None
 */
static void fdcan_config(FDCAN_HandleTypeDef *fdcan)
{
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;
	sFilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(fdcan) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
		Error_Handler();
	}

}

static void fdcan_set_tx_header(FDCAN_TxHeaderTypeDef *txHeader, uint32_t id)
{
	/* Prepare Tx Header */
	txHeader->Identifier = id;
	txHeader->IdType = FDCAN_STANDARD_ID;
	txHeader->TxFrameType = FDCAN_DATA_FRAME;
	txHeader->DataLength = FDCAN_DLC_BYTES_8;
	txHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader->BitRateSwitch = FDCAN_BRS_OFF;
	txHeader->FDFormat = FDCAN_CLASSIC_CAN;
	txHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader->MessageMarker = 0;

}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//
//	uint16_t bufLen;
//	char buf[RING_BUFFER_TX_SIZE];
//
//	FDCAN_RxHeaderTypeDef RxHeader;
//	uint8_t RxData[64];
//	uint32_t _id;
//
//	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//		Ingenia_IRQHandler(hfdcan);
//
////		/* Retrieve Rx messages from RX FIFO0 */
////		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
////			Error_Handler();
////		}
////		else {
////			_id = RxHeader.Identifier;
////			tpdo4Recv = 1;
////
////			uint32_t id, idType;
////			id = RxHeader.Identifier;
////
////			if (id < COB_NMT_CTRL)
////				id %= 0x80;
////			else if (id < COB_NMT_CTRL + 0x80)
////				id -= COB_NMT_CTRL;
////
////			idType = RxHeader.Identifier - id;
////
////			bufLen = sprintf(buf, "id= 0x%lX %d 0x%lX\r\n", _id,
////					(RxHeader.IdType == FDCAN_STANDARD_ID) ? 1 : 0, idType);
////			serial_write_str(&debug, buf, bufLen);
////
////			bufLen = sprintf(buf, "data= %02X %02X %02X %02X %02X %02X %02X %02X\r\n", RxData[0],
////					RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
////			serial_write_str(&debug, buf, bufLen);
////		}
//	}
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	uint16_t bufLen;
	char buf[RING_BUFFER_TX_SIZE];
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
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	serial_init(&debug, (char*) &rxDBuffer, sizeof(rxDBuffer), (char*) &txDBuffer,
			sizeof(txDBuffer), &huart1);
	bufLen = sprintf(buf, "Ingenia CAN library\r\n");
	serial_write_str(&debug, buf, bufLen);

//	Ingenia_debug_init(&debug, &hfdcan2);
//	fdcan_config(&hfdcan2);
//	fdcan_set_tx_header(&TxHeader, 0x20);
//	Ingenia_init(&mtrAzi, MTR_AZ_ID);

//	Ingenia_begin(&hfdcan2);
//	if (Ingenia_init(&mtrAzi, MTR_AZ_ID) != HAL_OK) {
//		bufLen = sprintf(buf, "init failed!\r\n");
//		serial_write_str(&debug, buf, bufLen);
//		return HAL_ERROR;
//	}

	if (motorInit() != HAL_OK) {
		HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t ledTimer = 0;
	Union_u p, v;
	uint8_t motorEnable = 0;
	while (1) {
		uint8_t canTxBuffer[CAN_DATA_MAX];
		uint8_t _dataRpdo[8];
		int32_t _pos;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO begin loop*/
		if (HAL_GetTick() >= ledTimer) {
			ledTimer = HAL_GetTick() + 1000;

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			bufLen = sprintf(buf, "p:v= %d:%d\r\n", posActual, veloActual);
			serial_write_str(&debug, buf, bufLen);
		}

		if (serial_available(&debug)) {
			char c = serial_read(&debug);

			if (c == '0') {
				Ingenia_setTargetPositionVelocity(&mtrAzi, 0, 0, 1, 0, 1);
				bufLen = sprintf(buf, "motor Stop!\r\n");
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'e') {
				if (motorEnable == 0) {
					Ingenia_enableMotor(&mtrAzi);
					bufLen = sprintf(buf, "motor Enable!\r\n");
					motorEnable = 1;
				}
				else {
					Ingenia_disableMotor(&mtrAzi);
					bufLen = sprintf(buf, "motor Disable!\r\n");
					motorEnable = 0;

				}
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == '<') {
				if (motorEnable > 0) {
					_pos = posActual - 1000;
					Ingenia_setTargetPositionVelocity(&mtrAzi, _pos, 1000UL, 1, 0, 0);
					bufLen = sprintf(buf, "move left! %ld\r\n", _pos);
					serial_write_str(&debug, buf, bufLen);
				}
			}
			else if (c == '>') {
				if (motorEnable > 0) {
					_pos = posActual + 1000;
					Ingenia_setTargetPositionVelocity(&mtrAzi, _pos, 1000UL, 1, 0, 0);
					bufLen = sprintf(buf, "move right! %ld\r\n", _pos);
					serial_write_str(&debug, buf, bufLen);
				}
			}
		}
		/* TODO End loop*/
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_FDCAN;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* TODO user functions*/
void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}

static void motorParamInit(Servo_t *servo)
{

	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);

	/* set mode to profile position */
	Ingenia_setModeOfOperation(servo, DRIVE_MODE_PROFILE_POSITION);

	/* set max motor speed */
	Ingenia_write_sdo_u32(servo, 0x6080, 0, 100000UL);
	/* set max velocity profile */
	Ingenia_write_sdo_u32(servo, 0x607F, 0, 100000UL);
	/* set max profile acceleration */
	Ingenia_write_sdo_u32(servo, 0x6083, 0, 1000000UL);
	/* set max profile de-acceleration */
	Ingenia_write_sdo_u32(servo, 0x6084, 0, 1000000UL);
	/* set max profile quick stop de-acceleration */
	Ingenia_write_sdo_u32(servo, 0x6085, 0, 2000000UL);
	/* set TPDO4 event timer */
	Ingenia_write_sdo_u16(servo, 0x1803, 5, 10);
}

static HAL_StatusTypeDef motorInit()
{
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

	bufLen = sprintf(buf, "motor init...\r\n");
	serial_write_str(&debug, buf, bufLen);
	serial_write_flush(&debug);

	Ingenia_begin(&hfdcan2);
	if (Ingenia_init(&mtrAzi, MTR_AZ_ID) != HAL_OK) {
		bufLen = sprintf(buf, "init failed!\r\n");
		serial_write_str(&debug, buf, bufLen);
		return HAL_ERROR;
	}

	bufLen = sprintf(buf, "enabling pan motor...");
	serial_write_str(&debug, buf, bufLen);
	serial_write_flush(&debug);

	motorParamInit(&mtrAzi);
	Ingenia_disableMotor(&mtrAzi);

	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);

	return HAL_OK;
}

void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;
	uint32_t idNode = 0;
	int _pos = 0, _velo = 0;

	if (can_buffer_available(buffer) > 0) {
		can_buffer_read(buffer, &data);
		if ((data.canRxHeader.Identifier & COB_TPDO4) == COB_TPDO4) {
			countTPDO4++;

			idNode = data.canRxHeader.Identifier - COB_TPDO4;
			if (idNode == mtrAzi._u8Node) {
				for ( int i = 0; i < 4; i++ ) {
					_pos |= (int) data.rxData[i] << (8 * i);
					_velo |= (int) data.rxData[i + 4] << (8 * i);
				}
				posActual = _pos;
				veloActual = _velo;
			}

		}

	}
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(LIM_EL_UP_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(LIM_EL_UP_Pin);

	}
	else if (__HAL_GPIO_EXTI_GET_IT(LIM_EL_DOWN_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(LIM_EL_DOWN_Pin);

	}
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(COUNTER_PULSE_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(COUNTER_PULSE_Pin);

	}

	if (__HAL_GPIO_EXTI_GET_IT(COCK_MAX_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(COCK_MAX_Pin);

	}
	else if (__HAL_GPIO_EXTI_GET_IT(COCK_MIN_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(COCK_MIN_Pin);

	}
}

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(LIM_AZ_ZERO_Pin) != 0) {
		__HAL_GPIO_EXTI_CLEAR_IT(LIM_AZ_ZERO_Pin);

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
		HAL_UART_Transmit(&huart1, "error!\r\n", 8, 100);
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
