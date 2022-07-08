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
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rwsCanID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN_BUFSIZE					8

typedef union
{
	float f;
	int32_t i32;
	uint32_t u32;
	uint8_t b[4];
} Union_u;

typedef union
{
	int16_t i16;
	uint8_t b[2];
} Union_YPR;

//typedef struct
//{
//	uint32_t id;
//	volatile bool state;
//	uint8_t data[CAN_BUFSIZE];
//	uint8_t size;
//	uint8_t online;
//	uint32_t debugCounter;
//} TCanRecvBuffer;

typedef struct
{
	uint32_t id;
	uint8_t data[CAN_BUFSIZE];
	uint8_t size;
	uint32_t debugCounter;
} TCanSendBuffer;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_IMU				1

#if DEBUG_IMU==1
#define LOG(str, ...) printf("[%lX %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILENAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOG(str, ...)
#define LOG_E(str, ...)
#endif	//if DEBUG_IMU==1

#define IMU_MAX_BUFSIZE			128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t imu_dma_buf[IMU_MAX_BUFSIZE];
char imuBuf[IMU_MAX_BUFSIZE];
uint8_t imu_updated = 0;
uint8_t imu_ready_to_send = 0;
/* CAN BUS */
CAN_TxHeaderTypeDef can1TxHeader;
uint32_t can1TxMailBox;
uint8_t can1TxBuffer[CAN_BUFSIZE];

//TCanSendBuffer canSendYpr = { CAN_ID_RWS_PLAT_YPR, { 0 }, 8, 0 };
TCanSendBuffer canSendYpr = { CAN_ID_RWS_PLAT_YPR_SLOW, { 0 }, 8, 0 };
TCanSendBuffer canSendYprSlow = { CAN_ID_RWS_PLAT_YPR_SLOW, { 0 }, 8, 0 };

uint32_t yprCounter = 0;
//uint32_t yprSlowCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Config();
static void imu_init();
static void imu_handler();
static void bus_init();
static void bus_handler();
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
	MX_IWDG_Init();
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Init */
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(500);
	retarget_init(&huart2);
	LOG("RWS-IMU on platform firmware!\r\n");

	imu_init();
	bus_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t timer_1s = 0;
	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		imu_handler();
		bus_handler();

		if (HAL_GetTick() >= timer_1s) {
			timer_1s = HAL_GetTick() + 1000;

			yprCounter = canSendYpr.debugCounter;
			canSendYpr.debugCounter = 0;
			LOG("tes\r\n");
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* TODO user functions */

static void sendToBus(const float yaw, const float pitch, const float roll)
{
	int16_t i16;
	int32_t pitch32, roll32, i32;

	i16 = (int16_t) (yaw * 100.0f);
	canSendYpr.data[0] = i16 & 0xFF;
	canSendYpr.data[1] = (i16 >> 8) & 0xFF;

	pitch32 = (int32_t) (pitch * 1000.0f);
	if (pitch32 < 0)
		i32 = 0x1000000 + pitch32;
	else
		i32 = pitch32;

	canSendYpr.data[2] = i32 & 0xFF;
	canSendYpr.data[3] = (i32 >> 8) & 0xFF;
	canSendYpr.data[4] = (i32 >> 16) & 0xFF;

	roll32 = (int32_t) (roll * 1000.0f);
	if (roll32 < 0)
		i32 = 0x1000000 + roll32;
	else
		i32 = roll32;

	canSendYpr.data[5] = i32 & 0xFF;
	canSendYpr.data[6] = (i32 >> 8) & 0xFF;
	canSendYpr.data[7] = (i32 >> 16) & 0xFF;

	imu_ready_to_send = 1;

//	LOG("%02X %02X %02X %02X %02X %02X %02X %02X\r\n", canSendYpr.data[0], canSendYpr.data[1], canSendYpr.data[2],
//			canSendYpr.data[3], canSendYpr.data[4], canSendYpr.data[5],
//			canSendYpr.data[6], canSendYpr.data[7]);
}

static void imu_init()
{
	memset(imu_dma_buf, 0, IMU_MAX_BUFSIZE);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_dma_buf, IMU_MAX_BUFSIZE);
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

static char** str_split(char *a_str, const char a_delim)
{
	char **result = 0;
	size_t count = 0;
	char *tmp = a_str;
	char *last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp) {
		if (a_delim == *tmp) {
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = malloc(sizeof(char*) * count);

	if (result) {
		size_t idx = 0;
		char *token = strtok(a_str, delim);

		while (token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return result;
}

static void imu_handler()
{
	char *s;
	char **tokens;
	float yaw, pitch, roll;

	if (imu_updated == 1) {
		imu_updated = 0;

//		LOG("%s\r\n", imuBuf);
		s = strstr(imuBuf, "$VNYPR,");
		if (s) {
			tokens = str_split(imuBuf, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					switch (i)
					{
					case 1:
						/* yaw */
						yaw = atof(s);
						break;
					case 2:
						/* pitch */
						pitch = atof(s);
						break;
					case 3:
						/* roll */
						roll = atof(s);
						break;
					}
					free(*(tokens + i));
				}	//for ( int i = 0; *(tokens + i); i++ ) {
				free(tokens);

				LOG("ypr: %.3f %.3f %.3f\t\t%ld\r\n", yaw, pitch, roll, yprCounter);
//				bufLen = sprintf(buf, "ypr= %.3f %.3f %.3f\t%ld %ld\r\n", yaw, pitch, roll,
//						yprCounter, yprSlowCounter);
//				serial_write_str(&debug, buf, bufLen);
				sendToBus(yaw, pitch, roll);
			}	//if (tokens) {
		} /*if (s) {*/

	}
}

/* TODO h/w related */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1) {
		memcpy(imuBuf, imu_dma_buf, IMU_MAX_BUFSIZE);
		memset(imu_dma_buf, 0, IMU_MAX_BUFSIZE);
		imu_updated = 1;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_dma_buf, IMU_MAX_BUFSIZE);
		__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
	}

}

static void bus_init()
{
	CAN_Config();
}

static void bus_handler()
{
	if (imu_ready_to_send == 1) {

		// Send Data
		can1TxHeader.StdId = canSendYpr.id;
		can1TxHeader.DLC = canSendYpr.size;
		memcpy(can1TxBuffer, canSendYpr.data, canSendYpr.size);

		if (HAL_CAN_AddTxMessage(&hcan, &can1TxHeader, can1TxBuffer, &can1TxMailBox) == HAL_OK) {
			canSendYpr.debugCounter++;
			imu_ready_to_send = 0;
		}
	}
}

static void CAN_Config()
{
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
//	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
//			!= HAL_OK) {
//		__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//	}
	/*##-5- Configure Transmission process #####################################*/
	can1TxHeader.StdId = 0x123;
	can1TxHeader.ExtId = 0x01;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = CAN_BUFSIZE;
	can1TxHeader.TransmitGlobalTime = DISABLE;
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

