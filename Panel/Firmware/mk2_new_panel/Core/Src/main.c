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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "rwsCanID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint32_t id;
//	volatile bool state;
	uint8_t counter;
	uint8_t data[8];
	uint8_t size;
	uint8_t online;
} TCanRecvBuffer;

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t size;
} TCanSendBuffer;

typedef struct
{
	uint8_t id;
	uint8_t dtab;
	int azimuth;	// scale -1000 to +1000
	int elevation;	// scale -1000 to +1000
} Joystick_t;

typedef struct
{
	uint8_t id;
	int16_t dx;
	int16_t dy;
} Track_Target_t;

typedef struct
{
	uint8_t state;
	Track_Target_t target[5];
} Track_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILENAME__,__LINE__, ##__VA_ARGS__)

#define PC_MAX_BUFSIZE		512
#define BUTTON_MAX_BUFSIZE	256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t pc_dma_buf[PC_MAX_BUFSIZE];
char in_pc_buf[PC_MAX_BUFSIZE];
bool in_pc_updated = false;

uint8_t button_dma_buf[BUTTON_MAX_BUFSIZE];
char in_button_buf[BUTTON_MAX_BUFSIZE];
bool in_button_updated = false;

TCanRecvBuffer canRecvMotorState = { CAN_ID_RWS_MOTOR, 0, { 0 }, 6, 0 };
TCanRecvBuffer canRecvMotorAngle = { CAN_ID_RWS_MTR_STAB_ANGLE, 0, { 0 }, 8, 0 };
//TCanRecvBuffer canRecvMotorSpeed = { CAN_ID_RWS_MTR_STAB_SPD, 0, { 0 }, 8, 0 };
TCanRecvBuffer canRecvOptLrf = { CAN_ID_RWS_OPT_LRF, 0, { 0 }, 3, 0 };
TCanRecvBuffer canRecvOptCam = { CAN_ID_RWS_OPT_CAM, 0, { 0 }, 1, 0 };
TCanRecvBuffer canRecvPlatformYPR = { CAN_ID_RWS_PLAT_YPR_SLOW, 0, { 0 }, 8, 0 };


TCanSendBuffer canSendButton = { CAN_ID_RWS_BUTTON, { 0 }, 3 };
TCanSendBuffer canSendMotorMode = { CAN_ID_RWS_PNL_MTR_MODE, { 0 }, 1 };
TCanSendBuffer canSendMotorManual = { CAN_ID_RWS_PNL_MTR_CORR_MAN, { 0 }, 8 };
TCanSendBuffer canSendMotorStab = { CAN_ID_RWS_PNL_MTR_CORR_STB, { 0 }, 8 };
TCanSendBuffer canSendMotorTrack = { CAN_ID_RWS_PNL_MTR_CORR_TRK, { 0 }, 8 };

CAN_TxHeaderTypeDef can1TxHeader;
uint32_t can1TxMailBox;

Joystick_t jRight = { 0, 0, 0, 0 };
Joystick_t jLeft = { 1, 0, 0, 0 };

//uint8_t track_state = 0;
Track_t track;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Config();
static void bus_init();
static void bus_handler();
static void pc_init();
static void pc_handler();
static void button_init();
static void button_handler();
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
	MX_CAN1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	/* TODO init */

	retarget_init(&huart2);
	LOG("panel init\r\n");

	bus_init();
	button_init();
	pc_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t ledTimer = 0;
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO begin loop */
		bus_handler();
		pc_handler();
		button_handler();

		if (HAL_GetTick() >= ledTimer) {
			ledTimer = HAL_GetTick() + 1000;

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//			LOG("led=%d\r\n", HAL_GPIO_ReadPin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin));
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* TODO user functions */

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

static void bus_init()
{
	CAN_Config();

}

static void bus_handler()
{
	static uint8_t imu_counter = 0;
	static uint32_t imu_timer = 0;
	static uint32_t imu_increment = 0;
	uint32_t u32;

	static uint32_t motor_mode_timer = 0;

	if (imu_counter != canRecvPlatformYPR.counter) {
		imu_counter = canRecvPlatformYPR.counter;

		imu_increment++;
	}

	if (HAL_GetTick() >= imu_timer) {
		imu_timer = HAL_GetTick() + 1000;

		u32 = imu_increment;
		imu_increment = 0;
		LOG("imu = %ld/s\r\n", u32);
	}

	if (HAL_GetTick() >= motor_mode_timer) {
//		motor_mode_timer=HAL_GetTick()+100;

		/* send can */
		can1TxHeader.StdId = canSendMotorMode.id;
		can1TxHeader.DLC = canSendMotorMode.size;
		if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, canSendMotorMode.data, &can1TxMailBox)
				== HAL_OK)
			motor_mode_timer = HAL_GetTick() + 100;
		else
			motor_mode_timer = HAL_GetTick() + 1;
	}
}

static void pc_init()
{
	memset(pc_dma_buf, 0, PC_MAX_BUFSIZE);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pc_dma_buf, PC_MAX_BUFSIZE);
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

static void pc_handler()
{
	if (in_pc_updated) {
		in_pc_updated = false;

//		LOG("%s\r\n", in_pc_buf);

		uint8_t _id;
		char *s;
		char **tokens;

		s = strstr(in_button_buf, "$TRKUD,");
		if (s) {
			tokens = str_split(in_button_buf, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					if (i == 1) {
						_id = atoi(s);
						if (_id >= 5)
							_id = 4;
					}
					else if (i == 2)
						track.target[_id].dx = atoi(s);
					else if (i == 3)
						track.target[_id].dy = atoi(s);

					free(*(tokens + i));
				}
				free(tokens);
			}
		}

	}
}

static void button_init()
{
	memset(button_dma_buf, 0, BUTTON_MAX_BUFSIZE);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, button_dma_buf, BUTTON_MAX_BUFSIZE);
	__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

static void button_handler()
{
	uint32_t _btnState = 0;
	bool _btnReady = false;
//	int16_t _i16;
//	const float alphaRight = 0.5;
//	const float alphaLeft = 0.5;
//	static int jsRight[2] = { 0, 0 };
//	static int jsLeft[2] = { 0, 0 };
	static uint8_t R_dtab = 0;
	static uint8_t L_dtab = 0;

	if (in_button_updated) {
		in_button_updated = false;

//		LOG("%s\r\n", in_button_buf);

		char *s;
		char **tokens;

		s = strstr(in_button_buf, "$BTN,");
		if (s) {
			tokens = str_split(in_button_buf, ',');
			if (tokens) {
				for ( int i = 0; *(tokens + i); i++ ) {
					s = *(tokens + i);

					switch (i)
					{
					case 1:
						_btnState = atoi(s);
						_btnReady = true;
						break;
					case 2:
						jRight.dtab = atoi(s);
						break;
					case 3:
					case 4:
						break;
					case 5:
						jLeft.dtab = atoi(s);
						break;
					case 6:
					case 7:
						break;
					}
					free(*(tokens + i));
				}
				free(tokens);
			}

//			if(_btnReady){
//				/* CAMERA SELECT
//				 * 0: sony
//				 * 1: thermal
//				 */
//				bitWrite(canSendButton.data[1], 0, bitRead(_btnState,12));
//
//			}

			if (jLeft.dtab != L_dtab) {
				L_dtab = jLeft.dtab;

				if ((L_dtab & 0b1000) != 0) {
					if (track.state == 0) {
						track.state = 1;
						canSendMotorMode.data[0] = 0b11;
						LOG("track ON\r\n");
					}
					else {
						track.state = 0;
						canSendMotorMode.data[0] = 0;
						LOG("track OFF\r\n");
					}
				}
			}

		}
	}
}

/* TODO hardware-related */
static void CAN_Config()
{
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	/* filter can id = CAN_ID_RWS_MOTOR= 0x300 - 0x30F */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x300 << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (0x7F0 << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_OPT_x= 0x330 - 0x33F */
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (0x330 << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (0x7F0 << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/* filter can id = CAN_ID_RWS_STAB_YPR */
	sFilterConfig.FilterBank = 2;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (canRecvPlatformYPR.id << 5);
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (canRecvPlatformYPR.id << 5);
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	/*##-5- Configure Transmission process #####################################*/
	can1TxHeader.StdId = 0x123;
	can1TxHeader.ExtId = 0x01;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = 8;
	can1TxHeader.TransmitGlobalTime = DISABLE;

#if USE_CAN2
	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = (0x100 << 5);
	sFilterConfig.FilterIdLow = (0x100 << 5);
	sFilterConfig.FilterMaskIdHigh = (0x700 << 5);
	sFilterConfig.FilterMaskIdLow = (0x700 << 5);
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	can2TxHeader.StdId = 0x21;
	can2TxHeader.ExtId = 0x01;
	can2TxHeader.RTR = CAN_RTR_DATA;
	can2TxHeader.IDE = CAN_ID_STD;
	can2TxHeader.DLC = CAN_BUFSIZE;
	can2TxHeader.TransmitGlobalTime = DISABLE;

#endif	//if USE_CAN2

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t *buf;
	uint16_t bufsize = 0;

	if (huart->Instance == USART1) {
		memcpy(in_pc_buf, pc_dma_buf, PC_MAX_BUFSIZE);
		memset(pc_dma_buf, 0, PC_MAX_BUFSIZE);
		in_pc_updated = true;

		buf = pc_dma_buf;
		bufsize = PC_MAX_BUFSIZE;
	}
	else if (huart->Instance == USART3) {
		memcpy(in_button_buf, button_dma_buf, BUTTON_MAX_BUFSIZE);
		memset(button_dma_buf, 0, BUTTON_MAX_BUFSIZE);
		in_button_updated = true;

		buf = button_dma_buf;
		bufsize = BUTTON_MAX_BUFSIZE;
	}

	/* start the DMA again */
	HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, bufsize);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t _id = 0;
	CAN_RxHeaderTypeDef can1RxHeader;
	uint8_t can1RxBuffer[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxBuffer) == HAL_OK) {
		_id = can1RxHeader.StdId;
		if (_id == canRecvOptCam.id) {
			canRecvOptCam.counter++;
			memcpy(canRecvOptCam.data, can1RxBuffer, canRecvOptCam.size);
		}
		else if (_id == canRecvOptLrf.id) {
			canRecvOptLrf.counter++;
			memcpy(canRecvOptLrf.data, can1RxBuffer, canRecvOptLrf.size);
		}
		else if (_id == canRecvMotorState.id) {
			canRecvMotorState.counter++;
			memcpy(canRecvMotorState.data, can1RxBuffer, canRecvMotorState.size);
		}
		else if (_id == canRecvMotorAngle.id) {
			canRecvMotorAngle.counter++;
			memcpy(canRecvMotorAngle.data, can1RxBuffer, canRecvMotorAngle.size);
		}
		else if (_id == canRecvPlatformYPR.id) {
			canRecvPlatformYPR.counter++;
			memcpy(canRecvPlatformYPR.data, can1RxBuffer, canRecvPlatformYPR.size);
		}
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

