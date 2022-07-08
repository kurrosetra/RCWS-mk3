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
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "driver/bus_can/bus_can.h"
#include "config/bus_button_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailBox;

	Bus_Tx_Buffer_t tx_opt_lrf;
	Bus_Tx_Buffer_t tx_opt_cam;

	Bus_Rx_Buffer_t rx_panel_command;
} Bus_t;

static Bus_t bus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_ENABLE			1
#if DEBUG_ENABLE==1
#define DEBUG_BUS				0

#define DEBUG_SONY				1
#define DEBUG_THC				0
#define DEBUG_LRF				0
#endif	//if DEBUG_ENABLE==1

#if DEBUG_ENABLE==1
#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOG(str, ...)
#define LOG_E(str, ...)
#endif	//if DEBUG_ENABLE==1

#if DEBUG_BUS==1
#define LOGB(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOGB_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOGB(str, ...)
#define LOGB_E(str, ...)
#endif	//if DEBUG_BUS==1

#if DEBUG_SONY==1
#define LOGS(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOGS_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOGS(str, ...)
#define LOGS_E(str, ...)
#endif	//if DEBUG_SONY==1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Panel_camera_command_t p_cmd;
Optronik_camera_state_t cam_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void camera_init();
static void camera_handler();

static HAL_StatusTypeDef can_init();
static HAL_StatusTypeDef bus_init();
static HAL_StatusTypeDef bus_send(Bus_Tx_Buffer_t *buffer);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	uart_init_all();

//	/* for serial passthrough purpose */
//	uint32_t _led_timer = 0;
//	while (1) {
//		HAL_IWDG_Refresh(&hiwdg);
//
//		if (HAL_GetTick() >= _led_timer) {
//			_led_timer = HAL_GetTick() + 500;
//
//			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//		}
//
//		if (serial_available(&sony) > 0)
//			serial_write(&debug, serial_read(&sony));
//
//		if (serial_available(&debug) > 0)
//			serial_write(&sony, serial_read(&debug));
//	}

	LOG("Optronik Firmware!\r\n");

	HAL_GPIO_WritePin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAMERA_ENABLE_GPIO_Port, CAMERA_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(THERMAL_PWR_EN_GPIO_Port, THERMAL_PWR_EN_Pin, GPIO_PIN_SET);

	uint32_t led_timer = 0;
	uint8_t video_mode = 0;
	while (1) {
		char c;

		HAL_IWDG_Refresh(&hiwdg);

		if (serial_available(&thc) > 0) {
			c = serial_read(&thc);

			printf("%02X ", (uint8_t) c);
		}

		if (serial_available(&debug) > 0) {
			c = serial_read(&debug);

			if (c == 'e') {
				HAL_GPIO_TogglePin(CAMERA_ENABLE_GPIO_Port, CAMERA_ENABLE_Pin);
				LOG("cam enable= %d\r\n", HAL_GPIO_ReadPin(CAMERA_ENABLE_GPIO_Port, CAMERA_ENABLE_Pin));
			}
			else if (c == 's') {
				HAL_GPIO_TogglePin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin);
				LOG("cam select= %d\r\n", HAL_GPIO_ReadPin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin));
			}
			else if (c == 'm') {
				if (video_mode == 0)
					video_mode = 1;
				else
					video_mode = 0;

				serial_write(&thc, 0xA0);
				serial_write(&thc, 0x79);
				serial_write(&thc, video_mode);
				serial_write(&thc, 0x00);
				serial_write(&thc, 0x00);
				serial_write(&thc, 0xFF);

				LOG("video mode= %d\r\n", video_mode);
			}
		}

		if (HAL_GetTick() >= led_timer) {
			led_timer = HAL_GetTick() + 500;

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}
	}

	bus_init();

	HAL_Delay(500);
	camera_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t debug_timer = 1000;
	uint8_t panel_counter = 0;
	while (1) {
		char c;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);

		camera_handler();

		if (bus.rx_panel_command.counter != panel_counter) {
			panel_counter = bus.rx_panel_command.counter;

			LOGB("recv: ");

			*(uint8_t*) &p_cmd = bus.rx_panel_command.data[4];
#if DEBUG_BUS==1
			for ( int i = 0; i < bus.rx_panel_command.len; i++ ) {
				printf("%02X ", bus.rx_panel_command.data[i]);
			}
			printf("\r\n");
#endif	//if DEBUG_BUS==1

			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}

		if (HAL_GetTick() >= debug_timer) {
			debug_timer = HAL_GetTick() + 500;

			bus.tx_opt_cam.data[0] = *(uint8_t*) &cam_state;
			bus_send(&bus.tx_opt_cam);
		}

		if (HAL_GetTick() >= (bus.rx_panel_command.lastTimestamp + 5000))
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* TODO User Function */

/* ====== */
/* CAMERA */
/* ====== */
static void camera_init()
{
//	HAL_GPIO_WritePin(THERMAL_PWR_EN_GPIO_Port, THERMAL_PWR_EN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(CAMERA_ENABLE_GPIO_Port, CAMERA_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAMERA_SELECT_GPIO_Port, CAMERA_SELECT_Pin, GPIO_PIN_SET);

}

static void c_sony_set_auto_focus(const uint8_t en)
{
	const uint8_t af_auto[6] = { 0x81, 0x01, 0x04, 0x38, 0x02, 0xFF };
	const uint8_t af_manual[6] = { 0x81, 0x01, 0x04, 0x38, 0x03, 0xFF };

	for ( int i = 0; i < 6; i++ ) {
		if (en == 0)
			serial_write(&sony, af_manual[i]);
		else
			serial_write(&sony, af_auto[i]);
	}
}

static void c_sony_set_zoom(uint8_t level)
{
	const uint8_t z[3][9] = { { 0x81, 0x01, 0x04, 0x47, 0x02, 0x0, 0x0, 0x00, 0xFF }, { 0x81, 0x01, 0x04, 0x47, 0x04, 0x0,
			0x0, 0x00, 0xFF }, { 0x81, 0x01, 0x04, 0x47, 0x06, 0x0, 0x0, 0x00, 0xFF } };

	if (level > 2)
		level = 2;

	for ( int i = 0; i < 9; i++ ) {
		serial_write(&sony, z[level][i]);
	}
}

static void c_sony_set_manual_focus(const uint8_t mode, uint8_t spd)
{
	if (spd > 7)
		spd = 7;

	uint8_t cmd[6] = { 0x81, 0x01, 0x04, 0x08, 0, 0xFF };

	if (mode == ZF_IN)
		cmd[4] = 0x20 | spd;
	else if (mode == ZF_OUT)
		cmd[4] = 0x30 | spd;

	for ( int i = 0; i < 6; i++ ) {
		serial_write(&sony, cmd);
	}

}

static void camera_handler()
{
	static Panel_camera_command_t old;
	static uint8_t zLevel = 0;
	static uint8_t af_state = 1;
	uint8_t fSpeed = 0;

	if (serial_available(&sony) > 0) {
#if DEBUG_SONY==1
		printf("%02X ", (uint8_t) serial_read(&sony));
#endif	//if DEBUG_SONY==1
	}

	if (p_cmd.zoom != old.zoom) {
		if (p_cmd.zoom == ZF_IN) {
			if (zLevel < 2) {
				LOGS("zoom IN\r\n");
				c_sony_set_zoom(++zLevel);
			}
		}
		else if (p_cmd.zoom == ZF_OUT) {
			if (zLevel > 0) {
				LOGS("zoom OUT\r\n");
				c_sony_set_zoom(--zLevel);
			}
		}
		else {
			af_state = 1;
			c_sony_set_auto_focus(af_state);
		}

		cam_state.zoomLevel = zLevel;
		old.zoom = p_cmd.zoom;
	}
	else {
		if (p_cmd.focus != old.focus) {
			if (p_cmd.focus == ZF_IN) {
				LOGS("FOCUS FAR start\r\n");
				if (af_state != 0) {
					af_state = 0;
					c_sony_set_auto_focus(af_state);
					HAL_Delay(10);
				}
			}
			else if (p_cmd.focus == ZF_OUT) {
				LOGS("FOCUS NEAR start\r\n");
				if (af_state != 0) {
					af_state = 0;
					c_sony_set_auto_focus(af_state);
					HAL_Delay(10);
				}
			}
			else {
				LOGS("FOCUS stop\r\n");
				af_state = 1;
			}

			fSpeed = (3 - zLevel) * 2;
			c_sony_set_manual_focus(p_cmd.focus, fSpeed);
			old.focus = p_cmd.focus;
		}
	}
}

/* === */
/* BUS */
/* === */
static HAL_StatusTypeDef can_init()
{
	HAL_StatusTypeDef ret = HAL_OK;

	CAN_RX_Filter(bus.hcan, 0, RWS_PANEL_CMD_ID, 0x7FF);

	CAN_Config(bus.hcan);
	CAN_Tx_Config(&bus.txHeader);

	return ret;
}

static HAL_StatusTypeDef bus_init()
{
	bus.hcan = &hcan1;

	/* can tx buffer init */
	bus.tx_opt_lrf.id = RWS_OPTRONIK_LRF_ID;
	bus.tx_opt_lrf.datalength = 8;
	bus.tx_opt_cam.id = RWS_OPTRONIK_CAM_ID;
	bus.tx_opt_cam.datalength = 1;

	/* can rx buffer init */
	bus.rx_panel_command.id = RWS_PANEL_CMD_ID;

	return can_init();
}

static HAL_StatusTypeDef bus_send(Bus_Tx_Buffer_t *buffer)
{
	bus.txHeader.StdId = buffer->id;
	bus.txHeader.DLC = buffer->datalength;

	if (HAL_CAN_GetTxMailboxesFreeLevel(bus.hcan) > 0)
		return HAL_CAN_AddTxMessage(bus.hcan, &bus.txHeader, buffer->data, &bus.txMailBox);
	else
		return HAL_ERROR;
}
/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		if (RxHeader.IDE == CAN_ID_STD) {
			uint32_t _id = RxHeader.StdId;

			/* FROM MOTOR */
			if (_id == RWS_PANEL_CMD_ID) {
				bus.rx_panel_command.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_panel_command.data, RxData, RxHeader.DLC);
				bus.rx_panel_command.len = RxHeader.DLC;
				bus.rx_panel_command.counter++;
			}
		}
	}

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	MX_CAN1_Init();
	bus_init();

	LOG("\r\n=============\r\n");
	LOG("BUS OFF ERROR\r\n");LOG("=============\r\n");
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
