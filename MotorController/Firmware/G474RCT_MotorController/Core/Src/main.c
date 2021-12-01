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
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
//#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#include "Ingenia_FdcanServoDriver.h"
#include "rws_config.h"
#include "bus_fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	MOVEMENT_MANUAL,
	MOVEMENT_STABILIZE,
	MOVEMENT_TRACK
} movement_mode_e;

typedef struct
{
	uint8_t movementMode;
	uint8_t weaponCommand;
	int32_t panSpeedDesired;
	int32_t tiltSpeedDesired;
	int32_t panSpeedCorrection;
	int32_t tiltSpeedCorrection;
	uint32_t timestamp;
} panel_command_t;

typedef struct
{
	uint8_t motorStatus;
	uint8_t weaponStatus;
	uint16_t munitionCounter;
} motor_weapon_status_t;

typedef struct
{
	Servo_t *servo;
	uint32_t maxVelo;
	uint32_t pos1Rev;
} servo_under_test_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PANEL_NOT_CONNECTED		1

#define MTR_AZ_ENABLE			0
#define MTR_EL_ENABLE			1

#define MTR_AZ_ID				0x20
#define MTR_EL_ID				0x21

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Servo_t mtrAzi;
Servo_t mtrEle;
volatile uint32_t countTPDO4 = 0;

Bus_Tx_Buffer_t busSendMotor;
Bus_Rx_Buffer_t busRecvPanel;

panel_command_t g_panel_command;
motor_weapon_status_t g_motor_weapon_status;

volatile uint8_t g_fdcan_bus_busOff_error = 0;
volatile uint8_t g_fdcan_motor_busOff_error = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void busInit(void);
static void busHandler(void);

static void weaponInit();
static void weaponHandler();

static HAL_StatusTypeDef motorInit();
static void motorHandler();
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
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	MX_TIM8_Init();
//	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Init*/
	usartAllInit();
	bufLen = sprintf(buf, "motorControl RWS-mk3 firmware!\r\n");
	serial_write_str(&debug, buf, bufLen);

	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	busInit();

//	HAL_IWDG_Refresh(&hiwdg);
	if (motorInit() != HAL_OK)
		Error_Handler();

	servo_under_test_t test;
#if MTR_AZ_ENABLE==1
	test.servo = &mtrAzi;
	test.maxVelo = 350UL;
	test.pos1Rev = 18UL;
#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
	test.servo = &mtrEle;
	test.maxVelo = 100000UL;
	test.pos1Rev = 4000UL;
#endif	//if MTR_EL_ENABLE==1

	bufLen = sprintf(buf, "mtrEle._u8Node= 0x%02X\r\n", test.servo->_u8Node);
	serial_write_str(&debug, buf, bufLen);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t _debugMotorEnable = 0;
#if PANEL_NOT_CONNECTED==1
	uint32_t panelCommandTimer = 0;
#endif	//if PANEL_NOT_CONNECTED==1

	while (1) {
		uint32_t _debugPos = 0;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/
//		HAL_IWDG_Refresh(&hiwdg);
#if PANEL_NOT_CONNECTED==0
		if (g_fdcan_bus_busOff_error == 1) {
			MX_FDCAN1_Init();
			busInit();
			g_fdcan_bus_busOff_error = 0;
		}
#endif	//if PANEL_NOT_CONNECTED==0

		if (g_fdcan_motor_busOff_error == 1) {
			MX_FDCAN2_Init();
			if (motorInit() != HAL_OK)
				Error_Handler();

			g_fdcan_motor_busOff_error = 0;
		}

#if PANEL_NOT_CONNECTED==0
		busHandler();
#else
		if (HAL_GetTick() >= panelCommandTimer) {
			panelCommandTimer = HAL_GetTick() + 50;

			g_panel_command.timestamp = HAL_GetTick();
			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		}
#endif	//if PANEL_NOT_CONNECTED==0
		motorHandler();

		if (serial_available(&debug) > 0) {
			char c = serial_read(&debug);

			if (c == '0') {
				g_panel_command.tiltSpeedDesired = 0;
				bufLen = sprintf(buf, "motor Stop!\r\n");
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'e') {
				if (bitRead(g_motor_weapon_status.motorStatus, 1)) {
					bitClear(g_panel_command.movementMode, 1);
					bufLen = sprintf(buf, "motor Disable!\r\n");
					serial_write_str(&debug, buf, bufLen);
				}
				else {
					bitSet(g_panel_command.movementMode, 1);
					bufLen = sprintf(buf, "motor Enable!\r\n");
					serial_write_str(&debug, buf, bufLen);
				}
			}
			else if (c == 'p') {
				g_panel_command.tiltSpeedDesired = 100000;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'o') {
				g_panel_command.tiltSpeedDesired = 50000;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'i') {
				g_panel_command.tiltSpeedDesired = 100;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'q') {
				g_panel_command.tiltSpeedDesired = -100000;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'w') {
				g_panel_command.tiltSpeedDesired = -50000;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
			else if (c == 'e') {
				g_panel_command.tiltSpeedDesired = -100;
				bufLen = sprintf(buf, "spd= %ld\r\n", g_panel_command.tiltSpeedDesired);
				serial_write_str(&debug, buf, bufLen);
			}
		}

//		if (serial_available(&debug) > 0) {
//			char c = serial_read(&debug);
//
//			if (c == '0') {
//				Ingenia_setTargetPositionVelocity(test.servo, 0, 0, 1, 0, 1);
//				bufLen = sprintf(buf, "motor Stop!\r\n");
//				serial_write_str(&debug, buf, bufLen);
//			}
//			else if (c == 'e') {
//				if (_debugMotorEnable == 0) {
//					Ingenia_enableMotor(test.servo);
//					bufLen = sprintf(buf, "motor set to enable!\r\n");
//					serial_write_str(&debug, buf, bufLen);
//					_debugMotorEnable = 1;
//				}
//				else {
//					Ingenia_disableMotor(test.servo);
//					bufLen = sprintf(buf, "motor set to disable!\r\n");
//					serial_write_str(&debug, buf, bufLen);
//					_debugMotorEnable = 0;
//				}
//			}
//			else if (c == '<') {
//				if (_debugMotorEnable != 0) {
//					_debugPos = test.servo->posActual - test.pos1Rev;
//					Ingenia_setTargetPositionVelocity(test.servo, _debugPos, test.maxVelo, 1, 0, 0);
//					bufLen = sprintf(buf, "move left! %ld\r\n", _debugPos);
//					serial_write_str(&debug, buf, bufLen);
//				}
//			}
//			else if (c == '>') {
//				if (_debugMotorEnable != 0) {
//					_debugPos = test.servo->posActual + test.pos1Rev;
//					Ingenia_setTargetPositionVelocity(test.servo, _debugPos, test.maxVelo, 1, 0, 0);
//					bufLen = sprintf(buf, "move right! %ld\r\n", _debugPos);
//					serial_write_str(&debug, buf, bufLen);
//				}
//			}
//			else if (c == 'T') {
//				bufLen = sprintf(buf, "\r\n\r\nfdcan1:0x%lX fdcan2:0x%lX uart1=0x%lX\r\n",
//						HAL_FDCAN_GetError(&hfdcan1), HAL_FDCAN_GetError(&hfdcan2),
//						HAL_UART_GetError(&huart1));
//				serial_write_str(&debug, buf, bufLen);
//			}
//		}
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
/* TODO User Functions*/
static void busInit(void)
{
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

	bufLen = sprintf(buf, "fdcan bus Init...\r\n");
	serial_write_str(&debug, buf, bufLen);

	FDCAN_RX_Filter_Panel(&hfdcan1, 0);
	busRecvPanel.id = RWS_PANEL_ID;
	FDCAN_TX_Config(&busSendMotor.txHeader, RWS_MOTOR_ID, RWS_MOTOR_DATA_LENGTH);
	FDCAN_Config(&hfdcan1);

	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
}

void Bus_Notification_Callback(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *data)
{
	if (rxHeader->Identifier == RWS_PANEL_ID) {
		busRecvPanel.lastTimestamp = HAL_GetTick();
		busRecvPanel.rxHeader = *rxHeader;
		memcpy(busRecvPanel.data, data, FDCAN_Convert_Datalength(rxHeader->DataLength));
		busRecvPanel.counter++;
	}
}

static void motorParamInit(Servo_t *servo)
{
	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);

	/* set mode to profile position */
	Ingenia_setModeOfOperation(servo, DRIVE_MODE_PROFILE_POSITION);

	if (servo->_u8Node == MTR_AZ_ID) {
		/* set max motor speed */
		Ingenia_write_sdo_u32(servo, 0x6080, 0, 350UL);
		/* set max velocity profile */
		Ingenia_write_sdo_u32(servo, 0x607F, 0, 350UL);
		/* set max profile acceleration */
		Ingenia_write_sdo_u32(servo, 0x6083, 0, 700UL);
		/* set max profile de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6084, 0, 700UL);
		/* set max profile quick stop de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6085, 0, 1000UL);
	}
	else if (servo->_u8Node == MTR_EL_ID) {
		/* set max motor speed */
		Ingenia_write_sdo_u32(servo, 0x6080, 0, 100000UL);
		/* set max velocity profile */
		Ingenia_write_sdo_u32(servo, 0x607F, 0, 100000UL);
		/* set max profile acceleration */
		Ingenia_write_sdo_u32(servo, 0x6083, 0, 200000UL);
		/* set max profile de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6084, 0, 200000UL);
		/* set max profile quick stop de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6085, 0, 500000UL);
	}

	/* set TPDO4 event timer */
	Ingenia_write_sdo_u16(servo, 0x1803, 5, 10);
}

static HAL_StatusTypeDef motorInit()
{
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

#if MTR_DEBUG_ENABLE==1
	bufLen = sprintf(buf, "motor init...\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if MTR_DEBUG_ENABLE==1

	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
		Error_Handler();
	}

	Ingenia_begin(&hfdcan2);

#if MTR_AZ_ENABLE==1
	if (Ingenia_init(&mtrAzi, MTR_AZ_ID) != HAL_OK) {
#if MTR_DEBUG_ENABLE==1
		bufLen = sprintf(buf, "init MTR_AZI failed!\r\n");
		serial_write_str(&debug, buf, bufLen);
		serial_write_flush(&debug);
#endif	//if MTR_DEBUG_ENABLE==1

		return HAL_ERROR;
	}

#if MTR_DEBUG_ENABLE==1
	bufLen = sprintf(buf, "enable pan motor...");
	serial_write_str(&debug, buf, bufLen);
#endif	//if MTR_DEBUG_ENABLE==1

	motorParamInit(&mtrAzi);
	Ingenia_disableMotor(&mtrAzi);

#if MTR_DEBUG_ENABLE==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if MTR_DEBUG_ENABLE==1

#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
	if (Ingenia_init(&mtrEle, MTR_EL_ID) != HAL_OK) {
#if MTR_DEBUG_ENABLE==1
		bufLen = sprintf(buf, "init MTR_ELE failed!\r\n");
		serial_write_str(&debug, buf, bufLen);
		serial_write_flush(&debug);
#endif	//if MTR_DEBUG_ENABLE==1

		return HAL_ERROR;
	}

#if MTR_DEBUG_ENABLE==1
	bufLen = sprintf(buf, "enable tilt motor...");
	serial_write_str(&debug, buf, bufLen);
#endif	//if MTR_DEBUG_ENABLE==1

	motorParamInit(&mtrEle);
	Ingenia_disableMotor(&mtrEle);

#if MTR_DEBUG_ENABLE==1
	bufLen = sprintf(buf, "done!\r\n");
	serial_write_str(&debug, buf, bufLen);
#endif	//if MTR_DEBUG_ENABLE==1
#endif	//if MTR_EL_ENABLE==1

	return HAL_OK;
}

void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;
	uint32_t idNode = 0;
	int _pos = 0, _velo = 0;

	if (can_buffer_available(buffer) > 0) {
		can_buffer_read(buffer, &data);
		countTPDO4++;
		if ((data.canRxHeader.Identifier & COB_TPDO4) == COB_TPDO4) {

			idNode = data.canRxHeader.Identifier - COB_TPDO4;
			for ( int i = 0; i < 4; i++ ) {
				_pos |= (int) data.rxData[i] << (8 * i);
				_velo |= (int) data.rxData[i + 4] << (8 * i);
			}

			if (idNode == mtrAzi._u8Node) {
				mtrAzi.posActual = _pos;
				mtrAzi.veloActual = _velo;
			}
			else if (idNode == mtrEle._u8Node) {
				mtrEle.posActual = _pos;
				mtrEle.veloActual = _velo;
			}

		}

	}
}

static void motorHandler()
{
	static uint8_t debugDisplayCounter = 0;
	static panel_command_t _prev_command = { 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0 };
	bool _bool;
	int32_t _posRel;
	uint32_t _veloRel;
	Rws_Union_u _veloCmd;
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

	if (g_panel_command.timestamp > _prev_command.timestamp) {
		if (g_panel_command.movementMode != _prev_command.movementMode) {

#if MTR_AZ_ENABLE==1
			_bool = bitRead(g_panel_command.movementMode, 0);
			if (_bool != bitRead(_prev_command.movementMode, 0)) {
				if (_bool)
					/* ENABLE PAN MOTOR */
					Ingenia_enableMotor(&mtrAzi);
				else
					/* DISABLE PAN MOTOR */
					Ingenia_disableMotor(&mtrAzi);
			}
	#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
			_bool = bitRead(g_panel_command.movementMode, 1);
			if (_bool != bitRead(_prev_command.movementMode, 1)) {
				if (_bool) {
					/* ENABLE TILT MOTOR */
					Ingenia_enableMotor(&mtrEle);
					bitSet(g_motor_weapon_status.motorStatus, 1);
				}
				else {
					/* DISABLE TILT MOTOR */
					Ingenia_disableMotor(&mtrEle);
					bitClear(g_motor_weapon_status.motorStatus, 1);
				}
			}
#endif	//if MTR_EL_ENABLE==1

			_prev_command.movementMode = g_panel_command.movementMode;
		}

#if MTR_AZ_ENABLE==1

	#endif	//if MTR_AZ_ENABLE==1

#if MTR_EL_ENABLE==1
		/* if motor enable */
		if (bitRead(_prev_command.movementMode, 1)) {
			_veloCmd.i32 = g_panel_command.tiltSpeedDesired;
			_veloRel = labs(_veloCmd.i32);
			_posRel = mtrEle.posActual + (_veloCmd.i32 / 10);
			/* TODO convert velocity to c/s */

			Ingenia_setTargetPositionVelocity(&mtrEle, _posRel, _veloRel, 1, 0, 0);

			if (++debugDisplayCounter > 20) {
				bufLen = sprintf(buf, "[%ld]p:v(cmd)=%ld:%ld\t(act)p:v=%ld:%ld\r\n", HAL_GetTick(),
						_posRel, _veloRel, mtrEle.posActual, mtrEle.veloActual);
				if (ring_buffer_free(&debug.TBufferTx) > bufLen)
					serial_write_str(&debug, buf, bufLen);
				debugDisplayCounter = 0;
			}

			_prev_command.tiltSpeedDesired = g_panel_command.tiltSpeedDesired;
		}
#endif	//if MTR_EL_ENABLE==1

		_prev_command.timestamp = g_panel_command.timestamp;
	}
}

static void busHandler(void)
{
	static uint8_t panel_counter = 0;
	static uint32_t motorSendTimer = 0;
	Rws_Union_u _pan_pos, _pan_velo;
	Rws_Union_u _tilt_pos, _tilt_velo;
	Rws_Union_u _cmd[4];

	if (panel_counter != busRecvPanel.counter) {
		panel_counter = busRecvPanel.counter;

		g_panel_command.timestamp = busRecvPanel.lastTimestamp;
		g_panel_command.movementMode = busRecvPanel.data[0];
		g_panel_command.weaponCommand = busRecvPanel.data[2];

		for ( int _i = 0; _i < 4; _i++ ) {
			_cmd[0].u8[_i] = busRecvPanel.data[4 + _i];
			_cmd[1].u8[_i] = busRecvPanel.data[8 + _i];
			_cmd[2].u8[_i] = busRecvPanel.data[12 + _i];
			_cmd[3].u8[_i] = busRecvPanel.data[16 + _i];
		}
		g_panel_command.panSpeedDesired = _cmd[0].i32;
		g_panel_command.tiltSpeedDesired = _cmd[1].i32;
		g_panel_command.panSpeedCorrection = _cmd[2].i32;
		g_panel_command.tiltSpeedCorrection = _cmd[3].i32;

		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
	}

	if (HAL_GetTick() >= motorSendTimer) {
		motorSendTimer = HAL_GetTick() + 50;

		busSendMotor.data[0] = g_motor_weapon_status.motorStatus;
		busSendMotor.data[1] = g_motor_weapon_status.weaponStatus;
		busSendMotor.data[2] = g_motor_weapon_status.munitionCounter & 0xFF;
		busSendMotor.data[3] = (g_motor_weapon_status.munitionCounter >> 8) & 0xFF;

		_pan_pos.i32 = mtrAzi.posActual;
		_pan_velo.i32 = mtrAzi.veloActual;
		_tilt_pos.i32 = mtrEle.posActual;
		_tilt_velo.i32 = mtrEle.veloActual;

		for ( int _index = 0; _index < 4; _index++ ) {
			busSendMotor.data[4 + _index] = _pan_pos.u8[_index];
			busSendMotor.data[8 + _index] = _pan_velo.u8[_index];
			busSendMotor.data[12 + _index] = _tilt_pos.u8[_index];
			busSendMotor.data[16 + _index] = _tilt_velo.u8[_index];
		}

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &busSendMotor.txHeader, busSendMotor.data);
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan->Instance == FDCAN1)
		g_fdcan_bus_busOff_error = 1;
	else if (hfdcan->Instance == FDCAN2)
		g_fdcan_motor_busOff_error = 1;
}
/* TODO END user functions*/
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
