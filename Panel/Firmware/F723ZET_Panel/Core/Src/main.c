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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "bus_can.h"
#include "bus_button_config.h"

#include "sla.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
	uint8_t deadman :1; /* JSR or JSL to activate */
	uint8_t trigger :1; /* JSR for firing, JSL for balistic compensation */
	uint8_t aButton :1; /* JSL for track mode, JSR for travel mode */
	uint8_t bButton :1; /* JSL for memory mode, JSR for stabilize mode */
	uint8_t reservedBit :4;
} __attribute__ ((packed)) TJoystickButton;

typedef struct
{
	uint16_t raw_min;
	uint16_t raw_mid;
	uint16_t raw_max;

	int32_t az; /* JSR for manual azimuth speed or azimuth correction speed, JSL for track size (right->bigger, left->smaller)  */
	uint32_t az_max; /* maximum value of az */
	int32_t az_tem; /* used for smoothing */
	int32_t el; /* JSR for manual elevation speed or elevation correction speed, JSL for zoom action (up->tele, down->wide)  */
	uint32_t el_max; /* maximum value of el */
	int32_t el_tem; /* used for smoothing */
	float alpha;
	float betha;
	TJoystickButton button; /* state of joystick's buttons */
} TJoystick;

typedef struct
{
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailBox;

	Bus_Tx_Buffer_t tx_command;
	Bus_Tx_Buffer_t tx_manual;
	Bus_Tx_Buffer_t tx_track;
	Bus_Tx_Buffer_t tx_balistik;
	Bus_Tx_Buffer_t tx_homing;

	Bus_Rx_Buffer_t rx_motor_state;
	Bus_Rx_Buffer_t rx_motor_position;
	Bus_Rx_Buffer_t rx_motor_imu;

	Bus_Rx_Buffer_t rx_opt_lrf;
	Bus_Rx_Buffer_t rx_opt_cam;
} Bus_t;

typedef struct
{
	uint8_t motor_state :1;
	uint8_t motor_pos :1;
	uint8_t motor_imu :1;
	uint8_t opt_cam_state :1;
	uint8_t opt_lrf :1;
	uint16_t reservedBit :11;
} __attribute__ ((packed)) led_state_t;

typedef struct
{
	uint8_t counter;
	int32_t az;  // 0-360000 (1/1000 degree)
	int32_t el;  // in 1/1000 degree
	uint16_t distance;
} Homing_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_ENABLE			1
#if DEBUG_ENABLE==1

#define DEBUG_BUS				1
#define DEBUG_BUTTON			1
#define DEBUG_SLA				1

#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)

#if DEBUG_BUS==1
#define LOGBUS(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOGBUS_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOGBUS(str, ...)
#define LOGBUS_E(str, ...)
#endif	//if DEBUG_BUS==1

#if DEBUG_BUTTON==1
#define LOGBTN(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOGBTN_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOGBTN(str, ...)
#define LOGBTN_E(str, ...)
#endif	//if DEBUG_BUTTON==1

#if DEBUG_SLA==1
#define LOGSLA(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick()%10000, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#define LOGSLA_E(str, ...) printf("[%s Err:%d] " str,  __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#define LOGSLA(str, ...)
#define LOGSLA_E(str, ...)
#endif	//if DEBUG_SLA==1

#else
#define LOG(str, ...)
#define LOG_E(str, ...)
#endif	//if DEBUG_ENABLE==1

#define INPUT_PORTD		GPIOD
#define INPUT_PORTG		GPIOG

#define MTR_MAX_SPEED			1000000UL
#define MTR_MID_SPEED			100000UL
#define MTR_LOW_SPEED			5000UL

#define RWS_MOTOR_PAN_FULL_REV_IN_C			417747
#define RWS_MOTOR_TILT_C_TO_DEG(c)			((float)c*0.000525247f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Bus_t bus;

//sModeGpioInput_t bInput;
//sModeGpioOutput_t bOutput;
TJoystick JRight, JLeft;
uint32_t adcVal[5];
volatile uint8_t adcConvCompleted = 0;
volatile uint8_t can_completed = 0;

static Panel_motor_mode_t p0;
static Panel_weapon_command_t p1;
static uint16_t shoot_limit = 0;
static Panel_camera_command_t p4;
static Panel_lrf_imu_command_t p5;

volatile uint32_t trig_timer = 0;
static led_state_t led_state;
static uint8_t rws_mode = MOVE_MODE_MAN;
static Homing_t homing_command = { .counter = 0, .az = 0, .el = 0, .distance = 0 };
static uint16_t lrf_val = 1000;
static float body_rws_pos[2] = { 0, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef can_init();
static HAL_StatusTypeDef bus_init();
static HAL_StatusTypeDef bus_send(Bus_Tx_Buffer_t *buffer);

static void button_init();
static void button_update();
static void js_update();

static void led_init();
static void led_handler();

static void move_handler();
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
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_TIM5_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	/* TODO INIT */
	led_init();

	uart_init_all();
	LOG("Panel Firmware!\r\n");

	bus_init();
	button_init();
	sla_init();

	sla_set_distance(lrf_val);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t adc_timer = 0;
	uint8_t motor_state_counter = 0;
	uint8_t motor_position_counter = 0;
	uint8_t opt_cam_counter = 0;
	uint32_t trig_out_timer = 1;
	while (1) {
		char c;
		Bus_Rx_Buffer_t tem;
		Rws_Union_u ru1, ru2;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP */

		HAL_IWDG_Refresh(&hiwdg);

		if (trig_timer != 0) {
			if (HAL_GetTick() >= trig_timer) {
				HAL_GPIO_WritePin(TRIG_PULSE_OUT_GPIO_Port, TRIG_PULSE_OUT_Pin, GPIO_PIN_RESET);

				trig_timer = 0;
			}
		}

		if (p1.triggerEnable == 1 && (JRight.button.deadman || JLeft.button.deadman))
			HAL_GPIO_WritePin(TRIG_PULSE_OUT_GPIO_Port, TRIG_PULSE_OUT_Pin,
					HAL_GPIO_ReadPin(JSR_TRIG_GPIO_Port, JSR_TRIG_Pin));

		if (serial_available(&debug) > 0) {
			c = serial_read(&debug);
			if (c == 't') {
				LOG("start continous trigering\r\n");
				if (trig_out_timer != 0)
					trig_out_timer = 0;
				else
					trig_out_timer = HAL_GetTick() + 500;
			}
			else if (c == 'm') {
				p1.munitionCounterReset = !p1.munitionCounterReset;
				LOG("munisi counter reset=%d\r\n", p1.munitionCounterReset);
			}
		}

		if (adcConvCompleted != 0) {
			adcConvCompleted = 0;

			LOGBTN("adc dT= %ld\r\n", HAL_GetTick() - adc_timer);
			adc_timer = HAL_GetTick();

			js_update();
			button_update();

			/* TODO change move mode  */
			p0.moveMode = rws_mode;
			bus.tx_command.data[0] = *(uint8_t*) &p0;
			bus.tx_command.data[1] = *(uint8_t*) &p1;
			bus.tx_command.data[2] = shoot_limit >> 8;
			bus.tx_command.data[3] = shoot_limit & 0xFF;
			bus.tx_command.data[4] = *(uint8_t*) &p4;
			bus.tx_command.data[5] = *(uint8_t*) &p5;
			if (bus_send(&bus.tx_command) == HAL_OK) {
				LOGBUS("cmd:[M%02X][C%02X]\r\n", bus.tx_command.data[0], bus.tx_command.data[4]);
			}

		}

		if (bus.rx_motor_state.counter != motor_state_counter) {
			motor_state_counter = bus.rx_motor_state.counter;

			tem = bus.rx_motor_state;
			LOGBUS("m_state: ");
#if DEBUG_BUS==1
			for ( int i = 0; i < tem.len; i++ ) {
				printf("%02X ", tem.data[i]);
			}
			printf("\r\n");
#endif	//if DEBUG_BUS==1

			if (rws_mode == MOVE_MODE_HOMING) {
				Body_motor_mode_t i0;
				*(uint8_t*) &i0 = tem.data[0];

				if (i0.moveModeEnded == 1)
					rws_mode = MOVE_MODE_MAN;
			}

			led_state.motor_state = 1;
		}

		if (bus.rx_motor_position.counter != motor_position_counter) {
			motor_position_counter = bus.rx_motor_position.counter;

			tem = bus.rx_motor_position;
			for ( int i = 0; i < 4; i++ ) {
				ru1.u8[i] = tem.data[i];
				ru2.u8[i] = tem.data[i + 4];
			}
			LOGBUS("pan= %ld,tilt= %ld\r\n", ru1.i32, ru2.i32);

			ru1.i32 %= RWS_MOTOR_PAN_FULL_REV_IN_C;
			body_rws_pos[0] = (float) ru1.i32 * 360.0f / RWS_MOTOR_PAN_FULL_REV_IN_C;
			if (body_rws_pos[0] < 0)
				body_rws_pos[0] += 360.0f;
			body_rws_pos[1] = (float) RWS_MOTOR_TILT_C_TO_DEG(ru2.i32);

			sla_set_attitude(body_rws_pos[0], body_rws_pos[1]);
		}

		if (bus.rx_opt_cam.counter != opt_cam_counter) {
			opt_cam_counter = bus.rx_opt_cam.counter;

			tem = bus.rx_opt_cam;
			LOGBUS("opt_cam: %02X\r\n", tem.data[0]);

			led_state.opt_cam_state = 1;
		}

		led_handler();
		sla_handler();
		move_handler();

		/* TODO END LOOP */
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* TODO User Functions */

static void manual_handler()
{
	Rws_Union_u p, t;
	static uint32_t _send_timer = 0;

	if (HAL_GetTick() >= _send_timer) {
		_send_timer = HAL_GetTick() + 50;

		if (rws_mode == MOVE_MODE_MAN) {
			if (p0.motorEnable == 1) {
				p.i32 = JRight.az;
				t.i32 = JRight.el;
			}
			else {
				p.i32 = 0;
				t.i32 = 0;
			}

			for ( int i = 0; i < 4; i++ ) {
				bus.tx_manual.data[i] = p.u8[i];
				bus.tx_manual.data[i + 4] = t.u8[i];
			}

			if (bus_send(&bus.tx_manual) == HAL_OK) {
//				LOG("manual=%ld,%ld\r\n", p.i32, t.i32);
			}
		}
	}
}

static void homing_handler()
{
	static uint8_t gsl_counter = 0xFF;
	static uint32_t homing_send_timer = 0;
	static uint8_t homing_button = 0;
	uint8_t _button = 0;
	Rws_Union_u a, e;

	if ((JRight.button.deadman | JLeft.button.deadman) && JLeft.button.bButton)
		_button = 1;

	if (sla_gsl_available() > 0) {
		sla_gsl_get_command(&homing_command.counter, &homing_command.az, &homing_command.el, &homing_command.distance);
//		if (rws_mode == MOVE_MODE_MAN)
//			_button = 1;
	}

	if (homing_button != _button) {
		if (_button == 1) {
			if (rws_mode == MOVE_MODE_MAN) {
				if (gsl_counter != homing_command.counter) {
					gsl_counter = homing_command.counter;

					a.i32 = homing_command.az;
					e.i32 = homing_command.el;

					lrf_val = homing_command.distance;
					sla_set_distance(lrf_val);
					sla_gsl_start();
				}
				else {
					a.i32 = 0;
					e.i32 = 0;
				}

				for ( int i = 0; i < 4; i++ ) {
					bus.tx_homing.data[i] = a.u8[i];
					bus.tx_homing.data[i + 4] = e.u8[i];
				}

				LOGSLA("HOMING STARTED!\r\nHoming=%ld,%ld,%d\r\n", homing_command.az, homing_command.el,
						homing_command.distance);

				p0.moveMode = MOVE_MODE_HOMING;
				p0.motorEnable = 1;
				homing_send_timer = HAL_GetTick();
				rws_mode = MOVE_MODE_HOMING;
			}
			else if (rws_mode == MOVE_MODE_HOMING) {
				/* abort */
				LOGSLA("rws_mode=MANUAL (aborted!)\r\n");
				rws_mode = MOVE_MODE_MAN;
			}
		}
		homing_button = _button;
	}

	if (rws_mode == MOVE_MODE_MAN)
		homing_send_timer = 0;

	if (homing_send_timer > 0) {
		if (HAL_GetTick() >= homing_send_timer) {
			homing_send_timer = HAL_GetTick() + 100;

			bus_send(&bus.tx_homing);
			LOGSLA("body pos=%.1f,%.3f\r\n", body_rws_pos[0], body_rws_pos[1]);
		}
	}

}

static void move_handler()
{
	static uint32_t _update_sla_timer = 0;
	uint16_t bufLen = 0;
	char buf[RING_BUFFER_TX_SIZE];

	manual_handler();
	homing_handler();

	if (HAL_GetTick() >= _update_sla_timer) {
		_update_sla_timer = HAL_GetTick() + 1000;

		if (rws_mode == MOVE_MODE_MAN) {
			bufLen = sprintf(buf, "$DISPSTR,1,5,30,MANUAL MODE*");
			serial_write_str(&pcE, buf, bufLen);
		}
		else if (rws_mode == MOVE_MODE_HOMING) {
			bufLen = sprintf(buf, "$DISPSTR,1,5,30,HOMING MODE*");
			serial_write_str(&pcE, buf, bufLen);
		}
	}
}

static uint8_t led_check_connection(const uint32_t *timestamp)
{
	if (HAL_GetTick() >= *timestamp + BUS_MAX_TIMEOUT)
		return 0;
	else
		return 1;
}

static void led_init()
{
	*(uint16_t*) &led_state = 0;
}

static void led_handler()
{
	static uint32_t _led_timer = 1000;
	static uint32_t _led_counter = 0;

	led_state.motor_state = led_check_connection(&bus.rx_motor_state.lastTimestamp);
	led_state.motor_pos = led_check_connection(&bus.rx_motor_position.lastTimestamp);
	led_state.motor_imu = led_check_connection(&bus.rx_motor_imu.lastTimestamp);
	led_state.opt_cam_state = led_check_connection(&bus.rx_opt_cam.lastTimestamp);
	led_state.opt_lrf = led_check_connection(&bus.rx_opt_lrf.lastTimestamp);

	if (HAL_GetTick() >= _led_timer) {
		_led_timer = HAL_GetTick() + 100;

		if (_led_counter % 10 < 1) {
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
		}
		else if (_led_counter % 10 < 5) {
			if (led_state.motor_state == 1)
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		}
		else if (_led_counter % 10 < 9) {
			if (led_state.opt_cam_state == 1)
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		}
		else
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

		if (++_led_counter > 10)
			_led_counter = 0;
	}

}

static void button_init()
{
	*(uint8_t*) &p0 = 0;
	*(uint8_t*) &p1 = 0;
	*(uint8_t*) &p4 = 0;
	*(uint8_t*) &p5 = 0;

	JRight.raw_mid = 950;
	JRight.raw_min = 740;
	JRight.raw_max = 1150;
	JLeft.raw_mid = 1300;
	JLeft.raw_min = 1000;
	JLeft.raw_max = 1600;

	*(uint8_t*) &JRight.button = 0;
	*(uint8_t*) &JLeft.button = 0;
	JRight.az = 0;
	JRight.az_tem = 0;
	JRight.el = 0;
	JRight.el_tem = 0;
	JLeft.az = 0;
	JLeft.az_tem = 0;
	JLeft.el = 0;
	JLeft.el_tem = 0;
	JRight.az_max = MTR_MID_SPEED;
	JRight.el_max = MTR_MID_SPEED;
	JRight.alpha = 0.2f;
	JRight.betha = 1.0f - JRight.alpha;
	JLeft.az_max = 1000UL;
	JLeft.el_max = 1000UL;
	JLeft.alpha = 1.0f;
	JLeft.betha = 1.0f - JLeft.alpha;

	/* enable timer2 */
	HAL_TIM_Base_Start(&htim5);
	/* start ADC as DMA */
	HAL_ADC_Start_DMA(&hadc1, adcVal, 5);
}

static GPIO_PinState button_read(const uint32_t port, const uint16_t pin)
{
	if ((port & pin) != (uint32_t) GPIO_PIN_RESET)
		return GPIO_PIN_SET;
	else
		return GPIO_PIN_RESET;
}

static void button_update()
{
	uint32_t portD = GPIOD->IDR;
	uint32_t portG = GPIOG->IDR;

	LOGBTN("portD= %02lX; portG=%02lX\r\n", (portD & 0xFF), (portG >> 8));

	p1.triggerEnable = !button_read(portD, IN_TRIGGER_ENABLE_Pin);

	if (button_read(portG, IN_FIRE_MODE0_Pin) == GPIO_PIN_RESET)
		shoot_limit = FIRING_MODE_1;
	else if (button_read(portD, IN_FIRE_MODE1_Pin) == GPIO_PIN_RESET)
		shoot_limit = FIRING_MODE_CONT;
	else
		shoot_limit = FIRING_MODE_3;

	if (button_read(portG, IN_CAMERA_SELECT_Pin) == GPIO_PIN_RESET)
		p4.cameraActive = CAM_SELECT_THERMAL;
	else
		p4.cameraActive = CAM_SELECT_DAY;

	if (button_read(portD, IN_SPD0_Pin) == GPIO_PIN_RESET)
		JRight.az_max = JRight.el_max = MTR_LOW_SPEED;
	else if (button_read(portD, IN_SPD1_Pin) == GPIO_PIN_RESET)
		JRight.az_max = JRight.el_max = MTR_MAX_SPEED;
	else
		JRight.az_max = JRight.el_max = MTR_MID_SPEED;

	if (button_read(portD, IN_FOCUS_FAR_Pin) == GPIO_PIN_RESET)
		p4.focus = ZF_IN;
	else if (button_read(portG, IN_FOCUS_NEAR_Pin) == GPIO_PIN_RESET)
		p4.focus = ZF_OUT;
	else
		p4.focus = ZF_STOP;

	p5.lrfEnable = !button_read(portG, IN_LRF_ENABLE_Pin);
	p5.lrfStart = !button_read(portD, IN_LRF_START_Pin);

//	if (button_read(portD, IN_LRF_MAN_UP_Pin) == GPIO_PIN_RESET)
//		bInput.lrf_man_mode = LRF_MAN_UP;
//	else if (button_read(portG, IN_LRF_MAN_DOWN_Pin) == GPIO_PIN_RESET)
//		bInput.lrf_man_mode = LRF_MAN_DOWN;
//	else
//		bInput.lrf_man_mode = LRF_MAN_NONE;
//
//	if (button_read(portD, IN_TARGET_NEXT_Pin) == GPIO_PIN_RESET)
//		bInput.target_select = TARGET_NEXT;
//	else if (button_read(portG, IN_TARGET_PREV_Pin) == GPIO_PIN_RESET)
//		bInput.target_select = TARGET_PREV;
//	else
//		bInput.target_select = TARGET_NONE;

//	/* TODO in_resv as cock start */
//	if (button_read(portG, IN_RESV_Pin) == GPIO_PIN_RESET) {
//		if (p1.triggerEnable == 0) {
//			p1.cockEnable = ENABLE;
//			p1.cockStartMoving = ENABLE;
//		}
//		else {
//			p1.cockEnable = DISABLE;
//			p1.cockStartMoving = DISABLE;
//		}
//	}
//	else {
//		p1.cockEnable = DISABLE;
//		p1.cockStartMoving = DISABLE;
//	}

	/* TODO in_resv as sla_enable/disable */
	HAL_GPIO_WritePin(SLA_POWER_ENABLE_GPIO_Port, SLA_POWER_ENABLE_Pin, HAL_GPIO_ReadPin(IN_RESV_GPIO_Port, IN_RESV_Pin));
}

static void js_button_update()
{
	JRight.button.deadman = HAL_GPIO_ReadPin(JSR_DEADMAN_GPIO_Port, JSR_DEADMAN_Pin);
	JRight.button.trigger = HAL_GPIO_ReadPin(JSR_TRIG_GPIO_Port, JSR_TRIG_Pin);
	JRight.button.aButton = HAL_GPIO_ReadPin(JSR_A_GPIO_Port, JSR_A_Pin);
	JRight.button.bButton = HAL_GPIO_ReadPin(JSR_B_GPIO_Port, JSR_B_Pin);

	JLeft.button.deadman = HAL_GPIO_ReadPin(JSL_DEADMAN_GPIO_Port, JSL_DEADMAN_Pin);
	JLeft.button.trigger = HAL_GPIO_ReadPin(JSL_TRIG_GPIO_Port, JSL_TRIG_Pin);
	JLeft.button.aButton = HAL_GPIO_ReadPin(JSL_A_GPIO_Port, JSL_A_Pin);
	JLeft.button.bButton = HAL_GPIO_ReadPin(JSL_B_GPIO_Port, JSL_B_Pin);

	uint8_t _deadman = JRight.button.deadman | JLeft.button.deadman;
	p0.motorEnable = _deadman;
}

static long constrain(long x, const long min, const long max)
{
	if (x < min)
		x = min;
	else if (x > max)
		x = max;

	return x;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int32_t adc_joystick_azel_conversion(const long min, const long mid, const long max, const uint32_t adcVal,
		const uint32_t maxValue)
{
	const long JS_ADC_HYST = 10;

	int _i;
	float _f = 0.0f;
	long in_min = min - mid;
	long in_max = max - mid;
	const int32_t out_min = -900; /* -90.0 degrees */
	const int32_t out_max = 900; /* 90.0 degrees */
	long _l;
	int32_t ret = 0;
	uint32_t u32;

	_l = adcVal - mid;
	if (labs(_l) < JS_ADC_HYST)
		_l = 0;
//	_i = (int) constrain(map(_l, in_min, in_max, out_min, out_max), out_min, out_max);

	if (_l < 0)
		_i = (int) constrain(map(_l, in_min, 0, out_min, 0), out_min, 0);
	else
		_i = (int) constrain(map(_l, 0, in_max, 0, out_max), 0, out_max);

	u32 = abs(_i);
	if (u32 > 100) {
		/* if below 30 degrees */
		if (u32 < 300)
			_i /= 5;
		_f = (float) _i / 10.0f;
		ret = (int32_t) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
		if (ret >= (maxValue * 90 / 100))
			ret = maxValue;
	}

	if (_f < 0)
		ret = 0 - ret;

	return ret;
}

static void adc_joystick_conversion(TJoystick *js, const uint32_t adc_x, const uint32_t adc_y)
{
//	js->az = adc_joystick_azel_conversion(js->raw_min, js->raw_mid, js->raw_max, adc_x, js->az_max);
//	js->el = adc_joystick_azel_conversion(js->raw_min, js->raw_mid, js->raw_max, adc_y, js->el_max);
	js->az_tem = adc_joystick_azel_conversion(js->raw_min, js->raw_mid, js->raw_max, adc_x, js->az_max);
	js->el_tem = adc_joystick_azel_conversion(js->raw_min, js->raw_mid, js->raw_max, adc_y, js->el_max);
	js->az = js->az_tem * js->alpha + js->az * js->betha;
	js->el = js->el_tem * js->alpha + js->el * js->betha;
}

static void js_update()
{
	const int32_t JLEFT_LOW_VAL = 400;
	const int32_t JLEFT_HIGH_VAL = 600;
	int32_t _i32;

	js_button_update();

	LOGBTN("adcVal=%ld,%ld,%ld,%ld\r\n", adcVal[0], adcVal[1], adcVal[2], adcVal[3]);

	adc_joystick_conversion(&JRight, adcVal[0], adcVal[1]);
	adc_joystick_conversion(&JLeft, adcVal[2], adcVal[3]);
	/* TODO JOYSTICK LEFT is broken */


	LOGBTN("JSR=%02X,%ld,%ld JSL=%02X,%ld,%ld\r\n", *(uint8_t* ) &JRight.button, JRight.az, JRight.el,
			*(uint8_t* ) &JLeft.button, JLeft.az, JLeft.el);

	/* ZOOM command */
	_i32 = abs(JLeft.el);
	if (p4.zoom == ZF_STOP) {
		if (_i32 >= JLEFT_HIGH_VAL) {
			if (JLeft.el > 0)
				p4.zoom = ZF_IN;
			else
				p4.zoom = ZF_OUT;
		}
	}
	else {
		if (_i32 <= JLEFT_LOW_VAL)
			p4.zoom = ZF_STOP;
	}

	/* TRACK GATE resize command */
//	_i32 = abs(JLeft.az);
//	if (comp.send.state.trackerGateResize == PC_TRK_GATE_STOP) {
//		if (_i32 >= jLeftHighVal) {
//			if (JLeft.az > 0)
//				comp.send.state.trackerGateResize = PC_TRK_GATE_BIGGER;
//			else
//				comp.send.state.trackerGateResize = PC_TRK_GATE_SMALLER;
//		}
//	}
//	else {
//		if (_i32 <= jLeftLowVal)
//			comp.send.state.trackerGateResize = PC_TRK_GATE_STOP;
//	}
}

static HAL_StatusTypeDef can_init()
{
	HAL_StatusTypeDef ret = HAL_ERROR;

	CAN_RX_Filter_Motor(bus.hcan, 0);
	CAN_RX_Filter_Optronik(bus.hcan, 1);

	CAN_Config(bus.hcan);
	CAN_Tx_Config(&bus.txHeader);

	return ret;
}

static HAL_StatusTypeDef bus_init()
{
	bus.hcan = &hcan1;

	/* can tx buffer init */
	bus.tx_command.id = RWS_PANEL_CMD_ID;
	bus.tx_command.datalength = 6;
	bus.tx_manual.id = RWS_PANEL_MAN_ID;
	bus.tx_track.id = RWS_PANEL_TRK_ID;
	bus.tx_balistik.id = RWS_PANEL_BAL_ID;
	bus.tx_homing.id = RWS_PANEL_HOM_ID;
	bus.tx_manual.datalength = bus.tx_track.datalength = bus.tx_balistik.datalength = bus.tx_homing.datalength = 8;

	/* can rx buffer init */
	bus.rx_motor_state.id = RWS_MOTOR_STATUS_ID;
	bus.rx_motor_position.id = RWS_MOTOR_POS_ID;
	bus.rx_motor_imu.id = RWS_MOTOR_IMU_ID;
	bus.rx_opt_lrf.id = RWS_OPTRONIK_LRF_ID;
	bus.rx_opt_cam.id = RWS_OPTRONIK_CAM_ID;

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
//			LOGBUS("%03X\r\n", _id);
			/* FROM MOTOR */
			if (_id == RWS_MOTOR_STATUS_ID) {
				bus.rx_motor_state.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_motor_state.data, RxData, RxHeader.DLC);
				bus.rx_motor_state.idType = CAN_ID_STD;
				bus.rx_motor_state.len = RxHeader.DLC;
				bus.rx_motor_state.counter++;
			}
			else if (_id == RWS_MOTOR_POS_ID) {
				bus.rx_motor_position.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_motor_position.data, RxData, RxHeader.DLC);
				bus.rx_motor_position.idType = CAN_ID_STD;
				bus.rx_motor_position.len = RxHeader.DLC;
				bus.rx_motor_position.counter++;
			}
			else if (_id == RWS_MOTOR_IMU_ID) {
				bus.rx_motor_imu.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_motor_imu.data, RxData, RxHeader.DLC);
				bus.rx_motor_imu.idType = CAN_ID_STD;
				bus.rx_motor_imu.len = RxHeader.DLC;
				bus.rx_motor_imu.counter++;
			}
			/* FROM OPTRONIK */
			else if (_id == RWS_OPTRONIK_LRF_ID) {
				bus.rx_opt_lrf.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_opt_lrf.data, RxData, RxHeader.DLC);
				bus.rx_opt_lrf.idType = CAN_ID_STD;
				bus.rx_opt_lrf.len = RxHeader.DLC;
				bus.rx_opt_lrf.counter++;
			}
			else if (_id == RWS_OPTRONIK_CAM_ID) {
				bus.rx_opt_cam.lastTimestamp = HAL_GetTick();
				memcpy(bus.rx_opt_cam.data, RxData, RxHeader.DLC);
				bus.rx_opt_cam.idType = CAN_ID_STD;
				bus.rx_opt_cam.len = RxHeader.DLC;
				bus.rx_opt_cam.counter++;
			}
		}
	}

}

/**
 * @brief  Error CAN callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	/* Bus-off error */
	if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0) {
		//	g_fdcan_bus_busOff_error = 1;
#if DEBUG_BUS==1
		printf("\t\t\tbus error!\r\n");
#endif	//if DEBUG_BUS==1
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hadc);
	adcConvCompleted = 1;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == DUAL_TRIG_IN_Pin) {
//		if (p1.triggerEnable == 1 && (JRight.button.deadman | JLeft.button.deadman)) {
//			HAL_GPIO_WritePin(TRIG_PULSE_OUT_GPIO_Port, TRIG_PULSE_OUT_Pin, GPIO_PIN_SET);
//			trig_timer = HAL_GetTick() + 500;
//		}
//	}
//}

/* TODO END User Function */
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

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
