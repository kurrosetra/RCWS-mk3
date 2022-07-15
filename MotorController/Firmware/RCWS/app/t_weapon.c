/*
 * weapon.c
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#include "common.h"
#include "driver/bus_fdcan/bus_fdcan.h"
#include "hal/weapon/kokang.h"
#include "hal/weapon/trigger.h"
#include "hal/sensor/munisi.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_WEAPON==1
#include <stdio.h>
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TWpn:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TWpn_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_WEAPON==1

typedef enum
{
	WS_IDLE,
	WS_COCK_ACTIVE,
	WS_TRIGGER_ACTIVE,
} Weapon_State_e;

typedef struct
{
	uint8_t enable;
	uint8_t start;
	uint16_t shoot_limit;
	uint32_t s_timeout;
	uint32_t h_timeout;
} Trigger_t;

typedef struct
{
	Weapon_State_e state;
	Cock_t cock;
	Trigger_t trig;
} Weapon_t;
Weapon_t weapon;

static void weapon_send_to_bus();

void tim_weapon_callback(void const *argument)
{
	/* USER CODE BEGIN tim_weapon_callback */
	(void) argument;

	HAL_StatusTypeDef ret;

	if (weapon.state == WS_COCK_ACTIVE) {
		ret = cock_handler(&weapon.cock);
		if (ret == HAL_OK || ret == HAL_ERROR)
			osTimerStop(mtr_get_timer(Tim_Weapon_id));
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
	}
	else if (weapon.state == WS_TRIGGER_ACTIVE) {
//		if (osKernelSysTick() >= weapon.trig.s_timeout)
//			trig_s_stop();
//		else if (osKernelSysTick() >= weapon.trig.h_timeout) {
//			trig_h_stop();
//			osTimerStop(mtr_get_timer(Tim_Motor_id));
//		}
////		ret = trig_handler(&weapon.trig);
////		if (ret == HAL_OK || ret == HAL_ERROR)
////			osTimerStop(mtr_get_timer(Tim_Weapon_id));
	}
}

void t_weapon(void const *argument)
{
	/* USER CODE BEGIN t_weapon */
	LOG("Created!\r\n");

	weapon.state = WS_IDLE;
	weapon.trig.enable = weapon.trig.start = weapon.trig.s_timeout = weapon.trig.h_timeout = 0;
	trig_all_stop();

	cock_init(&weapon.cock);
	munisi_reset();

	t_js_counter = 0;
	osDelay(T_Weapon_id);

	/* Infinite loop */
//	uint32_t led_timer = 0;
	uint32_t _bus_recv_timer = 0;
#if RTOS_USE_STACK_HIGH_WATER==1
	uint32_t _stack_highwater = 0;
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

	for ( ;; ) {
		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Weapon_id), 10);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Weapon_t *pRMail;
			pRMail = event.value.p;

			LOG("recv:%d -> %02X %02X\r\n", pRMail->sender_id, pRMail->param.command.mode.triggerEnable,
					pRMail->param.command.mode.cockEnable);

			switch (pRMail->sender_id)
			{
			case Weapon_Sender_Bus_id:
				if (pRMail->param.command.mode.triggerEnable == 1) {
					if (weapon.state == WS_IDLE) {
						trig_set_power(1);
						weapon.trig.enable = 1;
						weapon.state = WS_TRIGGER_ACTIVE;
					}
				}
				else {
					if (weapon.state == WS_TRIGGER_ACTIVE) {
						trig_set_power(0);
						weapon.trig.enable = 0;
						trig_all_stop();
						weapon.state = WS_IDLE;
					}
				}

				if (pRMail->param.command.mode.cockStartMoving == 1) {
					if (weapon.state == WS_COCK_ACTIVE) {
						cock_start(&weapon.cock);
						osTimerStart(mtr_get_timer(Tim_Weapon_id), 50);
						LOG("COCK start");
					}
				}

				if (pRMail->param.command.mode.cockEnable == 1) {
					if (weapon.state == WS_IDLE) {
						cock_power(&weapon.cock, 1);
						weapon.state = WS_COCK_ACTIVE;
						LOG("COCK enabled!\r\n");
					}
				}
				else {
					if (weapon.state == WS_COCK_ACTIVE) {
						cock_power(&weapon.cock, 0);
						weapon.state = WS_IDLE;
					}
				}

				if (weapon.state != WS_COCK_ACTIVE)
					weapon.trig.shoot_limit = pRMail->param.command.shoot_limit;

				if (pRMail->param.command.mode.munitionCounterReset == 1)
					munisi_reset();

				_bus_recv_timer = osKernelSysTick();

				if (weapon.state == WS_IDLE) {
					if (osMutexWait(mtr_get_mutex(Mutex_Motor_id), 0) == osOK) {
						osMutexRelease(mtr_get_mutex(Mutex_Motor_id));
						HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
					}
				}
				break;
			case Weapon_Sender_Sensor_Cock_id:
				break;
			case Weapon_Sender_Sensor_Trigger_id:
				break;
			};
			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Weapon_id), pRMail);

			if (weapon.trig.enable == 1) {
				if (trig_pulse_state() == 1) {
					if (weapon.trig.start == 0) {
						trig_start();
						weapon.trig.s_timeout = osKernelSysTick() + 60;
					}
					weapon.trig.start = 1;
				}
				else {
					trig_all_stop();
					weapon.trig.start = 0;
				}
			}
			else {
				trig_all_stop();
				weapon.trig.start = 0;
			}

			if (osKernelSysTick() >= weapon.trig.s_timeout) {
				trig_s_stop();
			}

		}

#if RTOS_USE_STACK_HIGH_WATER==1
		if (osKernelSysTick() >= _stack_highwater) {
			_stack_highwater = osKernelSysTick() + 1000;
			LOG("\t\tshw=%d\r\n",uxTaskGetStackHighWaterMark2(NULL));
		}
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Weapon_id, 0);

		if (weapon.state == WS_TRIGGER_ACTIVE)
			munisi_set_state(1);
		else
			munisi_set_state(0);

		if (weapon.state == WS_IDLE) {
			if (_bus_recv_timer == 0) {
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
			}
			else {
				if (osKernelSysTick() >= (_bus_recv_timer + BUS_MAX_TIMEOUT)) {
					_bus_recv_timer = 0;
				}
			}
		}
		else if (weapon.state == WS_TRIGGER_ACTIVE) {
			HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, !HAL_GPIO_ReadPin(T_START_GPIO_Port, T_START_Pin));
		}

		weapon_send_to_bus();

	}
	/* USER CODE END t_weapon */
}

static void weapon_send_to_bus()
{
	static uint32_t _timer = 0;

	if (osKernelSysTick() >= _timer) {
		_timer = osKernelSysTick() + 100;

		Body_weapon_status_t _state;
		MAIL_Bus_t *bus_mail;

		*(uint8_t*) &_state = 0;
		_state.triggerEnable = weapon.trig.enable;
		_state.cockEnable = weapon.cock.enable;
		_state.cockMoving = weapon.cock.start;

		// allocate memory; receiver must be free it
		bus_mail = osMailAlloc(mtr_get_mail(Mail_Bus_id), 0);
		bus_mail->sender_id = Bus_Sender_Weapon_State_id;
		bus_mail->param.weapon.state = _state;
		bus_mail->param.weapon.munition_counter = munisi_get_counter();

		/* send mail queue*/
		osMailPut(mtr_get_mail(Mail_Bus_id), bus_mail);

		LOG("mun= %d\r\n", munisi_get_counter());
	}
}
