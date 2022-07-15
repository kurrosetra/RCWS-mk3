/*
 * bus.c
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#include <stdio.h>
#include <string.h>

#include "common.h"
#include "driver/bus_fdcan/bus_fdcan.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_BUS==1
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TBus:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TBus_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MANAGER==1

BUS_Rx_Buffer_t bus_recv;
BUS_Tx_Buffer_t bus_send;

void t_bus(void const *argument)
{
	/* USER CODE BEGIN t_bus_recv */
	uint32_t _timer_motor_state, _timer_weapon_state, _timer_pos, _timer_imu;
	uint8_t _send_motor_state[5];
	uint8_t _send_weapon_state[3];
	int32_t _pan = 0, _tilt = 0;
	int16_t _yaw = 0;
	int32_t _pitch = 0, _roll = 0;

	LOG("BUS init\r\n");
	bus_init();

	osDelay(T_Bus_id);

	_timer_motor_state = _timer_weapon_state = _timer_pos = _timer_imu = osKernelSysTick() + 100;
	/* Infinite loop */
	for ( ;; ) {
		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Bus_id), 20);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Bus_t *pRMail;
			pRMail = event.value.p;

			switch (pRMail->sender_id)
			{
			case Bus_Sender_Weapon_State_id:
				/* from weapon_send_to_bus -> t_weapon.c */
				_send_weapon_state[0] = *(uint8_t*) &pRMail->param.weapon.state;
				_send_weapon_state[1] = pRMail->param.weapon.munition_counter >> 8;
				_send_weapon_state[2] = pRMail->param.weapon.munition_counter & 0xFF;
				break;
			case Bus_Sender_Motor_State_id:
				/* from mtr_send_to_bus() -> t_motor.c */
				_send_motor_state[0] = *(uint8_t*) &pRMail->param.motor.state.mode;
				_send_motor_state[1] = *(uint8_t*) &pRMail->param.motor.state.pan_state;
				_send_motor_state[2] = 0;
				_send_motor_state[3] = *(uint8_t*) &pRMail->param.motor.state.tilt_state;
				_send_motor_state[4] = 0;
				LOG("%02X %02X\r\n", _send_motor_state[0], _send_motor_state[1]);
				break;
			case Bus_Sender_Motor_Position_id:
				/* from mtr_send_to_bus() -> t_motor.c */
				_pan = pRMail->param.motor.position.pan;
				_tilt = pRMail->param.motor.position.tilt;
				break;
			case Bus_Sender_Imu_id:
				_yaw = pRMail->param.imu.yaw;
				_pitch = pRMail->param.imu.pitch;
				_roll = pRMail->param.imu.roll;
				break;
			}

			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Bus_id), pRMail);
		}

		/* send motor state to panel */
		if (HAL_GetTick() >= _timer_motor_state) {
			_timer_motor_state = HAL_GetTick() + 100;

			if (bus_send_motor_status(_send_motor_state) == HAL_OK)
				LOG("send M state: %02X\r\n", _send_motor_state[0]);
			else
				_timer_motor_state = HAL_GetTick() + 1;
		}

		if (HAL_GetTick() >= _timer_weapon_state) {
			_timer_weapon_state = HAL_GetTick() + 100;

			if (bus_send_weapon_status(_send_weapon_state) == HAL_OK)
				LOG("send W state: %02X\r\n",_send_motor_state[0]);
			else
				_timer_weapon_state = HAL_GetTick() + 1;
		}

		/* send motor position to panel */
		if (HAL_GetTick() >= _timer_pos) {
			_timer_pos = HAL_GetTick() + 100;

			if (bus_send_position(_pan, _tilt) == HAL_OK)
				LOG("send position (%ld,%ld)\r\n", _pan, _tilt);
			else
				_timer_pos = HAL_GetTick() + 1;
		}

		/* send imu's platform to panel */
		if (osKernelSysTick() >= _timer_imu) {
			_timer_imu = osKernelSysTick() + 100;

			if (bus_send_imu(_yaw, _pitch, _roll) != HAL_OK)
				_timer_imu = osKernelSysTick() + 1;

		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Bus_id, 0);
	}
	/* USER CODE END t_bus_recv */
}

