/*
 * manual.c
 *
 *  Created on: Jul 13, 2022
 *      Author: 62812
 */

#include <stdlib.h>
#include "t_motor.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_MOTOR_MANUAL==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%ld TMMan:%d] " str, (osKernelSysTick()%100000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMMan_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR_MANUAL==1

static const uint32_t MANUAL_UPDATE_TIMEOUT = MOTOR_UPDATE_TIMEOUT;

void t_motor_manual(void const *argument)
{
	/* USER CODE BEGIN t_motor_manual */
	uint32_t _mtr_update_timer = 0;
	uint32_t _cmd_recv_timestamp = 0;
	uint32_t _debug_send_timer = 0;
	uint16_t _debug_motor_command_counter = 0;
#if RTOS_USE_STACK_HIGH_WATER==1
	uint32_t _stack_highwater_timer = 0;
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

	*(uint8_t*) &motor.mode_state = 0;
	motor.mode_state.movementMode = MOVE_MODE_MAN;

	LOG("t_manual created\r\n");

	/* Infinite loop */
	for ( ;; ) {
		int32_t _p = 0;
		int32_t _t = 0;

		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_Ext_id), 5);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Motor_Ext_t *pRMail;
			MAIL_Motor_t *ext_state;
			pRMail = event.value.p;

			switch (pRMail->sender_id)
			{
			case Motor_Ext_Sender_Mode_id:
				LOG("mode= %02X -> %d\r\n", *(uint8_t* )&pRMail->param.mode, motor.tilt_command.power_enable);
				motor.pan_command.power_enable = motor.tilt_command.power_enable = pRMail->param.mode.motor_enable;
				if (pRMail->param.mode.ready_to_be_terminated == 1) {
					osMutexWait(mtr_get_mutex(Mutex_motor_ext_id), osWaitForever);
					/* free memory allocated for mail */
					osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);

//					/* stop motor */
//					hal_motor_set_pan_power(0);
//					motor.pan_state.power = 0;
//					hal_motor_set_tilt_power(0);
//					motor.tilt_state.power = 0;

					/* send ack to t_motor */
					/* allocate memory; receiver must be free it */
					ext_state = osMailAlloc(mtr_get_mail(Mail_Motor_id), 0);
					ext_state->sender_id = Motor_Sender_Ext_id;
					ext_state->param.ext.ready_to_be_terminated = 1;
					/* send mail queue*/
					osMailPut(mtr_get_mail(Mail_Motor_id), ext_state);

					osMutexRelease(mtr_get_mutex(Mutex_motor_ext_id));

					LOG("terminated!\r\n");

					/* delete task */
					osThreadTerminate(NULL);
					break;
				}
				_cmd_recv_timestamp = HAL_GetTick();
				break;
			case Motor_Ext_Sender_Value_id:
				_p = pRMail->param.value.pan;
				_t = pRMail->param.value.tilt;
				if (labs(_p) < 100)
					_p = 0;
				_p = hal_motor_pan_speed_to_c(_p);
				motor.pan_command.spd_man_in_c = _p;

				if (labs(_t) < 100)
					_t = 0;
				_t = hal_motor_tilt_speed_to_c(_t);
				motor.tilt_command.spd_man_in_c = _t;

				LOG("cmd=%ld,%ld\r\n", motor.pan_command.spd_man_in_c, motor.tilt_command.spd_man_in_c);
				_cmd_recv_timestamp = HAL_GetTick();
				break;
			default:
				break;
			}
			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);
		}

		if ((_cmd_recv_timestamp > 0) && (HAL_GetTick() >= _cmd_recv_timestamp + BUS_MAX_TIMEOUT)) {
			_cmd_recv_timestamp = 0;

			motor.pan_command.power_enable = 0;
			motor.pan_command.spd_man_in_c = 0;
			motor.pan_command.pos_bal_in_c = 0;

			motor.tilt_command.power_enable = 0;
			motor.tilt_command.spd_man_in_c = 0;
			motor.tilt_command.pos_bal_in_c = 0;

			LOG("timeout\r\n\r\n");
		}

		if (HAL_GetTick() >= _mtr_update_timer) {
			_mtr_update_timer = HAL_GetTick() + MANUAL_UPDATE_TIMEOUT;

			hal_motor_update_motor_state(&motor, 1);

			_debug_motor_command_counter += hal_motor_set_speed(motor.pan_command.spd_man_in_c,
					motor.tilt_command.spd_man_in_c);

//			LOG("cmd=%d,%ld,%ld\r\n", motor.tilt_command.power_enable,motor.pan_command.spd_man_in_c, motor.tilt_command.spd_man_in_c);

		}

		if (_debug_send_timer <= HAL_GetTick()) {
			_debug_send_timer = HAL_GetTick() + 1000;

//			LOG("[%d]in c= %ld,%ld\r\n", _debug_motor_command_counter, motor.pan_command.spd_man_in_c,
//					motor.tilt_command.spd_man_in_c);
//			LOG("[%d]spd in c= %ld,%ld\r\n", _debug_motor_command_counter, motor.pan_state.speed, motor.tilt_state.speed);
			_debug_motor_command_counter = 0;
		}

#if RTOS_USE_STACK_HIGH_WATER==1
		if (osKernelSysTick() >= _stack_highwater_timer) {
			_stack_highwater_timer = osKernelSysTick() + 1000;

			LOG("\t\tshw=%d\r\n", uxTaskGetStackHighWaterMark2(NULL));
		}
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_Ext_id, 0);
	}
	/* USER CODE END t_motor_manual */
}

