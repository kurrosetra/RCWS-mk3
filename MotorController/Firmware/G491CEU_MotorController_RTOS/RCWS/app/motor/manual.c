/*
 * manual.c
 *
 *  Created on: May 27, 2022
 *      Author: 62812
 */

#include "t_motor.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_MOTOR_MANUAL==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%ld TMMan:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMMan_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR_MANUAL==1

void t_motor_manual(void const *argument)
{
	/* USER CODE BEGIN t_motor_manual */
	uint32_t _mtr_send_timer = 0;

	LOG("t_manual created\r\n");

	uint32_t _debug_send_timer = 0;
#if RTOS_USE_STACK_HIGH_WATER==1
	uint32_t _stack_highwater_timer = 0;
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

	/* Infinite loop */
	for ( ;; ) {
		uint8_t _dbg_send_mtr_cmd_counter = 0;
		vTaskDelayUntil(&_mtr_send_timer, 25);

		hal_motor_get_motor_state(&motor.pan_state, &motor.tilt_state);
		mtr_set_power(&motor);

		if (motor.mode_state.moveMode == TASK_MOTOR_HOMING_id) {
			if (homing_state == 1)
				hal_motor_set_position(motor.pan_command.pos_hom_in_c, motor.tilt_command.pos_hom_in_c);
		}
		else {
			_dbg_send_mtr_cmd_counter += hal_motor_set_speed(motor.pan_command.spd_man_in_c,
					motor.tilt_command.spd_man_in_c);

			if (_debug_send_timer <= osKernelSysTick()) {
				_debug_send_timer = osKernelSysTick() + 500;
				LOG("[%d]in c= %ld,%ld\r\n", _dbg_send_mtr_cmd_counter, motor.pan_command.spd_man_in_c,
						motor.tilt_command.spd_man_in_c);
			}

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

