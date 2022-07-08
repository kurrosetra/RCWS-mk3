/*
 * homing.c
 *
 *  Created on: Jun 25, 2022
 *      Author: 62812
 */

#include "t_motor.h"
#if DEBUG_MOTOR_HOMING==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%ld TMHom:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMHom_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR_HOMING==1

void t_motor_homing(void const *argument)
{
	/* USER CODE BEGIN t_motor_homing */
	LOG("t_homing created!\r\n");
	int32_t target_pos_pan = 0;
	int32_t target_pos_tilt = 0;

	osDelay(100);
	/* Infinite loop */
	uint32_t timestamp = 0;
	for ( ;; ) {
		osDelayUntil(&timestamp, 10);
		hal_motor_get_motor_state(&motor.pan_state, &motor.tilt_state);
		mtr_set_power(&motor);
		hal_motor_set_position(target_pos_pan, target_pos_tilt);

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_Ext_id, 0);
	}

//	for ( ;; ) {
//		/* get message from the queue */
//		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_Ext_id), 10);
//		if (event.status == osEventMail) {
//			/* create buffer pointer to hold queue value */
//			MAIL_Motor_Ext_t *pRMail;
//			pRMail = event.value.p;
//
////			if (start == 0) {
////				start = 1;
//
//			target_pos_pan = pRMail->command.value.pan;
//			target_pos_tilt = pRMail->command.value.tilt;
//
//			LOG("start move=%ld,%ld\r\n", target_pos_pan, target_pos_tilt);
//			motor.pan_command.power_enable = motor.tilt_command.power_enable = 1;
//			mtr_set_power(&motor);
//			hal_motor_set_position(target_pos_pan, target_pos_tilt);
////			}
//
//			/* free memory allocated for mail */
//			osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);
//		}
//
//		hal_motor_get_motor_state(&motor.pan_state, &motor.tilt_state);
//
//		/* todo check if already in position */
////		if(motor.pan_state.statusword&0x)
//		/* send notif to task manager that this thread is still running */
//		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_Ext_id, 0);
//	}
	/* USER CODE END t_motor_homing */
}

