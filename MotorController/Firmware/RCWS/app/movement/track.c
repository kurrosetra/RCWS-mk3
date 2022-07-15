/*
 * track.c
 *
 *  Created on: May 27, 2022
 *      Author: 62812
 */

#include "t_motor.h"
#include "driver/kontrol/track_control.h"

#if DEBUG_MOTOR_TRACK==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%ld TMTrk:%d] " str, (osKernelSysTick()%100000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMTrk_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR_TRACK==1

static const uint32_t TRACK_UPDATE_TIMEOUT = MOTOR_UPDATE_TIMEOUT;
void t_motor_track(void const *argument)
{
	/* USER CODE BEGIN t_motor_track */
	uint32_t _track_timer = 0;
	uint32_t _cmd_recv_timestamp = HAL_GetTick();
	uint32_t _debug_send_timer = 0;
	uint16_t _debug_motor_command_counter = 0;

	LOG("t_track created!\r\n");

	*(uint8_t*) &motor.mode_state = 0;
	motor.mode_state.movementMode = MOVE_MODE_TRACK;

	motor.pan_command.power_enable = motor.tilt_command.power_enable = 1;
	hal_motor_update_motor_state(&motor, 1);

	Kontrol_Konstanta_init();
	Kontrol_init();

	/* Infinite loop */
	for ( ;; ) {
		float wMotor[2] = { 0.0f, 0.0f };

		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_Ext_id), 5);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Motor_Ext_t *pRMail;
			pRMail = event.value.p;

			switch (pRMail->sender_id)
			{
			case Motor_Ext_Sender_Mode_id:
				LOG("mode= %02X\r\n", *(uint8_t* ) &pRMail->param.mode);
				if (pRMail->param.mode.ready_to_be_terminated == 1) {
					osMutexWait(mtr_get_mutex(Mutex_motor_ext_id), osWaitForever);

					/* free memory allocated for mail */
					osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);

					/* send ack to t_motor */
					MAIL_Motor_t *ext_state;
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
//				trk_state = *(uint8_t*) &pRMail->param.track.trk_state;

				Kontrol_CalcQDot(*(uint8_t*) &pRMail->param.track.trk_state, motor.pan_state.pos, motor.tilt_state.pos,
						motor.pan_state.speed, motor.pan_state.speed, pRMail->param.track.trk_x, pRMail->param.track.trk_y,
						&wMotor[0], &wMotor[1]);

				break;
			case Motor_Ext_Sender_Offset_id:
				break;
			default:
				break;
			}

			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);
		}

		if (_cmd_recv_timestamp > 0 && (HAL_GetTick() >= _cmd_recv_timestamp + BUS_MAX_TIMEOUT)) {
			motor.pan_command.power_enable = motor.tilt_command.power_enable = 0;
			motor.pan_command.spd_trk_in_c = motor.tilt_command.spd_trk_in_c = 0;
			hal_motor_update_motor_state(&motor, 1);
		}

		if (HAL_GetTick() >= _track_timer) {
			_track_timer = HAL_GetTick() + TRACK_UPDATE_TIMEOUT;

			hal_motor_update_motor_state(&motor, 0);

			_debug_motor_command_counter += hal_motor_set_speed(motor.pan_command.spd_trk_in_c,
					motor.tilt_command.spd_trk_in_c);

//			LOG("cmd=%d,%ld,%ld\r\n", motor.tilt_command.power_enable,motor.pan_command.spd_man_in_c, motor.tilt_command.spd_man_in_c);
		}

		if (_debug_send_timer <= HAL_GetTick()) {
			_debug_send_timer = HAL_GetTick() + 1000;

			LOG("[%d]in c= %ld,%ld\r\n", _debug_motor_command_counter, motor.pan_command.spd_man_in_c,
					motor.tilt_command.spd_man_in_c);
			_debug_motor_command_counter = 0;
		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_Ext_id, 0);
	}
	/* USER CODE END t_motor_track */
}

