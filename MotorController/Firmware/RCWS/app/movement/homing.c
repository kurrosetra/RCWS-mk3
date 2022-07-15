/*
 * homing.c
 *
 *  Created on: Jun 25, 2022
 *      Author: 62812
 */

#include <stdlib.h>
#include "t_motor.h"

#if DEBUG_MOTOR_HOMING==1
#include <stdio.h>
#	define LOG(str, ...) printf("[%ld TMHom:%d] " str, (osKernelSysTick()%100000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMHom_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR_HOMING==1

static const uint32_t HOMING_UPDATE_TIMEOUT = 25;
static const uint32_t HOMING_HYST_POS = 5;

void t_motor_homing(void const *argument)
{
	/* USER CODE BEGIN t_motor_homing */
	uint32_t _homing_timer = 0;
	LOG("t_homing created!\r\n");

	*(uint8_t*) &motor.mode_state = 0;
	motor.mode_state.movementMode = MOVE_MODE_HOMING;

	motor.pan_command.power_enable = motor.tilt_command.power_enable = 1;
	/* update motor state */
	hal_motor_update_motor_state(&motor, 1);

	/* Infinite loop */
	for ( ;; ) {
		int32_t _p, _t;
		float _fp, _ft;

		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_Ext_id), 5);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Motor_Ext_t *pRMail;
			pRMail = event.value.p;

			switch (pRMail->sender_id)
			{
			case Motor_Ext_Sender_Mode_id:
				LOG("mode= %02X\r\n", *(uint8_t* )&pRMail->param.mode);
				if (pRMail->param.mode.ready_to_be_terminated == 1) {
					osMutexWait(mtr_get_mutex(Mutex_motor_ext_id), osWaitForever);

					/* free memory allocated for mail */
					osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);

					/* stop motor */
					hal_motor_set_pan_power(0);
					motor.pan_state.power = 0;
					hal_motor_set_tilt_power(0);
					motor.tilt_state.power = 0;

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
				break;
			case Motor_Ext_Sender_Value_id:
				if (motor.mode_state.moveModeStart == 0) {
					_p = motor.pan_state.pos;
					_t = motor.tilt_state.pos;
					_fp = (float) pRMail->param.value.pan / 1000;
					_ft = (float) pRMail->param.value.tilt / 1000;
					LOG("[I]recv hom=%.3f,%.3f\r\n", _fp, _ft);

					LOG("\r\n\r\n\r\n\r\n");
					/* convert actual position to deg */
					int32_t _rp = _p % RWS_MOTOR_PAN_FULL_REV_IN_C;
					float c_az = RWS_MOTOR_PAN_C_TO_DEG(_rp);
					if (c_az < 0)
						c_az += 360.f;

					LOG("[H]c_az=%.3f\r\n", c_az);

					/* find shortest angle */
					float d_az = 0.0f;
					if (_fp >= c_az)
						d_az = _fp - c_az;
					else
						d_az = _fp + 360.0f - c_az;

					if (d_az > 180.0f)
						d_az -= 360.0f;

					LOG("[H]d_az=%.3f\r\n", d_az);

					/* convert to c relative to actual position */
					motor.pan_command.pos_hom_in_c = _p + RWS_MOTOR_PAN_DEG_TO_C(d_az);
					LOG("[H]target=%ld\r\n", motor.pan_command.pos_hom_in_c);

					float c_el = RWS_MOTOR_TILT_C_TO_DEG(_t);
					float d_el = _ft - c_el;
					motor.tilt_command.pos_hom_in_c = _t + RWS_MOTOR_TILT_DEG_TO_C(d_el);

					LOG("\r\n\r\n\r\n\r\n");

					motor.mode_state.moveModeStart = 1;
				}
				break;
			case Motor_Ext_Sender_Balistic_id:
				break;
			default:
				break;
			}

			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Motor_Ext_id), pRMail);
		}

		if (HAL_GetTick() >= _homing_timer) {
			_homing_timer = HAL_GetTick() + HOMING_UPDATE_TIMEOUT;

			/* update motor state */
			hal_motor_update_motor_state(&motor, 0);
			/* check if ended */
			uint8_t _mtr_ended = 0;
			int32_t _dif_pos = 0;
			if (is_motor_az_enable(motor.enable)) {
				_dif_pos = motor.pan_command.pos_hom_in_c - motor.pan_state.pos;
				if (labs(_dif_pos) <= HOMING_HYST_POS) {
					_mtr_ended |= MTR_AZ_ENABLE;
					motor.pan_command.power_enable = 0;
				}
				else
					motor.tilt_command.power_enable = 1;
			}
			else
				_mtr_ended |= MTR_AZ_ENABLE;

			if (is_motor_el_enable(motor.enable)) {
				_dif_pos = motor.tilt_command.pos_hom_in_c - motor.tilt_state.pos;
				if (labs(_dif_pos) <= HOMING_HYST_POS) {
					_mtr_ended |= MTR_EL_ENABLE;
					motor.tilt_command.power_enable = 0;
				}
				else
					motor.tilt_command.power_enable = 1;
			}
			else
				_mtr_ended |= MTR_EL_ENABLE;

			if (_mtr_ended == (MTR_AZ_ENABLE | MTR_EL_ENABLE))
				motor.mode_state.moveModeEnd = 1;

			/* start moving */
			if (motor.mode_state.moveModeStart == 1 && motor.mode_state.moveModeEnd == 0)
				hal_motor_set_position(motor.pan_command.pos_hom_in_c, motor.tilt_command.pos_hom_in_c);

		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_Ext_id, 0);
	}
	/* USER CODE END t_motor_homing */
}

