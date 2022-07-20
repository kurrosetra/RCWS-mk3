/*
 * t_motor.c
 *
 *  Created on: Jul 12, 2022
 *      Author: 62812
 */
#include <stdlib.h>

#include "t_motor.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_MOTOR==1
#include <stdio.h>
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TMtr:%d] " str, (osKernelSysTick()%100000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMtr_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR==1

Motor_t motor;
uint32_t g_init_timeout = 0;

osThreadId TMoveManualHandle;
osThreadId TMoveTravelHandle;
osThreadId TMoveStabHandle;
osThreadId TMoveTrackHandle;
osThreadId TMoveMemoryHandle;
osThreadId TMoveHomingHandle;

extern void t_motor_manual(void const *argument);
extern void t_motor_travel(void const *argument);
extern void t_motor_stab(void const *argument);
extern void t_motor_track(void const *argument);
extern void t_motor_memory(void const *argument);
extern void t_motor_homing(void const *argument);

static void t_motor_init();
static void mtr_send_to_bus();
static void t_motor_ext_revive_task(const uint8_t movement_mode);

void tim_motor_callback(void const *argument)
{
	/* USER CODE BEGIN tim_motor_callback */
	if (HAL_GetTick() < g_init_timeout)
		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), (T_Motor_id | T_Motor_Ext_id), 0);
	else
		osTimerStop(mtr_get_timer(Tim_Motor_id));
}

void t_motor(void const *argument)
{
	/* USER CODE BEGIN t_motor */
	uint32_t _bus_send_timer = 0;

	LOG("Created!\r\n");

	osMutexWait(mtr_get_mutex(Mutex_Motor_id), osWaitForever);
	/* TODO Select which motor to be activated */
	motor.enable = 0;
	motor.enable |= MTR_AZ_ENABLE;
//	motor.enable |= MTR_EL_ENABLE;

	if (get_reset_cause() == RESET_CAUSE_EXTERNAL_RESET_PIN_RESET)
		osDelay(3000);
	t_motor_init();
	osMutexRelease(mtr_get_mutex(Mutex_Motor_id));
	t_motor_ext_revive_task(MOVE_MODE_MAN);

	/* create manual task */

	/* Infinite loop */
	for ( ;; ) {
		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_id), 50);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Motor_t *pRMail;
			MAIL_Motor_Ext_t *ext_mail;
			pRMail = event.value.p;

			switch (pRMail->sender_id)
			{
			case Motor_Sender_Bus_Mode_id:
//				LOG("[I]cmd=%02X;st=%02X;rst=%d\r\n", *(uint8_t* )&pRMail->param.mode, *(uint8_t* )&motor.mode_state,
//						get_reset_cause());
				motor.mode_command.movementMode = pRMail->param.mode.movementMode;

				if (osMutexWait(mtr_get_mutex(Mutex_motor_ext_id), 0) == osOK) {
					/* allocate memory; receiver must be free it */
					ext_mail = osMailAlloc(mtr_get_mail(Mail_Motor_Ext_id), 0);
					ext_mail->sender_id = Motor_Ext_Sender_Mode_id;
					*(uint8_t*) &ext_mail->param.mode = 0;

					if (pRMail->param.mode.modeAbort == 1)
						motor.mode_command.movementMode = MOVE_MODE_MAN;
					if (mtr_get_movement_mode() != motor.mode_command.movementMode)
						ext_mail->param.mode.ready_to_be_terminated = 1;
					ext_mail->param.mode.motor_enable = pRMail->param.mode.motorEnable;
					ext_mail->param.mode.balistic_active = pRMail->param.mode.ballisticActive;

					/* send mail queue*/
					osMailPut(mtr_get_mail(Mail_Motor_Ext_id), ext_mail);
					osMutexRelease(mtr_get_mutex(Mutex_motor_ext_id));
				}
				break;
			case Motor_Sender_Ext_id:
				if (pRMail->param.ext.ready_to_be_terminated == 1) {
					/* revive command.mode.movementMode */
					t_motor_ext_revive_task(motor.mode_command.movementMode);
				}
				break;
			case Motor_Sender_Sensor_id:
				break;
			default:
				break;
			}

			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Motor_id), pRMail);
		}

		if (osKernelSysTick() >= _bus_send_timer) {
			_bus_send_timer = HAL_GetTick() + 100;

			mtr_send_to_bus();
			LOG("[O]state=%02X,%04X,%02X,%04X\r\n", motor.pan_state.power, motor.pan_state.statusword,
					motor.tilt_state.power, motor.tilt_state.statusword);
		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_id, 0);
	}
	/* USER CODE END t_motor */
}

static void t_motor_init()
{
	motor.pan_state.pos = motor.tilt_state.pos = 0;

	g_init_timeout = osKernelSysTick() + MTR_INIT_TIMEOUT;
	osTimerStart(mtr_get_timer(Tim_Motor_id), 500);
	while (hal_motor_init(motor.enable) != HAL_OK) {
		LOG_E("motor init failed!\r\n");
		osDelay(10);
	}
	osTimerStop(mtr_get_timer(Tim_Motor_id));
}

static void t_motor_ext_revive_task(const uint8_t movement_mode)
{
	static const int8_t priority = osPriorityNormal;
	taskENTER_CRITICAL();

	if (movement_mode == MOVE_MODE_MAN) {
		/* definition and creation of TMoveManual */
		osThreadDef(TMoveManual, t_motor_manual, priority, 0, 512);
		TMoveManualHandle = osThreadCreate(osThread(TMoveManual), NULL);
	}
	else if (movement_mode == MOVE_MODE_TRAVEL) {
		/* definition and creation of TMoveManual */
		osThreadDef(TMoveTravel, t_motor_travel, priority, 0, 512);
		TMoveTravelHandle = osThreadCreate(osThread(TMoveTravel), NULL);
	}
	else if (movement_mode == MOVE_MODE_STAB) {
		/* definition and creation of TMoveStab */
		osThreadDef(TMoveStab, t_motor_stab, priority, 0, 512);
		TMoveStabHandle = osThreadCreate(osThread(TMoveStab), NULL);
	}
	else if (movement_mode == MOVE_MODE_TRACK) {
		/* definition and creation of TMoveTrack */
		osThreadDef(TMoveTrack, t_motor_track, priority, 0, 512);
		TMoveTrackHandle = osThreadCreate(osThread(TMoveTrack), NULL);
	}
	else if (movement_mode == MOVE_MODE_MEMORY) {
		/* definition and creation of TMoveMemory */
		osThreadDef(TMoveMemory, t_motor_memory, priority, 0, 512);
		TMoveMemoryHandle = osThreadCreate(osThread(TMoveMemory), NULL);
	}
	else if (movement_mode == MOVE_MODE_HOMING) {
		/* definition and creation of TMoveHoming */
		osThreadDef(TMoveHoming, t_motor_homing, priority, 0, 512);
		TMoveHomingHandle = osThreadCreate(osThread(TMoveHoming), NULL);
	}

	taskEXIT_CRITICAL();
}

static void mtr_send_to_bus()
{
	/* ================ */
	/* send motor state */
	/* ================ */
	Body_motor_status_t _pan_state;
	Body_motor_status_t _tilt_state;
	MAIL_Bus_t *state_mail;

	*(uint8_t*) &_pan_state = 0;
	*(uint8_t*) &_tilt_state = 0;
	_pan_state.motorEnable = motor.pan_state.power;
	if ((motor.pan_state.statusword & 0x4000) != 0)
		_pan_state.initialAngleSuccess = 1;
	if ((motor.pan_state.statusword & 0x8) != 0)
		_pan_state.fault |= 0b1;
	if ((motor.pan_state.statusword & 0x2000) != 0)
		_pan_state.fault |= 0b10;

	_tilt_state.motorEnable = motor.tilt_state.power;
	if ((motor.tilt_state.statusword & 0x4000) != 0)
		_tilt_state.initialAngleSuccess = 1;
	if ((motor.tilt_state.statusword & 0x8) != 0)
		_tilt_state.fault |= 0b1;
	if ((motor.tilt_state.statusword & 0x2000) != 0)
		_tilt_state.fault |= 0b10;

	/* allocate memory; receiver must be free it */
	state_mail = osMailAlloc(mtr_get_mail(Mail_Bus_id), 0);
	state_mail->sender_id = Bus_Sender_Motor_State_id;
	state_mail->param.motor.state.mode = motor.mode_state;
	state_mail->param.motor.state.pan_state = _pan_state;
	state_mail->param.motor.state.tilt_state = _tilt_state;

	/* send mail queue*/
	osMailPut(mtr_get_mail(Mail_Bus_id), state_mail);

	/* =================== */
	/* send motor position */
	/* =================== */
	MAIL_Bus_t *pos_mail;

	/* allocate memory; receiver must be free it */
	pos_mail = osMailAlloc(mtr_get_mail(Mail_Bus_id), 0);
	pos_mail->sender_id = Bus_Sender_Motor_Position_id;
	pos_mail->param.motor.position.pan = motor.pan_state.pos;
	pos_mail->param.motor.position.tilt = motor.tilt_state.pos;

//	LOG("st=%02X;PT=%ld,%ld\r\n", *(uint8_t* )&motor.mode_state, pos_mail->param.motor.position.pan,
//			pos_mail->param.motor.position.tilt);

	/* send mail queue*/
	osMailPut(mtr_get_mail(Mail_Bus_id), pos_mail);
}

uint8_t mtr_get_movement_mode()
{
	return motor.mode_state.movementMode;
}

