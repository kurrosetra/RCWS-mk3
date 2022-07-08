/*
 * motor.c
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#include <stdio.h>
#include <stdlib.h>

#include "t_motor.h"
#include "driver/hal_motor/hal_motor.h"
#include "rws_config.h"

#include "iwdg.h"

#if RTOS_USE_STACK_HIGH_WATER==1
#include "task.h"
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

#if DEBUG_MOTOR==1
/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld TMtr:%d] " str, (osKernelSysTick()%10000UL), __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[TMtr_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR==1

Motor_t motor = { .current_task_id = 0 };

#define is_motor_az_enable()	((motor.enable & MTR_AZ_ENABLE) == MTR_AZ_ENABLE)
#define is_motor_el_enable()	((motor.enable & MTR_EL_ENABLE) == MTR_EL_ENABLE)

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

typedef enum
{
	timer_used_by_none,
	timer_used_by_init,
	timer_used_by_homing,
} Timer_user_e;
Timer_user_e tim_user = timer_used_by_none;
uint8_t g_timer_in_used = 0;
uint32_t g_init_timeout = 0;
uint8_t homing_state = 0;

static void mtr_send_to_bus();
static void _stop_timer();
static int32_t _man_pan_speed_to_c(int32_t speed);
static int32_t _man_tilt_speed_to_c(int32_t speed);
static void t_motor_delete_task(const uint8_t living_task);
static void t_motor_change_task(const uint8_t task_tobe_revived);
static void t_motor_init(const uint32_t delay);

void tim_motor_callback(void const *argument)
{
	/* USER CODE BEGIN tim_motor_callback */
	g_timer_in_used = 1;
	if (tim_user == timer_used_by_init) {
		if (osKernelSysTick() < g_init_timeout)
			/* send notif to task manager that this thread is still running */
			osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), (T_Motor_id | T_Motor_Ext_id), 0);
	}
	else {
		_stop_timer();
	}
}

void t_motor(void const *argument)
{
	/* USER CODE BEGIN t_motor */
//	uint32_t _mtr_send_timer = 0;
	uint32_t _bus_send_timer = 0;
	uint32_t _bus_recv_timestamp = 0;
#if RTOS_USE_STACK_HIGH_WATER==1
	uint32_t _stack_highwater = 0;
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

	LOG("Created!\r\n");

	osMutexWait(mtr_get_mutex(Mutex_Motor_id), osWaitForever);
	/* TODO Select which motor to be activated */
	motor.enable = 0;
	motor.enable |= MTR_AZ_ENABLE;
	motor.enable |= MTR_EL_ENABLE;

	t_motor_init(3000);
	osMutexRelease(mtr_get_mutex(Mutex_Motor_id));

	motor.current_task_id = 0;
	t_motor_change_task(TASK_MOTOR_MANUAL_id);

	/* Infinite loop */
	for ( ;; ) {
		int32_t _p, _t;
		float _fp, _ft;

		/* get message from the queue */
		osEvent event = osMailGet(mtr_get_mail(Mail_Motor_id), 50);
		if (event.status == osEventMail) {
			/* create buffer pointer to hold queue value */
			MAIL_Motor_t *pRMail;
			pRMail = event.value.p;

			Motor_Sender_Id_e sender = pRMail->sender_id;
			switch (sender)
			{
			case Motor_Sender_Bus_Mode_id:
				LOG("[I]st= %02X\r\n", *(uint8_t* )&pRMail->param.command.mode);
				if (motor.mode_state.moveMode == TASK_MOTOR_MANUAL_id)
					motor.pan_command.power_enable = motor.tilt_command.power_enable =
							pRMail->param.command.mode.motorEnable;
//				*(uint8_t*) &motor.mode_command = *(uint8_t*) &pRMail->param.command.mode;
				switch (pRMail->param.command.mode.moveMode)
				{
				case MOVE_MODE_MAN:
					motor.mode_command.moveMode = TASK_MOTOR_MANUAL_id;
					break;
				case MOVE_MODE_TRAVEL:
					motor.mode_command.moveMode = TASK_MOTOR_TRAVEL_id;
					break;
				case MOVE_MODE_STAB:
					motor.mode_command.moveMode = TASK_MOTOR_STAB_id;
					break;
				case MOVE_MODE_TRACK:
					motor.mode_command.moveMode = TASK_MOTOR_TRACK_id;
					break;
				case MOVE_MODE_MEMORY:
					motor.mode_command.moveMode = TASK_MOTOR_MEMORY_id;
					break;
				case MOVE_MODE_HOMING:
					motor.mode_command.moveMode = TASK_MOTOR_HOMING_id;
					break;
				default:
					break;
				}

				/* TODO select mode active */
				if (motor.mode_state.moveMode != motor.mode_command.moveMode) {
					motor.mode_state.moveMode = motor.mode_command.moveMode;

					LOG("Change task motor ext [%d]\r\n", motor.mode_state.moveMode);
				}

				_bus_recv_timestamp = osKernelSysTick();
				break;
			case Motor_Sender_Bus_Command_Speed_id:
				_p = pRMail->param.command.value.pan;
				_t = pRMail->param.command.value.tilt;
				LOG("[I]rx man= %ld,%ld\r\n", _p, _t);

				if (motor.mode_state.moveMode == TASK_MOTOR_MANUAL_id) {
					if (motor.pan_command.power_enable != 0) {
						if (abs(_p) < 100)
							_p = 0;
						motor.pan_command.spd_man_in_c = _man_pan_speed_to_c(_p);
					}
					else
						motor.pan_command.spd_man_in_c = 0;

					if (motor.tilt_command.power_enable != 0) {
						if (abs(_t) < 100)
							_p = 0;
						motor.tilt_command.spd_man_in_c = _man_tilt_speed_to_c(_t);
					}
					else
						motor.tilt_command.spd_man_in_c = 0;

					_bus_recv_timestamp = osKernelSysTick();
				}

				_bus_recv_timestamp = osKernelSysTick();
				break;
			case Motor_Sender_Bus_Command_Homing_id:
				_p = motor.pan_state.pos;
				_t = motor.tilt_state.pos;
				_fp = (float) pRMail->param.command.value.pan / 1000;
				_ft = (float) pRMail->param.command.value.tilt / 1000;
				LOG("[I]recv hom=%.3f,%.3f\r\n", _fp, _ft);
				if (motor.mode_state.moveMode == TASK_MOTOR_HOMING_id) {
					if (homing_state == 0) {
						homing_state = 1;
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

//						motor.tilt_command.pos_hom_in_c = 0;
						LOG("\r\n\r\n\r\n\r\n");
					} LOG("[S]target=%ld,%ld\r\n", motor.pan_command.pos_hom_in_c, motor.tilt_command.pos_hom_in_c);
					_bus_recv_timestamp = osKernelSysTick();
				}
				break;
			default:
				break;
			}

			/* free memory allocated for mail */
			osMailFree(mtr_get_mail(Mail_Motor_id), pRMail);
		}

		if (motor.mode_state.moveMode == TASK_MOTOR_MANUAL_id)
			homing_state = 0;

//		hal_motor_get_motor_state(&motor.pan_state, &motor.tilt_state);

		if ((_bus_recv_timestamp > 0) && (osKernelSysTick() >= (_bus_recv_timestamp + BUS_MAX_TIMEOUT))) {
			_bus_recv_timestamp = 0;

			motor.pan_command.power_enable = 0;
			motor.pan_command.spd_man_in_c = 0;
			motor.pan_command.spd_trk_in_c = 0;
			motor.pan_command.pos_bal_in_c = 0;
			motor.pan_command.pos_hom_in_c = 0;

			motor.tilt_command.power_enable = 0;
			motor.tilt_command.spd_man_in_c = 0;
			motor.tilt_command.spd_trk_in_c = 0;
			motor.tilt_command.pos_bal_in_c = 0;
			motor.tilt_command.pos_hom_in_c = 0;
			LOG("\r\n\r\n\r\n\r\n");
		}
		else {
			if (motor.mode_state.moveMode != TASK_MOTOR_MANUAL_id) {
				motor.pan_command.power_enable = 1;
				motor.tilt_command.power_enable = 1;
			}
		}

		if (osKernelSysTick() >= _bus_send_timer) {
			_bus_send_timer = osKernelSysTick() + 100;

			mtr_send_to_bus();
			LOG("[O]state=%02X,%04X,%02X,%04X\r\n", motor.pan_state.power, motor.pan_state.statusword,
					motor.tilt_state.power, motor.tilt_state.statusword);
//			LOG("[O]pan state=%X,%X\r\n", motor.pan_state.power, motor.pan_state.statusword);
//			LOG("[O]tilt state=%X,%X\r\n", motor.tilt_state.power, motor.tilt_state.statusword);
		}

#if RTOS_USE_STACK_HIGH_WATER==1
		if (osKernelSysTick() >= _stack_highwater) {
			_stack_highwater = osKernelSysTick() + 1000;

			LOG("\t\tshw= %d\r\n", uxTaskGetStackHighWaterMark2(NULL));
		}
#endif	//if RTOS_USE_STACK_HIGH_WATER==1

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), T_Motor_id, 0);
	}
	/* USER CODE END t_motor */
}

static void _stop_timer()
{
	osTimerStop(mtr_get_timer(Tim_Motor_id));
	g_timer_in_used = 0;
	tim_user = timer_used_by_none;
}

static void t_motor_delete_task(const uint8_t living_task)
{
	if (living_task == TASK_MOTOR_MANUAL_id) {
		vTaskDelete(TMoveManualHandle);
		LOG("delete t_motor_manual task\r\n");
	}
	else if (living_task == TASK_MOTOR_TRAVEL_id) {
		vTaskDelete(TMoveTravelHandle);
		LOG("delete t_motor_travel task\r\n");
	}
	else if (living_task == TASK_MOTOR_STAB_id) {
		vTaskDelete(TMoveStabHandle);
		LOG("delete t_motor_stab task\r\n");
	}
	else if (living_task == TASK_MOTOR_TRACK_id) {
		vTaskDelete(TMoveTrackHandle);
		LOG("delete t_motor_track task\r\n");
	}
	else if (living_task == TASK_MOTOR_MEMORY_id) {
		vTaskDelete(TMoveMemoryHandle);
		LOG("delete t_motor_memory task\r\n");
	}
	else if (living_task == TASK_MOTOR_HOMING_id) {
		vTaskDelete(TMoveHomingHandle);
		LOG("delete t_motor_homing task\r\n");
	}
}

static void t_motor_change_task(const uint8_t task_tobe_revived)
{
	taskENTER_CRITICAL();

	if (task_tobe_revived == TASK_MOTOR_MANUAL_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveManual */
		osThreadDef(TMoveManual, t_motor_manual, osPriorityBelowNormal, 0, 512);
		TMoveManualHandle = osThreadCreate(osThread(TMoveManual), NULL);
		motor.current_task_id = task_tobe_revived;
	}
	else if (task_tobe_revived == TASK_MOTOR_TRAVEL_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveTravel */
		osThreadDef(TMoveTravel, t_motor_travel, osPriorityBelowNormal, 0, 512);
		TMoveTravelHandle = osThreadCreate(osThread(TMoveTravel), NULL);
		motor.current_task_id = task_tobe_revived;
	}
	else if (task_tobe_revived == TASK_MOTOR_STAB_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveStab */
		osThreadDef(TMoveStab, t_motor_stab, osPriorityBelowNormal, 0, 512);
		TMoveStabHandle = osThreadCreate(osThread(TMoveStab), NULL);
		motor.current_task_id = task_tobe_revived;
	}
	else if (task_tobe_revived == TASK_MOTOR_TRACK_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveTrack */
		osThreadDef(TMoveTrack, t_motor_track, osPriorityBelowNormal, 0, 512);
		TMoveTrackHandle = osThreadCreate(osThread(TMoveTrack), NULL);
		motor.current_task_id = task_tobe_revived;
	}
	else if (task_tobe_revived == TASK_MOTOR_MEMORY_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveMemory */
		osThreadDef(TMoveMemory, t_motor_memory, osPriorityBelowNormal, 0, 512);
		TMoveMemoryHandle = osThreadCreate(osThread(TMoveMemory), NULL);
		motor.current_task_id = task_tobe_revived;
	}
	else if (task_tobe_revived == TASK_MOTOR_HOMING_id) {
		if (motor.current_task_id != task_tobe_revived)
			t_motor_delete_task(motor.current_task_id);
		/* definition and creation of TMoveHoming */
		osThreadDef(TMoveHoming, t_motor_homing, osPriorityBelowNormal, 0, 512);
		TMoveHomingHandle = osThreadCreate(osThread(TMoveHoming), NULL);
		motor.current_task_id = task_tobe_revived;
	}

	taskEXIT_CRITICAL();LOG("[M]current task: %d\r\n", motor.current_task_id);
}

static void t_motor_init(const uint32_t delay)
{
	osDelay(delay);
	motor.pan_state.pos = motor.tilt_state.pos = 0;

	tim_user = timer_used_by_init;
	g_init_timeout = osKernelSysTick() + MTR_INIT_TIMEOUT;
	osTimerStart(mtr_get_timer(Tim_Motor_id), 500);
	while (hal_motor_init(motor.enable) != HAL_OK) {
		LOG_E("motor init failed!\r\n");
		osDelay(10);
	}
	_stop_timer();
}

uint8_t mtr_set_power(Motor_t *mtr)
{
	uint8_t ret = 0;

	if (is_motor_az_enable()) {
		if (mtr->pan_command.power_enable != mtr->pan_state.power) {
			mtr->pan_state.power = mtr->pan_command.power_enable;
			hal_motor_set_pan_power(mtr->pan_state.power);
			ret |= MTR_AZ_ENABLE;
		}
	}

	if (is_motor_el_enable()) {
		if (mtr->tilt_command.power_enable != mtr->tilt_state.power) {
			mtr->tilt_state.power = mtr->tilt_command.power_enable;
			hal_motor_set_tilt_power(mtr->tilt_state.power);
			ret |= MTR_EL_ENABLE;
		}
	}

	return ret;
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

	LOG("PT= %ld,%ld\r\n", pos_mail->param.motor.position.pan, pos_mail->param.motor.position.tilt);

	/* send mail queue*/
	osMailPut(mtr_get_mail(Mail_Bus_id), pos_mail);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int32_t mtr_speed_to_c(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	int64_t atas = ((int64_t) x - in_min) * ((int64_t) out_max - out_min);
	int32_t bawah = in_max - in_min;

	if (bawah > 0)
		return (uint32_t) ((atas / bawah) + out_min);

	return 1;
}

/* mapping max_command(1000000) to RWS_MOTOR_PAN_MAX_SPEED(100000) */
static int32_t _man_pan_speed_to_c(int32_t speed)
{
	return mtr_speed_to_c(speed, -1000000, 1000000, -RWS_MOTOR_PAN_MAX_SPEED, RWS_MOTOR_PAN_MAX_SPEED);
}

/* mapping max_command(1000000) to RWS_MOTOR_TILT_MAX_SPEED (172032L) */
static int32_t _man_tilt_speed_to_c(int32_t speed)
{
	return mtr_speed_to_c(speed, -1000000L, 1000000L, -RWS_MOTOR_TILT_MAX_SPEED, RWS_MOTOR_TILT_MAX_SPEED);
}
