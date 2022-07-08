/*
 * hal_motor.c
 *
 *  Created on: Jun 12, 2022
 *      Author: 62812
 */

#include <stdlib.h>

#include "hal_motor.h"
#include "fdcan.h"

#include "rws_config.h"
#include "driver/bus_fdcan/bus_fdcan.h"
//#include "driver/ingenia/ingenia.h"

#define DEBUG_HAL_MOTOR				0

#if (DEBUG_MOTOR==1)|(DEBUG_HAL_MOTOR==1)
#include <stdio.h>

/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, (osKernelSysTick()%10000UL),__FILE_NAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s_Err:%d] " str, __FILE_NAME__,__LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_MOTOR==1

#define is_motor_az_enable()	((mc.mtr_enable & MTR_AZ_ENABLE) == MTR_AZ_ENABLE)
#define is_motor_el_enable()	((mc.mtr_enable & MTR_EL_ENABLE) == MTR_EL_ENABLE)

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;

	uint8_t mtr_enable;

	Servo_t mtrAzi;
	Servo_t mtrEle;
} Motor_control_t;

Motor_control_t mc;

static void motor_param_init(Servo_t *servo)
{
	Ingenia_write_nmt(servo, NMT_START_REMOTE_NODE);
	uint32_t motor_pan_max_accel = (RWS_MOTOR_PAN_MAX_SPEED * 3) / 2;
//	uint32_t motor_tilt_max_accel = (RWS_MOTOR_TILT_MAX_SPEED * 3) / 2;
	uint32_t motor_tilt_max_accel = RWS_MOTOR_TILT_MAX_SPEED;

	/* set mode to profile position */
	Ingenia_setModeOfOperation(servo, DRIVE_MODE_PROFILE_POSITION);

	LOG("mode=position mode\r\n");

	if (servo->_u8Node == MTR_AZ_ID) {
		/* set max motor speed */
		Ingenia_write_sdo_u32(servo, 0x6080, 0, RWS_MOTOR_PAN_MAX_SPEED);
		/* set max profile acceleration */
		Ingenia_write_sdo_u32(servo, 0x6083, 0, motor_pan_max_accel);
		/* set max profile de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6084, 0, motor_pan_max_accel);
		/* set max profile quick stop de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6085, 0, motor_pan_max_accel * 2);
	}
	else if (servo->_u8Node == MTR_EL_ID) {
		/* set max motor speed */
		Ingenia_write_sdo_u32(servo, 0x6080, 0, RWS_MOTOR_TILT_MAX_SPEED);
		/* set max profile acceleration */
		Ingenia_write_sdo_u32(servo, 0x6083, 0, motor_tilt_max_accel);
		/* set max profile de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6084, 0, motor_tilt_max_accel);
		/* set max profile quick stop de-acceleration */
		Ingenia_write_sdo_u32(servo, 0x6085, 0, motor_tilt_max_accel * 2);
	}

	/* set TPDO4 event timer */
//	Ingenia_write_sdo_u16(servo, 0x1803, 5, 10);
	Ingenia_write_sdo_u16(servo, 0x1803, 10, 20);

}

HAL_StatusTypeDef hal_motor_init(const uint8_t enable)
{
	LOG("Motor init...\r\n");

	mc.hfdcan = &hfdcan2;
	mc.mtr_enable = enable;

//	if (HAL_FDCAN_ActivateNotification(mc.hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
//		return HAL_ERROR;

	Ingenia_begin(mc.hfdcan);

	if (is_motor_az_enable()) {
		LOG("enabling pan motor ...\r\n");
		if (Ingenia_init(&mc.mtrAzi, mc.hfdcan, MTR_AZ_ID) != HAL_OK) {
			LOG("init MTR_AZI failed!\r\n");
			return HAL_ERROR;
		}

		LOG("pan param init...\r\n");
		motor_param_init(&mc.mtrAzi);
		LOG("done!\r\n");
	}

	if (is_motor_el_enable()) {
		LOG("enabling tilt motor ...\r\n");
		if (Ingenia_init(&mc.mtrEle, mc.hfdcan, MTR_EL_ID) != HAL_OK) {
			LOG("init MTR_ELE failed!\r\n");
			return HAL_ERROR;
		}
		else
			LOG("tilt node=0x%02X\r\n", mc.mtrEle._u8Node);

		LOG("tilt param init\r\n");
		motor_param_init(&mc.mtrEle);
		LOG("done!\r\n");
	}

	if ((is_motor_az_enable() != 0) || (is_motor_el_enable() != 0)) {
		uint32_t _mtrInitialTimer = osKernelSysTick() + MTR_INIT_TIMEOUT;
		uint8_t _mtr_ready = 0;
		uint8_t _mtr_finished = 0;
		if (is_motor_az_enable())
			bitSet(_mtr_finished, 0);
		if (is_motor_el_enable())
			bitSet(_mtr_finished, 1);

		LOG("start angle det process\r\n");
		while (osKernelSysTick() < _mtrInitialTimer) {
			if (is_motor_az_enable()) {
				if (!bitRead(_mtr_ready, 0))
					Ingenia_enableMotor(&mc.mtrAzi);

				if (mc.mtrAzi._isInitialAngleDeterminationProcessFinished == 1) {
					bitSet(_mtr_ready, 0);
					Ingenia_disableMotor(&mc.mtrAzi);
				}
			}

			if (is_motor_el_enable()) {
				if (!bitRead(_mtr_ready, 1))
					Ingenia_enableMotor(&mc.mtrEle);

				if (mc.mtrEle._isInitialAngleDeterminationProcessFinished == 1) {
					bitSet(_mtr_ready, 1);
					Ingenia_disableMotor(&mc.mtrEle);
				}
			}

			if (_mtr_ready == _mtr_finished)
				break;
			else {
				for ( int i = 0; i < 300; i++ ) {
					printf(".");
					osDelay(50);
					if (is_motor_az_enable()) {
						/* get statusword from can interrupt */
						Ingenia_getDecodedStatusWord(&mc.mtrAzi);
						if (mc.mtrAzi._isInitialAngleDeterminationProcessFinished)
							bitSet(_mtr_ready, 0);
					}

					if (is_motor_el_enable()) {
						/* get statusword from can interrupt */
						Ingenia_getDecodedStatusWord(&mc.mtrEle);
						if (mc.mtrEle._isInitialAngleDeterminationProcessFinished)
							bitSet(_mtr_ready, 1);
					}

					/* if all motor already initialized */
					if (_mtr_ready == _mtr_finished)
						break;
				}
			}

			if (is_motor_az_enable())
				Ingenia_disableMotor(&mc.mtrAzi);
			if (is_motor_el_enable())
				Ingenia_disableMotor(&mc.mtrEle);

			osDelay(100);
		}
		printf("\r\n");
	}

	LOG("Motor Ready!\r\n");

	return HAL_OK;
}

static void hal_motor_stop(Servo_t *servo)
{
	Ingenia_setTargetPositionVelocity(servo, servo->posActual, 0, 1, 0, 0);
}

static HAL_StatusTypeDef hal_motor_speed(Servo_t *servo, const int32_t spd)
{
	uint32_t _abs_speed = abs(spd);

	int32_t _pos = servo->posActual;
	int32_t _max_pos = 0;

	_max_pos = _abs_speed / 2;

	if (spd == 0)
		hal_motor_stop(servo);
	else {
		if (spd < 0)
			_pos -= _max_pos;
		else
			_pos += _max_pos;

		Ingenia_setTargetPositionVelocity(servo, _pos, _abs_speed, 1, 0, 0);
	}
	return HAL_OK;
}

void hal_motor_set_pan_power(const uint8_t act)
{
	if (is_motor_az_enable()) {
		if (act != 0)
			Ingenia_enableMotor(&mc.mtrAzi);
		else
			Ingenia_disableMotor(&mc.mtrAzi);
	}
	LOG("\r\nPAN_power: %d\r\n\r\n", act);
}

void hal_motor_set_tilt_power(const uint8_t act)
{
	if (is_motor_el_enable()) {
		if (act == 1)
			Ingenia_enableMotor(&mc.mtrEle);
		else
			Ingenia_disableMotor(&mc.mtrEle);
	}
	LOG("\r\nTILT_power: %d\r\n\r\n", act);
}

uint8_t hal_motor_set_position(const int32_t pan_pos_in_c, const int32_t tilt_pos_in_c)
{
	uint8_t ret = 0;
	if (is_motor_az_enable()) {
		if (Ingenia_getDecodedStatusWord(&mc.mtrAzi) == STATUS_OPERATION_ENABLED) {
//			LOG("motor pan moving\r\n");
			Ingenia_setTargetPositionVelocity(&mc.mtrAzi, pan_pos_in_c, (RWS_MOTOR_PAN_MAX_SPEED / 2), 1, 0, 0);
			ret++;
		}
	}

	if (is_motor_el_enable()) {
		if (Ingenia_getDecodedStatusWord(&mc.mtrEle) == STATUS_OPERATION_ENABLED) {
//			LOG("motor tilt moving\r\n");
			Ingenia_setTargetPositionVelocity(&mc.mtrEle, tilt_pos_in_c, (RWS_MOTOR_TILT_MAX_SPEED / 2), 1, 0, 0);
			ret++;
		}
	}

	return ret;
}

uint8_t hal_motor_set_speed(const int32_t pan_speed, const int32_t tilt_speed)
{
	static uint8_t _pan_idle_counter = 0;
	static uint8_t _tilt_idle_counter = 0;
	uint8_t ret = 0;

	if (is_motor_az_enable()) {
		if (Ingenia_getDecodedStatusWord(&mc.mtrAzi) == STATUS_OPERATION_ENABLED) {
			if (hal_motor_speed(&mc.mtrAzi, pan_speed) == HAL_OK)
				ret++;
			_pan_idle_counter = 0;
		}
		else {
			/* reduce upload rate */
			if (++_pan_idle_counter > 10) {
				_pan_idle_counter = 0;
				hal_motor_set_pan_power(0);
				ret++;
			}
		}
	}

	if (is_motor_el_enable()) {
		if (Ingenia_getDecodedStatusWord(&mc.mtrEle) == STATUS_OPERATION_ENABLED) {
			if (hal_motor_speed(&mc.mtrEle, tilt_speed) == HAL_OK)
				ret++;
			_tilt_idle_counter = 0;
		}
		else {
			/* reduce upload rate */
			if (++_tilt_idle_counter > 10) {
				_tilt_idle_counter = 0;
				hal_motor_set_tilt_power(0);
				ret++;
			}
		}
	}

	return ret;
}

uint8_t hal_motor_is_fault(const uint16_t statusword)
{
	return bitRead(statusword, 3);
}

float hal_motor_get_voltage(const uint32_t volt)
{
	return (float) volt / 1000;
}

float hal_motor_get_current(const int16_t current, const float max)
{
	return (float) current * max / 1000;
}

float hal_motor_get_pan_pos_deg()
{
	int32_t _pos = mc.mtrAzi.posActual % RWS_MOTOR_PAN_FULL_REV_IN_C;

	return (float) _pos / RWS_MOTOR_PAN_FULL_REV_IN_C;
}

float hal_motor_get_tilt_pos_deg()
{

	return 0.0f;
}

void hal_motor_get_motor_state(Servo_Value_t *p, Servo_Value_t *t)
{
	static uint32_t p_state_counter = 0;
	static uint32_t p_value_counter = 0;
	static uint32_t t_state_counter = 0;
	static uint32_t t_value_counter = 0;

	if (p_state_counter != mc.mtrAzi.tpdo3_counter) {
		p_state_counter = mc.mtrAzi.tpdo3_counter;
		p->statusword = mc.mtrAzi.statusword;
		p->current_value = mc.mtrAzi.current;
		p->dc_link_voltage = mc.mtrAzi.voltage;
	}

	if (p_value_counter != mc.mtrAzi.tpdo4_counter) {
		p_state_counter = mc.mtrAzi.tpdo4_counter;
		p->speed = mc.mtrAzi.veloActual;
		p->pos = mc.mtrAzi.posActual;
	}

	if (t_state_counter != mc.mtrEle.tpdo3_counter) {
		t_state_counter = mc.mtrEle.tpdo3_counter;
		t->statusword = mc.mtrEle.statusword;
		t->current_value = mc.mtrEle.current;
		t->dc_link_voltage = mc.mtrEle.voltage;
	}

	if (t_value_counter != mc.mtrEle.tpdo4_counter) {
		t_state_counter = mc.mtrEle.tpdo4_counter;
		t->speed = mc.mtrEle.veloActual;
		t->pos = mc.mtrEle.posActual;
	}

}

void mtr_error_callback()
{
	MX_FDCAN2_Init();
	hal_motor_init(mc.mtr_enable);
	LOG("\r\n\nBUS MTR ERROR\r\n\n");
}

void EXTI9_5_IRQHandler(void)
{
	/* EXTI line interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(LIM_AZ_ZERO_Pin) != 0x00u) {
		__HAL_GPIO_EXTI_CLEAR_IT(LIM_AZ_ZERO_Pin);

		/* TODO lim_az_zero_callback */
//		HAL_GPIO_EXTI_Callback(LIM_AZ_ZERO_Pin);
//		static uint32_t lim_az_zero_timestamp = 0;
//
//		if ((HAL_GetTick() - lim_az_zero_timestamp) >= 50) {
//			lim_az_zero_timestamp = HAL_GetTick() + 50;
//
//			MAIL_Weapon_t *pTMail;
//			// allocate memory; receiver must be free it
//			pTMail = osMailAlloc(mtr_get_mail(T_Weapon_id), 0);
//			pTMail->sender_id = Weapon_Sender_Sensor_Trigger_id;
//			*(uint8_t*) &pTMail->param.sensor.trigger = 0;
//
//			if (HAL_GPIO_ReadPin(LIM_AZ_ZERO_GPIO_Port, LIM_AZ_ZERO_Pin) == GPIO_PIN_RESET)
//				pTMail->param.sensor.trigger.pulse_on = 1;
//			else
//				pTMail->param.sensor.trigger.pulse_off = 1;
//
//			osMailPut(mtr_get_mail(T_Weapon_id), pTMail);
//		}
	}
}

void Ingenia_tpdo_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;
	uint32_t _id_node = 0;
	int _pos = 0, _velo = 0;
	int _voltage = 0, _current = 0, _statusword = 0;

	if (can_buffer_available(buffer) > 0) {
		can_buffer_read(buffer, &data);
		if ((data.id & COB_TPDO4) == COB_TPDO4) {
			_id_node = data.id - COB_TPDO4;

			for ( int i = 0; i < 4; i++ ) {
				_pos |= (int32_t) data.rxData[i] << (8 * i);
				_velo |= (int32_t) data.rxData[i + 4] << (8 * i);
			}

			if (_id_node == mc.mtrAzi._u8Node) {
				mc.mtrAzi.posActual = _pos;
				mc.mtrAzi.veloActual = _velo;
				mc.mtrAzi.tpdo4_counter++;
			}
			else if (_id_node == mc.mtrEle._u8Node) {
				mc.mtrEle.posActual = _pos;
				mc.mtrEle.veloActual = _velo;
				mc.mtrEle.tpdo4_counter++;
			}
		}
		else if ((data.id & COB_TPDO3) == COB_TPDO3) {
			_id_node = data.id - COB_TPDO3;

			for ( int i = 0; i < 4; i++ )
				_voltage |= (int32_t) data.rxData[i] << (8 * i);
			_current = (int16_t) data.rxData[5] << 8 | data.rxData[4];
			_statusword = (uint16_t) data.rxData[7] << 8 | data.rxData[6];

			if (_id_node == mc.mtrAzi._u8Node) {
				mc.mtrAzi.voltage = _voltage;
				mc.mtrAzi.current = _current;
				mc.mtrAzi.statusword = _statusword;
				mc.mtrAzi.tpdo3_counter++;
			}
			else if (_id_node == mc.mtrEle._u8Node) {
				mc.mtrEle.voltage = _voltage;
				mc.mtrEle.current = _current;
				mc.mtrEle.statusword = _statusword;
				mc.mtrEle.tpdo3_counter++;
			}
		}
	}
}

void Ingenia_nmt_callback(CAN_Buffer_t *buffer)
{
	CAN_Data_t data;

	if (can_buffer_available(buffer) > 0) {
		can_buffer_peek(buffer, &data);

		LOG("NMT_id= %03lX\r\n", data.id);
		for ( int i = 0; i < data.len; i++ ) {
			printf("%02X ", data.rxData[i]);
		}
		printf("\r\n");
	}
}

