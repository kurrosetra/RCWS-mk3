/*
 * hal_motor.h
 *
 *  Created on: Jun 12, 2022
 *      Author: 62812
 */

#ifndef DRIVER_HAL_MOTOR_HAL_MOTOR_H_
#define DRIVER_HAL_MOTOR_HAL_MOTOR_H_

#include "app/common.h"
#include "driver/ingenia/ingenia.h"

#define MTR_AZ_ENABLE						0b01
#define MTR_EL_ENABLE						0b10
#define MTR_AZ_ID							0x20
#define MTR_EL_ID							0x21
#define RWS_MOTOR_PAN_MAX_SPEED				100000
#define RWS_MOTOR_TILT_MAX_SPEED			172032	/* 2521 RPM */
#define MTR_INIT_TIMEOUT					60000
#define RWS_MOTOR_PAN_FULL_REV_IN_C			417747
#define RWS_MOTOR_PAN_C_TO_DEG(c)			((float)c*0.0008617656f)
#define RWS_MOTOR_PAN_DEG_TO_C(deg)			((int32_t)(deg*1160.4083f))
#define RWS_MOTOR_TILT_C_TO_DEG(c)			((float)c*0.000525247f)
#define RWS_MOTOR_TILT_DEG_TO_C(deg)		((int32_t)(deg*1903.866f))

typedef struct
{
	uint8_t power;
	uint16_t statusword;
	int16_t current_value;
	uint32_t dc_link_voltage;
	int32_t pos;
	int32_t speed;
} Servo_Value_t;

typedef struct
{
	uint8_t power_enable;

	int32_t spd_man_in_c;
	int32_t spd_trk_in_c;
	int32_t pos_bal_in_c;
	int32_t pos_hom_in_c;
} Servo_Command_t;

typedef struct
{
	uint8_t enable; /* which servo is activated */

	Panel_motor_command_t mode_command;
	Servo_Command_t pan_command;
	Servo_Command_t tilt_command;

	Body_movement_mode_t mode_state;
	Servo_Value_t pan_state;
	Servo_Value_t tilt_state;
} Motor_t;

#define is_motor_az_enable(enable)	((enable & MTR_AZ_ENABLE) == MTR_AZ_ENABLE)
#define is_motor_el_enable(enable)	((enable & MTR_EL_ENABLE) == MTR_EL_ENABLE)

HAL_StatusTypeDef hal_motor_init(const uint8_t enable);
void mtr_error_callback();
void hal_motor_update_motor_state(Motor_t *mtr, const uint8_t power_update);
//void hal_motor_get_motor_state(Servo_Value_t *p, Servo_Value_t *t);
void hal_motor_set_pan_power(const uint8_t act);
void hal_motor_set_tilt_power(const uint8_t act);
int32_t hal_motor_pan_speed_to_c(int32_t speed);
int32_t hal_motor_tilt_speed_to_c(int32_t speed);
uint8_t hal_motor_set_speed(const int32_t pan_speed_in_c, const int32_t tilt_speed_in_c);
uint8_t hal_motor_set_position(const int32_t pan_pos_in_c, const int32_t tilt_pos_in_c);
uint8_t hal_motor_is_fault(const uint16_t statusword);
float hal_motor_get_voltage(const uint32_t volt);
float hal_motor_get_current(const int16_t current, const float max);
float hal_motor_get_pan_pos_deg();
float hal_motor_get_tilt_pos_deg();

#endif /* DRIVER_HAL_MOTOR_HAL_MOTOR_H_ */
