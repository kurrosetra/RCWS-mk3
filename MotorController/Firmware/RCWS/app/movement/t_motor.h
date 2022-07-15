/*
 * t_motor.h
 *
 *  Created on: Jul 12, 2022
 *      Author: 62812
 */

#ifndef APP_MOVEMENT_T_MOTOR_H_
#define APP_MOVEMENT_T_MOTOR_H_

#include "app/common.h"
#include "hal/motor/hal_motor.h"

#define DEBUG_MOTOR_MANUAL			0
#define DEBUG_MOTOR_TRAVEL			1
#define DEBUG_MOTOR_STAB			1
#define DEBUG_MOTOR_TRACK			1
#define DEBUG_MOTOR_MEMORY			1
#define DEBUG_MOTOR_HOMING			0

extern Motor_t motor;
//extern uint8_t homing_state;
//uint8_t mtr_set_power(Motor_t *mtr);


#endif /* APP_MOVEMENT_T_MOTOR_H_ */
