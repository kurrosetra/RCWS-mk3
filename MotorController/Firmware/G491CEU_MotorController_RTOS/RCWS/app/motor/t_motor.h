/*
 * motor.h
 *
 *  Created on: May 27, 2022
 *      Author: 62812
 */

#ifndef APP_MOTOR_MOTOR_H_
#define APP_MOTOR_MOTOR_H_

#include "app/common.h"
#include "driver/hal_motor/hal_motor.h"

#if DEBUG_MOTOR==1
#define DEBUG_MOTOR_MANUAL			1
#define DEBUG_MOTOR_TRAVEL			1
#define DEBUG_MOTOR_STAB			1
#define DEBUG_MOTOR_TRACK			1
#define DEBUG_MOTOR_MEMORY			1
#define DEBUG_MOTOR_HOMING			1
#endif	//if DEBUG_MOTOR==1

extern Motor_t motor;
extern uint8_t homing_state;

uint8_t mtr_set_power(Motor_t *mtr);

#endif /* APP_MOTOR_MOTOR_H_ */
