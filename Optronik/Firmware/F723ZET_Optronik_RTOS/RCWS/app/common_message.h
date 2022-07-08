/*
 * common_message.h
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#ifndef APP_COMMON_MESSAGE_H_
#define APP_COMMON_MESSAGE_H_

#include "bus_button_config.h"

typedef enum
{
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_LOW_POWER_RESET,
	RESET_CAUSE_WINDOW_WATCHDOG_RESET,
	RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
	RESET_CAUSE_SOFTWARE_RESET,
	RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
//	RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
//	RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_e;

typedef enum
{
	Q_CAMERA_NOTIF,
	Q_LRF_NOTIF,
	Q_MANAGER_NOTIF,
} QUEUE_ID;

typedef enum
{
	M_CAMERA_STATE,
	M_LRF_STATE,
} MUTEX_ID;

typedef enum
{
	T_Bus_Recv_id = 0b1,
	T_Camera_id = 0b10,
	T_Lrf_id = 0b100,
	T_Bus_Send_id = 0b1000,
	T_Manager_id=0b1111,
} Task_ID;

typedef enum
{
	Tim_Camera_Notif_id,
	Tim_Camera_Timeout_id,
	Tim_Lrf_Notif_id,
	Tim_Lrf_Timeout_id,
} Timer_ID;

typedef struct
{
	Panel_camera_command_t command;
	Optronik_camera_state_t state;
	uint8_t busy;
} Camera_t;

typedef struct
{
	uint8_t lrf_enable;
	Optronik_Lrf_state_t value;
} Lrf_State_t;

typedef struct
{
	Panel_lrf_imu_command_t command;
	Lrf_State_t state;
	uint8_t busy;
} Lrf_t;

#endif /* APP_COMMON_MESSAGE_H_ */
