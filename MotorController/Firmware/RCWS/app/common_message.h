/*
 * common_message.h
 *
 *  Created on: May 25, 2022
 *      Author: 62812
 */

#ifndef APP_COMMON_MESSAGE_H_
#define APP_COMMON_MESSAGE_H_

#include "rws_command_state.h"

typedef enum
{
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_LOW_POWER_RESET,
	RESET_CAUSE_WINDOW_WATCHDOG_RESET,
	RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
	RESET_CAUSE_SOFTWARE_RESET,
	RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
	RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
	RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_e;

typedef enum
{
	T_Bus_id = 0b1,
	T_Motor_id = 0b10,
	T_Weapon_id = 0b100,
	T_Motor_Ext_id = 0b1000,
	T_Manager_id = 0b1111,
} Task_ID;

typedef enum
{
	Mail_Bus_id,
	Mail_Motor_id,
	Mail_Motor_Ext_id,
	Mail_Weapon_id,
} Mail_ID;

typedef enum
{
	Q_MANAGER_NOTIF,
} Queue_ID;

typedef enum
{
	Tim_Weapon_id,
	Tim_Motor_id,
} Timer_ID;

typedef enum
{
	Mutex_Weapon_id,
	Mutex_Motor_id,
	Mutex_motor_ext_id,
} Mutex_ID;

/*
 **************
 * BUS PARAMs *
 **************
 */
typedef enum
{
	Bus_Sender_Motor_State_id,
	Bus_Sender_Motor_Position_id,
	Bus_Sender_Weapon_State_id,
	Bus_Sender_Imu_id,
} Bus_Sender_id;

typedef struct
{
	Bus_Sender_id sender_id;
	union
	{
		struct
		{
			Body_weapon_status_t state;
			uint16_t munition_counter;
		} weapon;
		union
		{
			struct
			{
				Body_movement_mode_t mode;
				Body_motor_status_t pan_state;
				Body_motor_status_t tilt_state;
			} state;
			struct
			{
				int32_t pan;
				int32_t tilt;
			} position;
		} motor;
		struct
		{
			int16_t yaw;
			int32_t pitch;
			int32_t roll;
		} imu;
	} param;
} MAIL_Bus_t;

/*
 ****************
 * MOTOR PARAMs *
 ****************
 */

typedef enum
{
	Motor_Sender_Bus_Mode_id,
	Motor_Sender_Ext_id,
	Motor_Sender_Sensor_id,
} Motor_Sender_Id_e;

typedef struct
{
	Motor_Sender_Id_e sender_id;
	union
	{
		/* FROM BUS */
		Panel_motor_command_t mode;

		/* FROM MOTOR EXT */
		struct
		{
			uint8_t movementModeActive :4; /* movement mode, see Movement_mode_e */
			uint8_t ready_to_be_terminated :1;
			uint8_t resv_bit :3;
		} __attribute__ ((packed)) ext;

		/* FROM SENSOR */
		union
		{
			struct
			{
				struct
				{
					uint8_t pulse :1;	//0-> LOW; 1-> HIGH
					uint8_t direction :1;	//0-> RIGHT; 1-> LEFT
					uint8_t resv_bit :6;
				} __attribute__ ((packed)) state;
				int32_t pan_pos;
			} az_zero;
		} sensor;
	} param;
} MAIL_Motor_t;

typedef enum
{
	Motor_Ext_Sender_Mode_id,
	Motor_Ext_Sender_Value_id,
	Motor_Ext_Sender_Offset_id,
	Motor_Ext_Sender_Balistic_id,
} Motor_Ext_Sender_Id_e;

typedef struct
{
	Motor_Ext_Sender_Id_e sender_id;
	union
	{
		struct
		{
			uint8_t motor_enable :1;
			uint8_t balistic_active :1;
			uint8_t ready_to_be_terminated :1;
			uint8_t reservedBit :5;
		} __attribute__ ((packed)) mode;

		struct
		{
			int32_t pan;
			int32_t tilt;
		} value;

		struct
		{
			Optronik_camera_state_t trk_state;
			int16_t trk_x;
			int16_t trk_y;
		} track;
	} param;
} MAIL_Motor_Ext_t;

/*
 *****************
 * WEAPON PARAMs *
 *****************
 */

typedef enum
{
	Weapon_Sender_Bus_id,

	Weapon_Sender_Sensor_Trigger_id,
	Weapon_Sender_Sensor_Cock_id,

} Weapon_Sender_Id_e;

typedef struct
{
	Panel_weapon_command_t mode;
	uint16_t shoot_limit;
} Weapon_Bus_Command_t;

typedef struct
{
	Weapon_Sender_Id_e sender_id;
	union
	{
		/* FROM BUS */
		Weapon_Bus_Command_t command;

		/* FROM SENSOR */
		union
		{
			struct
			{
				/* TODO add trigger's pulse & munition sensor here */
				uint8_t pulse_on :1;
				uint8_t pulse_off :1;
				uint8_t shoot_limit_reached :1;
				uint8_t resv_bit :5;
			} __attribute__ ((packed)) trigger;
			struct
			{
				/* TODO add cock sensor here */
				uint8_t erect_reached :1;
				uint8_t erect_timeout :1;
				uint8_t retract_reached :1;
				uint8_t retract_timeout :1;
				uint8_t resv_bit :4;
			} __attribute__ ((packed)) cock;
		} sensor;
	} param;
} MAIL_Weapon_t;

#endif /* APP_COMMON_MESSAGE_H_ */
