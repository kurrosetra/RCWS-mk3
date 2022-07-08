/*
 * button_config.h
 *
 *  Created on: Dec 7, 2021
 *      Author: miftakur
 */

#ifndef BUS_BUTTON_CONFIG_H_
#define BUS_BUTTON_CONFIG_H_

#include "main.h"

typedef enum
{
	SPD_SELECT_MIN,
	SPD_SELECT_MID,
	SPD_SELECT_MAX
} spd_selector_e;

typedef enum
{
	FIRING_MODE_CONT,
	FIRING_MODE_3,
	FIRING_MODE_1
} firing_mode_e;

typedef enum
{
	ZF_STOP,
	ZF_IN,
	ZF_OUT
} zoom_focus_mode_e;

typedef enum
{
	TARGET_NONE,
	TARGET_NEXT,
	TARGET_PREV
} target_selector_e;

typedef enum
{
	LRF_MAN_NONE,
	LRF_MAN_UP,
	LRF_MAN_DOWN
} lrf_man_mode_e;

typedef enum
{
	CAM_SELECT_DAY,
	CAM_SELECT_THERMAL
} cam_selector_e;

typedef enum
{
	OPT_ZOOM_LEVEL_1, /* observe mode */
	OPT_ZOOM_LEVEL_2, /* combat mode */
	OPT_ZOOM_LEVEL_3 /* tele mode */
} OPT_zoom_level_e;

typedef struct
{
	uint8_t cameraActive :1; /* OPT camera active (0: day camera, 1: thermal camera) */
	uint8_t zoomLevel :2; /* OPT zoom level, see OPT_zoom_level_e*/
	uint8_t reservedBit :5;
} __attribute__ ((packed)) Optronik_camera_state_t;

typedef struct
{
	uint8_t counter;
	uint16_t d[3];
} Optronik_Lrf_state_t;

typedef struct
{
	uint8_t cameraActive :1; /* camera active id, see cam_selector_e */
	uint8_t zoom :2; /* zoom command, see zoom_focus_mode_e */
	uint8_t focus :2; /* focus command, see zoom_focus_mode_e */
	uint8_t resvBit :2;
	uint8_t stabilizerActive :1;
} __attribute__ ((packed)) Panel_camera_command_t;

typedef struct
{
	uint8_t lrfEnable :1;
	uint8_t lrfStart :1;
	uint8_t lrfContinousMode :1;
	uint8_t lrfPointerActive :1;
	uint8_t resvBit :4;
} __attribute__ ((packed)) Panel_lrf_imu_command_t;

typedef struct
{
	uint8_t triggerEnable :1;
	uint8_t cockEnable :1;
	uint8_t cockStartMoving :1;
	uint8_t munitionCounterReset :1;
	uint8_t resvBit :4;
} __attribute__ ((packed)) Panel_weapon_command_t;

typedef enum
{
	MOVE_MODE_MAN,
	MOVE_MODE_TRAVEL,
	MOVE_MODE_STAB,
	MOVE_MODE_TRACK,
	MOVE_MODE_MEMORY,
	MOVE_MODE_HOMING,
} Movement_mode_e;

typedef struct
{
	uint8_t motorEnable :1;
	uint8_t moveMode :4;
	uint8_t modeAbort :1;
	uint8_t ballisticActive :1;
	uint8_t reservedBit :1;
} __attribute__ ((packed)) Panel_motor_mode_t;

typedef struct
{
	uint8_t moveMode :4;
	uint8_t moveModeEnded :1;
	uint8_t moveModeFault :1;
	uint8_t ballisticActive :1;
	uint8_t reservedBit :1;
} __attribute__ ((packed)) Body_motor_mode_t;

typedef struct
{
	uint8_t motorEnable :1;
	uint8_t initialAngleSuccess :1;
	uint8_t limitReached :1;
	uint8_t fault :2;
	uint8_t reservedBit :3;
} __attribute__ ((packed)) Body_motor_status_t;

typedef struct
{
	uint8_t triggerEnable :1;
	uint8_t cockEnable :1;
	uint8_t cockMoving :1;
	uint8_t resvBit :5;
} __attribute__ ((packed)) Body_weapon_status_t;

#endif /* BUS_BUTTON_CONFIG_H_ */
