/*
 * button_config.h
 *
 *  Created on: Dec 7, 2021
 *      Author: miftakur
 */

#ifndef BUS_BUTTON_CONFIG_H_
#define BUS_BUTTON_CONFIG_H_

#include "main.h"

typedef struct
{
	uint8_t trig_enable :1; /* CMD trigger enable (0: trigger disable, 1: trigger enable) */
	uint8_t firing_mode :2; /* CMD firing mode, see firing_mode_e */
	uint8_t spd_select :2; /* CMD manual speed max, spd_selector_e */

	uint8_t cam_select :1; /* CMD camera selection (0: day camera, 1: thermal camera) */
	uint8_t focus_mode :2; /* CMD camera focus, see focus_mode_e */
	uint8_t lrf_enable :1;
	uint8_t lrf_start :1;

	uint8_t lrf_man_mode :2;
	uint8_t target_select :2;

	uint8_t cock_enable :1;
	uint8_t cock_start :1;
} __attribute__ ((packed)) sModeGpioInput_t;

typedef struct
{
	uint8_t power_state :1;
	uint8_t trig_enable :1;
	uint8_t lrf_enable :1;
	uint8_t cam_select :1;
	uint8_t reservedBit4_7 :4;
} __attribute__ ((packed)) sModeGpioOutput_t;

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

typedef enum
{
	JS_ADC_MIN = 680,
	JS_ADC_MID = 910,
	JS_ADC_MAX = 1120,
	JS_ADC_HYST = 15
} joystick_adc_value_e;

#define button_update_timeout_ms	50UL

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
	uint8_t cockMoving :1;
	uint8_t resvBit :6;
} __attribute__ ((packed)) Bus_weapon_status_t;

typedef struct
{
	uint8_t triggerEnable :1;
	uint8_t cockEnable :1;
	uint8_t cockStartMoving :1;
	uint8_t munitionCounterReset :1;
	uint8_t resvBit :4;
} __attribute__ ((packed)) Panel_weapon_command_t;

typedef struct
{
	uint8_t panEnable :1;
	uint8_t tiltEnable :1;
	uint8_t panZeroReached :1;
	uint8_t tiltUpwardReached :1;
	uint8_t tiltDownwardReached :1;
	uint8_t panFault :1;
	uint8_t tiltFault :1;
	uint8_t resvBit :1;
} __attribute__ ((packed)) Bus_motor_status_t;

typedef enum
{
	MOVE_MODE_MAN = 0,
	MOVE_MODE_TRAVEL,
	MOVE_MODE_STAB,
	MOVE_MODE_TRACK,
	MOVE_MODE_MEMORY,
} Panel_movement_mode_e;

typedef struct
{
	uint8_t panEnable :1;
	uint8_t tiltEnable :1;
	uint8_t moveMode :4;
	uint8_t resvBit :2;
} __attribute__ ((packed)) Panel_movement_mode_t;

#endif /* BUS_BUTTON_CONFIG_H_ */
