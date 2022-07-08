/*
 * busPanel.h
 *
 *  Created on: Dec 13, 2021
 *      Author: miftakur
 */

#ifndef BUSPANEL_H_
#define BUSPANEL_H_

#include "main.h"
#include "bus_fdcan.h"
#include "bus_button_config.h"

//typedef struct
//{
//	uint8_t panEnable :1;
//	uint8_t tiltEnable :1;
//	uint8_t panZeroReached :1;
//	uint8_t tiltUpwardReached :1;
//	uint8_t tiltDownwardReached :1;
//	uint8_t resvBit :3;
//} __attribute__ ((packed)) Bus_motor_status_t;

//typedef struct
//{
//	uint8_t triggerEnable :1;
//	uint8_t cockMoving :1;
//	uint8_t resvBit :6;
//} __attribute__ ((packed)) Bus_weapon_status_t;

typedef struct
{
	Bus_motor_status_t motorStatus;
	Bus_weapon_status_t weaponStatus;
	uint16_t munitionCounter;
	int32_t panPosition;
	int32_t tiltPosition;
	int32_t panVelocity;
	int32_t tiltVelocity;

	uint16_t panVoltage;
	int16_t panCurrent;
	uint16_t tiltVoltage;
	int16_t tiltCurrent;
} Motor_Weapon_Status_t;

//enum
//{
//	MOVE_MODE_MAN = 0,
//	MOVE_MODE_TRAVEL = 1,
//	MODE_MODE_STAB = 2,
//	MOVE_MODE_TRACK = 3
//} Panel_movement_mode_e;
//
//typedef struct
//{
//	uint8_t panEnable :1;
//	uint8_t tiltEnable :1;
//	uint8_t moveMode :2;
//	uint8_t resvBit :4;
//} __attribute__ ((packed)) Panel_movement_mode_t;

//typedef struct
//{
//	uint8_t cameraActive :1; /* camera active id, see cam_selector_e */
//	uint8_t zoom :2; /* zoom command, see zoom_focus_mode_e */
//	uint8_t focus :2; /* focus command, see zoom_focus_mode_e */
//	uint8_t resvBit :2;
//	uint8_t stabilizerActive :1;
//} __attribute__ ((packed)) Panel_camera_command_t;

//typedef struct
//{
//	uint8_t triggerEnable :1;
//	uint8_t cockStartMoving :1;
//	uint8_t munitionCounterReset :1;
//	uint8_t resvBit :5;
//} __attribute__ ((packed)) Panel_weapon_command_t;

//typedef struct
//{
//	uint8_t lrfEnable :1;
//	uint8_t lrfStart :1;
//	uint8_t lrfContinousMode :1;
//	uint8_t lrfPointerActive :1;
//	uint8_t imuFastRate :1;
//	uint8_t resvBit :3;
//} __attribute__ ((packed)) Panel_lrf_imu_command_t;

typedef struct
{
	Panel_movement_mode_t movementMode;
	Panel_camera_command_t cameraCommand;
	Panel_weapon_command_t weaponCommand;
	Panel_lrf_imu_command_t lrfImuCommand;
	int32_t panSpeedCommand;
	int32_t tiltSpeedCommand;
	int32_t panSpeedCorrection;
	int32_t tiltSpeedCorrection;
} Panel_button_command_t;

//typedef struct
//{
//	uint8_t cameraActive :1; /* OPT camera active (0: day camera, 1: thermal camera) */
//	uint8_t zoom :2; /* OPT zoom level, see OPT_zoom_level_e*/
//	uint8_t reservedBit :5;
//} __attribute__ ((packed)) Optronik_camera_state_t;

typedef struct
{
	uint8_t counter;
	uint16_t d[3];
} Optronik_Lrf_t;

typedef struct
{
	Optronik_camera_state_t camera;
	Optronik_Lrf_t lrf;
} Optronik_Status_t;

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;

	Bus_Tx_Buffer_t sendPanel;

	Bus_Rx_Buffer_t recvMotor;
	Bus_Rx_Buffer_t recvOptronik;
	Bus_Rx_Buffer_t recvImu;

	Motor_Weapon_Status_t motorWeaponStatus;
	Panel_button_command_t panelButtonCommand;
	Optronik_Status_t optronikStatus;
} Bus_t;
extern Bus_t bus;

void bus_init(FDCAN_HandleTypeDef *hfdcan);
void bus_handler();

#endif /* BUSPANEL_H_ */
