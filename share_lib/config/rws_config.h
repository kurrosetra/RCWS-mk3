/*
 * config.h
 *
 *  Created on: Sep 10, 2021
 *      Author: miftakur
 */

#ifndef RWS_CONFIG_H_
#define RWS_CONFIG_H_

#include "main.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/** @defgroup RWS_ID_code RWS ID Code
 * @{
 */
#define RWS_PANEL_ID					0x110
#define 	RWS_PANEL_CMD_ID				0x111
#define 	RWS_PANEL_MAN_ID				0x112
#define 	RWS_PANEL_TRK_ID				0x113
#define 	RWS_PANEL_BAL_ID				0x114
#define		RWS_PANEL_HOM_ID				0x115
#define RWS_MOTOR_ID					0x120
#define 	RWS_MOTOR_STATUS_ID				0x121
#define 	RWS_MOTOR_POS_ID				0x122
#define 	RWS_MOTOR_IMU_ID				0x123
#define 	RWS_WEAPON_STATUS_ID			0x124
#define RWS_OPTRONIK_ID					0x130
#define 	RWS_OPTRONIK_LRF_ID				0x131
#define 	RWS_OPTRONIK_CAM_ID				0x132
#define 	RWS_OPTRONIK_IMU_ID				0x133


typedef union
{
	float f;
	int32_t i32;
	uint32_t u32;
	int16_t i16[2];
	uint16_t u16[2];
	int8_t i8[4];
	uint8_t u8[4];
} Rws_Union_u;

#define BUS_MAX_TIMEOUT			500

#endif /* RWS_CONFIG_H_ */
