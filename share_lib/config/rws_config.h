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
#define RWS_MOTOR_ID		0x300
#define RWS_PANEL_ID		0x310
#define RWS_OPTRONIK_ID		0x320
#define RWS_IMU_ID			0x330

/** @defgroup RWS_data_length_code RWS ID Code
 * @{
 */
#define RWS_MOTOR_DATA_LENGTH		FDCAN_DLC_BYTES_20
#define RWS_PANEL_DATA_LENGTH		FDCAN_DLC_BYTES_20
#define RWS_OPTRONIK_DATA_LENGTH	FDCAN_DLC_BYTES_8
#define RWS_IMU_DATA_LENGTH			FDCAN_DLC_BYTES_9

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

#define RWS_MOTOR_PAN_MAX_SPEED				350UL
#define RWS_MOTOR_TILT_MAX_SPEED			100000UL

#endif /* RWS_CONFIG_H_ */
