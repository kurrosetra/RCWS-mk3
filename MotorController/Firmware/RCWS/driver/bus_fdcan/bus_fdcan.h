/*
 * bus.h
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#ifndef DRIVER_BUS_FDCAN_BUS_H_
#define DRIVER_BUS_FDCAN_BUS_H_

#include "main.h"

typedef struct
{
	uint32_t id;
	uint8_t len;
	uint8_t data[64];
	uint8_t counter;
	uint32_t lastTimestamp;
} BUS_Rx_Buffer_t;

typedef struct
{
	uint8_t data[64];
	uint32_t lastTimestamp;
} BUS_Tx_Buffer_t;

void bus_init();
HAL_StatusTypeDef bus_send_motor_status(uint8_t *data);
HAL_StatusTypeDef bus_send_weapon_status(uint8_t *data);
HAL_StatusTypeDef bus_send_position(const int32_t pan, const int32_t tilt);
HAL_StatusTypeDef bus_send_imu(const int16_t yaw, const int32_t pitch, const int32_t roll);
void bus_rx_callback(BUS_Rx_Buffer_t *buffer);
uint8_t FDCAN_Convert_Datalength(const uint32_t datalength);

#endif /* DRIVER_BUS_FDCAN_BUS_H_ */
