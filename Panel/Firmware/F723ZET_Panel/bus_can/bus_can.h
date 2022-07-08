/*
 * bus_can.h
 *
 *  Created on: Jun 10, 2022
 *      Author: 62812
 */

#ifndef DRIVER_BUS_CAN_BUS_CAN_H_
#define DRIVER_BUS_CAN_BUS_CAN_H_

#include "rws_config.h"
#include "main.h"

typedef enum
{
	CAN_BUS_STANDARD,
	CAN_BUS_EXTENDED,
} CAN_BUS_ID_TYPE_e;

typedef struct
{
	uint32_t id;
	uint8_t idType;
	uint8_t len;
	uint8_t data[64];
	uint8_t counter;
	uint32_t lastTimestamp;
} Bus_Rx_Buffer_t;

typedef struct
{
	uint32_t id;
	uint32_t datalength;
	uint8_t data[8];
	uint32_t lastTimestamp;
} Bus_Tx_Buffer_t;

HAL_StatusTypeDef CAN_RX_Filter(CAN_HandleTypeDef *hcan, const uint32_t filterIndex, const uint32_t filterId,
		const uint32_t filterMask);
void CAN_RX_Filter_Panel(CAN_HandleTypeDef *hcan, const uint32_t filterIndex);
void CAN_RX_Filter_Motor(CAN_HandleTypeDef *hcan, const uint32_t filterIndex);
void CAN_RX_Filter_Optronik(CAN_HandleTypeDef *hcan, const uint32_t filterIndex);

void CAN_Config(CAN_HandleTypeDef *hcan);
void CAN_Tx_Config(CAN_TxHeaderTypeDef *txHeader);

#endif /* DRIVER_BUS_CAN_BUS_CAN_H_ */
