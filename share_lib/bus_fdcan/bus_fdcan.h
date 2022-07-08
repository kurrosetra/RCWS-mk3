/*
 * bus_fdcan.h
 *
 *  Created on: Nov 5, 2021
 *      Author: miftakur
 */

#ifndef BUS_FDCAN_H_
#define BUS_FDCAN_H_

#include "rws_config.h"
#include "main.h"

//typedef enum
//{
//	CAN_BUS_STANDARD,
//	CAN_BUS_EXTENDED,
//} CAN_BUS_ID_TYPE_e;

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
	uint8_t data[64];
	uint32_t lastTimestamp;
} Bus_Tx_Buffer_t;

void FDCAN_RX_Filter_Motor(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);
void FDCAN_RX_Filter_Panel(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);
void FDCAN_RX_Filter_Optronik(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);

void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan);
void FDCAN_TX_Config(FDCAN_TxHeaderTypeDef *txHeader, const uint32_t txID,
		const uint32_t dataLength);

uint8_t FDCAN_Convert_Datalength(const uint32_t datalength);

#endif /* BUS_FDCAN_H_ */
