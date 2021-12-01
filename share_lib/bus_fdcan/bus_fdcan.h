/*
 * bus_fdcan.h
 *
 *  Created on: Nov 5, 2021
 *      Author: miftakur
 */

#ifndef BUS_FDCAN_H_
#define BUS_FDCAN_H_

#include <rws_config.h>
#include "main.h"

typedef struct
{
	uint32_t id;
	FDCAN_RxHeaderTypeDef rxHeader;
	uint8_t data[64];
	uint8_t counter;
	uint32_t lastTimestamp;
} Bus_Rx_Buffer_t;

typedef struct
{
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t data[64];
	uint32_t lastTimestamp;
} Bus_Tx_Buffer_t;

void FDCAN_RX_Filter_Motor(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);
void FDCAN_RX_Filter_Panel(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);
void FDCAN_RX_Filter_Optronik(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);
void FDCAN_RX_Filter_Imu(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex);

void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan);
void FDCAN_TX_Config(FDCAN_TxHeaderTypeDef *txHeader, const uint32_t txID,
		const uint32_t dataLength);
void FDCAN_RX_Interrupt_Config(FDCAN_HandleTypeDef *hfdcan);

uint8_t FDCAN_Convert_Datalength(const uint32_t datalength);

#endif /* BUS_FDCAN_H_ */
