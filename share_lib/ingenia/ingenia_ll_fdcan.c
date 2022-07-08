/*
 * ingenia_ll_comm.c
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#include "ingenia.h"

#if INGENIA_BUS_USED==INGENIA_USE_FDCAN
#include "fdcan.h"

#if DEBUG_INGENIA_ENABLE==1
#include <stdio.h>

/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILENAME__, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)

#endif	//if DEBUG_INGENIA_ENABLE==1

//static const uint8_t dlc2len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
static const uint32_t len2dlc[16] = { FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6,
FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16,
FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64 };

//static uint8_t _Convert_Datalength(const uint32_t datalength)
//{
//	return dlc2len[datalength >> 16];
//}
//
//static uint32_t _Convert_Datalength2(const uint8_t datalength)
//{
//	return len2dlc[datalength];
//}

HAL_StatusTypeDef Ingenia_begin(CAN_INTERFACE_HANDLETYPE *hcan)
{
	/* init node slave buffer */
	Ingenia_node_init();

	/*##-2- Configure the CAN Filter ###########################################*/
	FDCAN_FilterTypeDef sFilterConfig;
	uint32_t filterIndex, filterID1, filterID2;

	/* NMT filter */
	filterIndex = 0;
	filterID1 = 0x81;
	filterID2 = 0xFF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TPDO1 filter */
	filterIndex = 1;
	filterID1 = 0x181;
	filterID2 = 0x1FF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TPDO2 filter */
	filterIndex = 2;
	filterID1 = 0x281;
	filterID2 = 0x2FF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TPDO3 filter */
	filterIndex = 3;
	filterID1 = 0x381;
	filterID2 = 0x3FF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TPDO4 filter */
	filterIndex = 4;
	filterID1 = 0x481;
	filterID2 = 0x4FF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TSDO filter */
	filterIndex = 5;
	filterID1 = 0x581;
	filterID2 = 0x5FF;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* NMT Error filter */
	filterIndex = 6;
	filterID1 = 0x701;
	filterID2 = 0x77F;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterID1;
	sFilterConfig.FilterID2 = filterID2;
	if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

//	/* Configure global filter:
//	 Filter all remote frames with STD and EXT ID
//	 Reject non matching frames with STD ID and EXT ID */
//	if (HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
//	FDCAN_FILTER_REMOTE) != HAL_OK) {
//		Error_Handler();
//	}

	/*##-3- Activate CAN RX notification #######################################*/
	if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

//	if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
//		Error_Handler();
//	}

	/*##-4- Start the CAN peripheral ###########################################*/
	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(hcan) != HAL_OK) {
		Error_Handler();
	}

	return HAL_OK;
}

void Ingenia_prepare_tx_header(Servo_t *servo)
{
	servo->txHeader.IdType = FDCAN_STANDARD_ID;
	servo->txHeader.TxFrameType = FDCAN_DATA_FRAME;
	servo->txHeader.DataLength = FDCAN_DLC_BYTES_8;
	servo->txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	servo->txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	servo->txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	servo->txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	servo->txHeader.MessageMarker = 0;
}

HAL_StatusTypeDef _INGENIA_write_data(Servo_t *servo, const uint32_t id, const uint8_t len, const uint8_t *buf)
{
	servo->txHeader.Identifier = id;
	servo->txHeader.DataLength = len2dlc[len];

	return HAL_FDCAN_AddMessageToTxFifoQ(servo->hcan, &servo->txHeader, (uint8_t*) buf);
}

HAL_StatusTypeDef _INGENIA_get_data(CAN_INTERFACE_HANDLETYPE *hcan, const uint32_t rxLocation, uint32_t *_id_recv,
		uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef rxHeader;

	if (HAL_FDCAN_GetRxMessage(hcan, rxLocation, &rxHeader, buf) == HAL_OK) {
		if (rxHeader.IdType == FDCAN_STANDARD_ID) {
			*_id_recv = rxHeader.Identifier;

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

#endif	//if INGENIA_BUS_USED==INGENIA_USE_FDCAN
