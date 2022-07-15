/*
 * ingenia_ll_can.c
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

/*
 * ingenia_ll_comm.c
 *
 *  Created on: Jan 5, 2022
 *      Author: miftakur
 */

#include "ingenia.h"

#if INGENIA_BUS_USED==INGENIA_USE_CAN
#include "can.h"

#if DEBUG_INGENIA_ENABLE==1
#include <stdio.h>

/*** Internal Const Values, Macros ***/
#	define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILENAME__, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)

#endif	//if DEBUG_INGENIA_ENABLE==1

HAL_StatusTypeDef Ingenia_begin(CAN_INTERFACE_HANDLETYPE *hcan)
{
	/* init slave buffer */
	Ingenia_node_init();

	/*##-2- Configure the CAN Filter ###########################################*/
	CAN_FilterTypeDef sFilterConfig;

	/* NMT filter */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x80 << 5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x880 << 5;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x700 << 5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x780 << 5;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
		/* filter configuration error */
		return HAL_ERROR;
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(hcan) != HAL_OK) {
		/* Start Error */
		return HAL_ERROR;
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		return HAL_ERROR;
	}

	return HAL_OK;
}

void Ingenia_prepare_tx_header(Servo_t *servo)
{
	servo->txHeader.StdId = 0x1;
	servo->txHeader.ExtId = 0x01;
	servo->txHeader.RTR = CAN_RTR_DATA;
	servo->txHeader.IDE = CAN_ID_STD;
	servo->txHeader.DLC = CAN_DATA_MAX;
	servo->txHeader.TransmitGlobalTime = DISABLE;
}

HAL_StatusTypeDef _INGENIA_write_data(Servo_t *servo, const uint32_t id, const uint8_t len,
		const uint8_t *buf)
{
	uint32_t txMailBox;
	servo->txHeader.StdId = id;
	servo->txHeader.DLC = len;

	return HAL_CAN_AddTxMessage(servo->hcan, &servo->txHeader, (uint8_t*) buf, &txMailBox);
}

HAL_StatusTypeDef _INGENIA_get_data(CAN_INTERFACE_HANDLETYPE *hcan, const uint32_t rxLocation,
		uint32_t *_id_recv, uint8_t *buf)
{
	CAN_RxHeaderTypeDef rxHeader;

	if (HAL_CAN_GetRxMessage(hcan, rxLocation, &rxHeader, buf) == HAL_OK) {
		if (rxHeader.IDE == CAN_ID_STD) {
			*_id_recv = rxHeader.StdId;

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}


#endif	//if INGENIA_BUS_USED==INGENIA_USE_CAN

