/*
 * bus_can.c
 *
 *  Created on: Jun 10, 2022
 *      Author: 62812
 */

#include "bus_can.h"
#include "can.h"

HAL_StatusTypeDef CAN_RX_Filter(CAN_HandleTypeDef *hcan, const uint32_t filterIndex, const uint32_t filterId,
		const uint32_t filterMask)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	CAN_FilterTypeDef sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/

	sFilterConfig.FilterIdHigh = ((filterId << 5) | (filterId >> (32 - 5))) & 0xFFFF;  // STID[10:0] & EXTID[17:13]
	sFilterConfig.FilterIdLow = (filterId >> (11 - 3)) & 0xFFF8;  // EXID[12:5] & 3 Reserved bits
	sFilterConfig.FilterMaskIdHigh = ((filterMask << 5) | (filterMask >> (32 - 5))) & 0xFFFF;
	sFilterConfig.FilterMaskIdLow = (filterMask >> (11 - 3)) & 0xFFF8;

	sFilterConfig.FilterBank = filterIndex;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x0000;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0x0000;
//	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	ret = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
	if (ret != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	return ret;
}

void CAN_RX_Filter_Panel(CAN_HandleTypeDef *hcan, const uint32_t filterIndex)
{
	CAN_RX_Filter(hcan, filterIndex, RWS_PANEL_ID, 0x7F0);
}

void CAN_RX_Filter_Motor(CAN_HandleTypeDef *hcan, const uint32_t filterIndex)
{
	CAN_RX_Filter(hcan, filterIndex, RWS_MOTOR_ID, 0x7F0);
}

void CAN_RX_Filter_Optronik(CAN_HandleTypeDef *hcan, const uint32_t filterIndex)
{
	CAN_RX_Filter(hcan, filterIndex, RWS_OPTRONIK_ID, 0x7F0);
}

void CAN_Config(CAN_HandleTypeDef *hcan)
{
	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(hcan) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}
}

void CAN_Tx_Config(CAN_TxHeaderTypeDef *txHeader)
{
	/*##-5- Configure Transmission process #####################################*/
	txHeader->StdId = 0x111;
	txHeader->ExtId = 0x01;
	txHeader->RTR = CAN_RTR_DATA;
	txHeader->IDE = CAN_ID_STD;
	txHeader->DLC = 8;
	txHeader->TransmitGlobalTime = DISABLE;
}

