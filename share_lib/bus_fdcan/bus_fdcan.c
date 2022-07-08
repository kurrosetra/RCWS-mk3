/*
 * bus_fdcan.c
 *
 *  Created on: Nov 5, 2021
 *      Author: miftakur
 */
#include "fdcan.h"
#include "bus_fdcan.h"

const uint8_t dlc2len[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

__weak void FDCAN_RX_Interrupt_Config(FDCAN_HandleTypeDef *hfdcan)
{
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  Configures the FDCAN.
 * @param  None
 * @retval None
 */
__weak void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan)
{
	/* Configure and enable Tx Delay Compensation, required for BRS mode.
	 TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
	 TdcFilter default recommended value: 0 */
	if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) {
		/* Configure and enable Tx Delay Compensation, required for BRS mode.
		 TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
		 TdcFilter default recommended value: 0 */
		if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, (hfdcan->Init.DataTimeSeg1 * hfdcan->Init.DataPrescaler), 0)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) {
			Error_Handler();
		}
	}

	/* Activate Rx FIFO 0 new message notification */
	FDCAN_RX_Interrupt_Config(hfdcan);

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
		Error_Handler();
	}

}

uint8_t FDCAN_Convert_Datalength(const uint32_t datalength)
{
	return dlc2len[datalength >> 16];
}

void FDCAN_RX_Filter_Motor(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex)
{
	FDCAN_FilterTypeDef sFilterConfig;
	uint32_t filterId1 = RWS_MOTOR_ID, filterId2 = RWS_MOTOR_ID + 0xF;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	sFilterConfig.FilterID1 = filterId1;
	sFilterConfig.FilterID2 = filterId2;
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}
}

void FDCAN_RX_Filter_Panel(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex)
{
	FDCAN_FilterTypeDef sFilterConfig;
	uint32_t filterId1 = RWS_PANEL_ID, filterId2 = RWS_PANEL_ID + 0xF;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	sFilterConfig.FilterID1 = filterId1;
	sFilterConfig.FilterID2 = filterId2;
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}
}

void FDCAN_RX_Filter_Optronik(FDCAN_HandleTypeDef *hfdcan, const uint32_t filterIndex)
{
	FDCAN_FilterTypeDef sFilterConfig;
	uint32_t filterId1 = RWS_OPTRONIK_ID, filterId2 = RWS_OPTRONIK_ID + 0xF;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filterIndex;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
	sFilterConfig.FilterID1 = filterId1;
	sFilterConfig.FilterID2 = filterId2;
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  Formatting Tx Header.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  txID indicates Tx Header ID.
 * @param  dataLength FDCAN Tx Data Length, default FDCAN_DLC_BYTES_8 8 Bytes to be send.
 *         This parameter can be any combination of @arg FDCAN_data_length_code.
 * @retval None
 */
void FDCAN_TX_Config(FDCAN_TxHeaderTypeDef *txHeader, const uint32_t txID, const uint32_t dataLength)
{
	/* Prepare Tx Header */
	txHeader->Identifier = txID;
	txHeader->IdType = FDCAN_STANDARD_ID;
	txHeader->TxFrameType = FDCAN_DATA_FRAME;
	txHeader->DataLength = dataLength;
	txHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader->BitRateSwitch = FDCAN_BRS_ON;
	txHeader->FDFormat = FDCAN_FD_CAN;
	txHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader->MessageMarker = 0;
}
