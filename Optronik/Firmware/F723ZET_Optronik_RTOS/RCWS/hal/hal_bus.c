/*
 * hal_bus.c
 *
 *  Created on: Jan 10, 2022
 *      Author: miftakur
 */

#include <stdio.h>
#include <string.h>

#include "hal_bus.h"
#include "can.h"

typedef struct
{
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailBox;

//	Bus_Tx_Buffer_t sendOptronik;
	Bus_Rx_Buffer_t recvPanel;
} Bus_t;

static Bus_t bus;

void hal_bus_init()
{
	bus.hcan = &hcan1;

#if DEBUG_BUS==1
	printf("can bus init ...\r\n");
#endif	//if DEBUG_BUS==1

	CAN_RX_Filter(bus.hcan, 0, RWS_PANEL_CMD_ID, 0x7FF);
	bus.recvPanel.id = RWS_PANEL_CMD_ID;
	CAN_Config(bus.hcan);
	CAN_Tx_Config(&bus.txHeader);

#if DEBUG_BUS==1
	printf("done!\r\n");
#endif	//if DEBUG_BUS==1

}

HAL_StatusTypeDef hal_bus_send(Bus_Tx_Buffer_t *buffer)
{
	bus.txHeader.StdId = buffer->id;
	bus.txHeader.DLC = buffer->datalength;
	return HAL_CAN_AddTxMessage(bus.hcan, &bus.txHeader, buffer->data, &bus.txMailBox);
}

HAL_StatusTypeDef hal_lrf_send(Bus_Tx_Buffer_t *buffer)
{
	bus.txHeader.StdId = buffer->id;
	bus.txHeader.DLC = buffer->datalength;
	return HAL_CAN_AddTxMessage(bus.hcan, &bus.txHeader, buffer->data, &bus.txMailBox);
}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t _counter = 0;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	Bus_Rx_Buffer_t *pTMail;

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		if ((RxHeader.StdId == RWS_PANEL_CMD_ID) || (RxHeader.IDE == CAN_ID_STD)) {
			// allocate memory; receiver must be free it
			pTMail = osMailAlloc(opt_get_bus_mail(), 0);
			pTMail->lastTimestamp = osKernelSysTick();
			pTMail->id = RxHeader.StdId;
			pTMail->idType = CAN_BUS_STANDARD;
			pTMail->len = RxHeader.DLC;
			memcpy(pTMail->data, RxData, RxHeader.DLC);
			pTMail->counter = _counter++;

			/* send mail queue*/
			osMailPut(opt_get_bus_mail(), pTMail);
		}

	}

}

/**
 * @brief  Error CAN callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	/* Bus-off error */
	if (hcan->ErrorCode & HAL_CAN_ERROR_BOF != 0) {
		//	g_fdcan_bus_busOff_error = 1;
#if DEBUG_BUS==1
		printf("\t\t\tbus error!\r\n");
#endif	//if DEBUG_BUS==1
	}
}
