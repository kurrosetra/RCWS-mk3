/*
 * hal_bus.c
 *
 *  Created on: Jan 10, 2022
 *      Author: miftakur
 */

#include <stdio.h>
#include <string.h>

#include "hal_bus.h"
#include "fdcan.h"

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_TxHeaderTypeDef txHeader;
	Bus_Tx_Buffer_t sendOptronik;
	Bus_Rx_Buffer_t recvPanel;
} Bus_t;

static Bus_t bus;

void hal_bus_init()
{
	bus.hfdcan = &hfdcan1;

#if DEBUG_BUS==1
	printf("fdcan bus init ...\r\n");
#endif	//if DEBUG_BUS==1

	FDCAN_RX_Filter_Panel(bus.hfdcan, 0);
	bus.recvPanel.id = RWS_PANEL_ID;
	FDCAN_TX_Config(&bus.txHeader, RWS_OPTRONIK_ID, RWS_OPTRONIK_DATA_LENGTH);
	FDCAN_Config(bus.hfdcan);

#if DEBUG_BUS==1
	printf("done!\r\n");
#endif	//if DEBUG_BUS==1

}

HAL_StatusTypeDef hal_bus_send(const uint8_t *data)
{
	memcpy(bus.sendOptronik.data, data, FDCAN_Convert_Datalength(RWS_OPTRONIK_DATA_LENGTH));

	return HAL_FDCAN_AddMessageToTxFifoQ(bus.hfdcan, &bus.txHeader,
			bus.sendOptronik.data);

}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static uint8_t _counter = 0;
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[64];
	Bus_Rx_Buffer_t *pTMail;

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			if ((RxHeader.Identifier == RWS_PANEL_ID) || (RxHeader.IdType == FDCAN_STANDARD_ID)) {
				// allocate memory; receiver must be free it
				pTMail = osMailAlloc(opt_get_bus_mail(), 0);
				pTMail->lastTimestamp = osKernelSysTick();
				pTMail->id = RxHeader.Identifier;
				pTMail->idType = CAN_BUS_STANDARD;
				pTMail->len = FDCAN_Convert_Datalength(RxHeader.DataLength);
				memcpy(pTMail->data, RxData, FDCAN_Convert_Datalength(RxHeader.DataLength));
				pTMail->counter = _counter++;

				/* send mail queue*/
				osMailPut(opt_get_bus_mail(), pTMail);
			}
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
//	g_fdcan_bus_busOff_error = 1;
#if DEBUG_BUS==1
	printf("\t\t\tbus error!\r\n");
#endif	//if DEBUG_BUS==1

}
