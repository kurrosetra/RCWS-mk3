/*
 * noptel_ll.c
 *
 *  Created on: Jan 4, 2022
 *      Author: miftakur
 */

#include "noptel.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#if DEBUG_NOPTEL==1
#include <stdio.h>
#endif	//if DEBUG_NOPTEL==1

static osMessageQId q_noptelHandle;
/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/
static HAL_StatusTypeDef _NOPTEL_prepare_packet(Noptel_interface_t *iface)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* create queue */
	osMessageQDef(q_noptel, NOPTEL_INPUT_BUFFER_SIZE, uint8_t);
	q_noptelHandle = osMessageCreate(osMessageQ(q_noptel), NULL);

	ret = HAL_UARTEx_ReceiveToIdle_DMA(iface->port_fd, (uint8_t*) iface->ibuf,
	NOPTEL_INPUT_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(iface->port_fd->hdmarx, DMA_IT_HT);

	return ret;
}

static HAL_StatusTypeDef _NOPTEL_get_packet(Noptel_interface_t *iface)
{
	HAL_StatusTypeDef ret = HAL_ERROR;

	osEvent event = osMessageGet(q_noptelHandle, NOPTEL_SERIAL_LONG_WAIT);
	if (event.status == osEventMessage) {
		iface->ilen = event.value.v;
#if DEBUG_NOPTEL==1
		printf("%dB -> ", iface->ilen);
		for ( int i = 0; i < iface->ilen; i++ ) {
			printf("[%02X]", iface->ibuf[i]);
		}
		printf("\r\n");
#endif	//if DEBUG_NOPTEL==1
		ret = HAL_OK;
	}
	else if (event.status == osEventTimeout) {
#if DEBUG_NOPTEL==1
		printf("timeout\r\n");
#endif	//if DEBUG_NOPTEL==1
	}

	/* delete queue */
	osMessageDelete(q_noptelHandle);

	return ret;
}

HAL_StatusTypeDef _NOPTEL_write_packet_data(Noptel_interface_t *iface, const uint8_t get_reply)
{
	if (get_reply != 0) {
		_NOPTEL_prepare_packet(iface);
		HAL_UART_Transmit(iface->port_fd, iface->oBuf, iface->olen, NOPTEL_SERIAL_WAIT);

		return _NOPTEL_get_packet(iface);
	}
	else
		HAL_UART_Transmit(iface->port_fd, iface->oBuf, iface->olen, NOPTEL_SERIAL_WAIT);

	return HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == huart7.Instance) {
		if (q_noptelHandle != NULL)
			osMessagePut(q_noptelHandle, Size, 0);
	}
}

