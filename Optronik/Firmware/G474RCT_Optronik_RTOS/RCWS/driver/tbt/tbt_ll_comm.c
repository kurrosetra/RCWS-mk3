/*
 * tbt_low_level_comm.c
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */

#include "tbt.h"
#include "usart.h"
#include "cmsis_os.h"

#if DEBUG_TBT==1
#include <stdio.h>
#endif	//if DEBUG_TBT==1

static osMessageQId q_thermalHandle;

uint32_t _TBT_write_packet_data(TBT_interface_t *iface)
{
	if (HAL_UART_Transmit(iface->port_fd, iface->oBuf, TBT_OUTPUT_BUFFER_SIZE, TBT_SERIAL_WAIT)
			!= HAL_OK)
		return TBT_FAILURE;

//	return _TBT_get_packet(iface);

#if DEBUG_TBT==1
	printf("send: ");
	for ( int i = 0; i < TBT_OUTPUT_BUFFER_SIZE; i++ ) {
		printf("[%02X] ", (int) iface->oBuf[i] & 0xFF);
	}
	printf("\r\n");
#endif	//if DEBUG_TBT==1

	return TBT_SUCCESS;
}

uint32_t _TBT_get_packet(TBT_interface_t *iface)
{
	uint32_t ret = TBT_FAILURE;

	/* create queue */
	osMessageQDef(q_thermal, TBT_INPUT_BUFFER_SIZE, uint8_t);
	q_thermalHandle = osMessageCreate(osMessageQ(q_thermal), NULL);

//	/* enable uart in receive interrupt */
//	__HAL_UART_ENABLE_IT(iface->port_fd, UART_IT_RXNE);

	uint32_t pos = 0;
	uint32_t timer = HAL_GetTick() + TBT_SERIAL_LONG_WAIT;

#if DEBUG_TBT==1
	printf("\r\nrecv: ");
#endif	//if DEBUG_TBT==1
	while (HAL_GetTick() < timer) {
		osEvent event = osMessageGet(q_thermalHandle, TBT_SERIAL_WAIT);
		if (event.status == osEventMessage) {
			iface->ibuf[pos] = (uint8_t) event.value.v;
#if DEBUG_TBT==1
			printf("[%02X]", iface->ibuf[pos]);
#endif	//if DEBUG_TBT==1

			if (iface->ibuf[pos] == TBT_TERMINATOR)
				break;
			else {
				if (++pos >= TBT_INPUT_BUFFER_SIZE) {
					goto exit;
				}
			}
		}

	}
	iface->bytes = pos + 1;
	if (HAL_GetTick() >= timer) {
#if DEBUG_TBT==1
		printf("timeout!\r\n");
#endif	//if DEBUG_TBT==1
		ret = TBT_FAILURE;
	}
	else {
#if DEBUG_TBT==1
		printf("\r\n");
#endif	//if DEBUG_TBT==1
		ret = TBT_SUCCESS;
	}

	exit:
	/* disable uart in receive interrupt */
	__HAL_UART_DISABLE_IT(iface->port_fd, UART_IT_RXNE);
	/* delete queue */
	osMessageDelete(q_thermalHandle);

	return ret;
}

void UART5_IRQHandler(void)
{
	uint32_t isrflags = READ_REG(huart5.Instance->ISR);
	/* UART in mode Receiver ---------------------------------------------------*/
	if ((isrflags & USART_ISR_RXNE) != 0U) {

		uint8_t recv_byte = (huart5.Instance->RDR & 0xFF);
		osMessagePut(q_thermalHandle, recv_byte, 0);
	}
}

