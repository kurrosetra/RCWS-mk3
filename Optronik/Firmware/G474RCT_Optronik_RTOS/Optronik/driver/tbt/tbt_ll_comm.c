/*
 * tbt_low_level_comm.c
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */
#include <stdio.h>

#include "tbt.h"

uint32_t _TBT_write_packet_data(TBT_interface_t *iface)
{
	if (HAL_UART_Transmit(iface->port_fd, iface->oBuf, TBT_OUTPUT_BUFFER_SIZE, TBT_SERIAL_WAIT)
			!= HAL_OK)
		return TBT_FAILURE;

#if DEBUG_TBT==1
	printf("send: ");
	for ( int i = 0; i < TBT_OUTPUT_BUFFER_SIZE; i++ ) {
		printf("[%02X] ", (int) iface->oBuf & 0xFF);
	}
	printf("\r\n");
#endif	//if DEBUG_TBT==1

	return TBT_SUCCESS;
}

uint32_t _TBT_get_packet(TBT_interface_t *iface)
{
	uint32_t pos = 0;
	uint32_t timer = HAL_GetTick() + TBT_SERIAL_WAIT;

#if DEBUG_TBT==1
	printf("\r\nrecv: ");
#endif	//if DEBUG_TBT==1
	while (HAL_GetTick() < timer) {
		if (HAL_UART_Receive(iface->port_fd, &iface->ibuf[pos], 1, 1) == HAL_OK) {
#if DEBUG_TBT==1
			printf("[%02X]", iface->ibuf[pos]);
#endif	//if DEBUG_TBT==1

			if (iface->ibuf[pos] == TBT_TERMINATOR)
				break;
			else {
				if (++pos >= TBT_INPUT_BUFFER_SIZE)
					return TBT_FAILURE;
			}
		}
	}
	iface->bytes = pos + 1;

	if (HAL_GetTick() >= timer) {
		printf("timeout\r\n");
		return TBT_FAILURE;
	}
	else {
		printf("\r\n");
		return TBT_SUCCESS;
	}
}
