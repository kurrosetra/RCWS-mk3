/*
 * visca_low_level_comm.c
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */

#include <stdio.h>

#include "visca.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


static osMessageQId q_sonyHandle;


/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/
uint32_t _VISCA_write_packet_data(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet)
{
	HAL_UART_Transmit(iface->port_fd, packet->bytes, packet->length, VISCA_SERIAL_WAIT);

//#if DEBUG_VISCA==1
//	printf("send: ");
//	for ( int i = 0; i < packet->length; i++ ) {
//		printf("[%02X] ", packet->bytes[i]);
//	}
//	printf("\r\n");
//#endif	//if DEBUG_VISCA==1

	return VISCA_SUCCESS;
}

uint32_t _VISCA_get_packet(VISCAInterface_t *iface)
{
	uint32_t ret = VISCA_FAILURE;

	/* create queue */
	osMessageQDef(q_sony, VISCA_INPUT_BUFFER_SIZE, uint8_t);
	q_sonyHandle = osMessageCreate(osMessageQ(q_sony), NULL);

	/* enable uart in receive interrupt */
	__HAL_UART_ENABLE_IT(iface->port_fd, UART_IT_RXNE);

	uint32_t pos = 0;
	uint32_t timer = HAL_GetTick() + VISCA_SERIAL_LONG_WAIT;

#if DEBUG_VISCA==1
	printf("\r\nrecv: ");
#endif	//if DEBUG_VISCA==1

	while (HAL_GetTick() < timer) {
		osEvent event = osMessageGet(q_sonyHandle, VISCA_SERIAL_WAIT);
		if (event.status == osEventMessage) {
			iface->ibuf[pos] = (uint8_t) event.value.v;
#if DEBUG_VISCA==1
			printf("[%02X]", iface->ibuf[pos]);
#endif	//if DEBUG_VISCA==1

			if (iface->ibuf[pos] == VISCA_TERMINATOR)
				break;
			else {
				if (++pos >= VISCA_INPUT_BUFFER_SIZE) {
					ret = VISCA_FAILURE;
					goto exit;
				}
			}
		}

	}
	iface->bytes = pos + 1;

	if (HAL_GetTick() >= timer) {
		printf("timeout\r\n");
		ret = VISCA_FAILURE;
	}
	else {
		printf("\r\n");
		ret = VISCA_SUCCESS;
	}

	exit:
	/* disable uart in receive interrupt */
	__HAL_UART_DISABLE_IT(iface->port_fd, UART_IT_RXNE);
	/* delete queue */
	osMessageDelete(q_sonyHandle);

	return ret;
}

uint32_t VISCA_unread_bytes(VISCAInterface_t *iface, unsigned char *buffer, uint32_t *buffer_size)
{
// TODO
	*buffer_size = 0;
	return VISCA_SUCCESS;
}

void USART2_IRQHandler(void)
{
	uint32_t isrflags = READ_REG(huart2.Instance->ISR);
	/* UART in mode Receiver ---------------------------------------------------*/
	if ((isrflags & USART_ISR_RXNE) != 0U) {

		uint8_t recv_byte = (huart2.Instance->RDR & 0xFF);
		osMessagePut(q_sonyHandle, recv_byte, 0);
	}
}

