/*
 * tbt.c
 *
 *  Created on: Dec 30, 2021
 *      Author: miftakur
 */
#include <string.h>

#include "tbt.h"

#if DEBUG_TBT==1
#include <stdio.h>
#endif	//if DEBUG_TBT==1

static const uint8_t tbt_default_output_format[6] = { TBT_HEADER, 0, 0, 0, 0, TBT_TERMINATOR };

/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/
__weak uint32_t _TBT_write_packet_data(TBT_interface_t *iface)
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

__weak uint32_t _TBT_get_packet(TBT_interface_t *iface)
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

/****************************************************************************/
/*                           PUBLIC FUNCTIONS                               */
/****************************************************************************/

/***********************************/
/*       SYSTEM  FUNCTIONS         */
/***********************************/

/***********************************/
/*       COMMAND FUNCTIONS         */
/***********************************/

uint32_t TBT_set_zoom_optical_stop(TBT_interface_t *iface)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_OPTICAL_ZOOM_STOP;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_zoom_optical_start(TBT_interface_t *iface, const uint8_t zoom_end,
		const uint8_t speed)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_OPTICAL_ZOOM_START;
	iface->oBuf[2] = zoom_end;
	iface->oBuf[3] = speed;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_zoom_optical_direct(TBT_interface_t *iface, const uint16_t zoom,
		const uint8_t speed)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_OPTICAL_ZOOM_DIRECT;
	iface->oBuf[2] = (uint8_t) (zoom >> 8);
	iface->oBuf[3] = (uint8_t) (zoom & 0xFF);
	iface->oBuf[4] = speed;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_focus_stop(TBT_interface_t *iface)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_FOCUS_STOP;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_focus_start(TBT_interface_t *iface, const uint8_t focus_end, const uint8_t speed)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_FOCUS_START;
	iface->oBuf[2] = focus_end;
	iface->oBuf[3] = speed;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_focus_direct(TBT_interface_t *iface, const uint16_t focus, const uint8_t speed)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_FOCUS_DIRECT;
	iface->oBuf[2] = (uint8_t) (focus >> 8);
	iface->oBuf[3] = (uint8_t) (focus & 0xFF);
	iface->oBuf[4] = speed;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_focus_auto_one_push(TBT_interface_t *iface)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_AUTO_FOCUS_START;

	return _TBT_write_packet_data(iface);
}

uint32_t TBT_set_image_stabilizer(TBT_interface_t *iface, const uint8_t power)
{
	memcpy(iface->oBuf, tbt_default_output_format, TBT_OUTPUT_BUFFER_SIZE);

	iface->oBuf[1] = TBT_IMAGE_STABILIZER;
	iface->oBuf[2] = power;

	return _TBT_write_packet_data(iface);
}

