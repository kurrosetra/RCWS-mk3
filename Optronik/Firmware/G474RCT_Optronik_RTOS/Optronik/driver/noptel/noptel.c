/*
 * noptel.c
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#include <string.h>

#include "noptel.h"

#if DEBUG_NOPTEL==1
#include <stdio.h>

#define LOG(str, ...) printf("[%ld %s:%d] " str, HAL_GetTick(), __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[%s Err:%d] " str, __FILENAME__, __LINE__, ##__VA_ARGS__)
#endif	//if DEBUG_NOPTEL==1

static HAL_StatusTypeDef _NOPTEL_write_packet_data_with_reply(Noptel_interface_t *iface)
{
	return _NOPTEL_write_packet_data(iface, 1);
}

/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/
__weak HAL_StatusTypeDef _NOPTEL_write_packet_data(Noptel_interface_t *iface,
		const uint8_t get_reply)
{
	HAL_UART_Transmit(iface->port_fd, iface->oBuf, iface->olen, NOPTEL_SERIAL_WAIT);

	if (get_reply != 0)
		/* wait reply */
		;

	return HAL_OK;
}

static uint8_t _get_check_byte(Noptel_interface_t *iface)
{
	uint8_t cb = 0;
	if (iface->olen > 0) {
		for ( int i = 0; i < iface->olen; i++ )
			cb += iface->oBuf[i];
		cb ^= 0x50;
	}
	return cb;
}

static void _append_byte(Noptel_interface_t *iface, const uint8_t byte)
{
	iface->oBuf[iface->olen++] = byte;
}

static void _init_packet(Noptel_interface_t *iface)
{
	iface->olen = 0;
	memset(iface->ibuf, 0, NOPTEL_INPUT_BUFFER_SIZE);
	iface->ilen = 0;
}

HAL_StatusTypeDef noptel_init(Noptel_t *noptel, UART_HandleTypeDef *huart, GPIO_TypeDef *port,
		const uint16_t pin)
{
	noptel->iface.port_fd = huart;
	noptel->iface.reset_port = NULL;
	if (port != NULL) {
		noptel->iface.reset_port = port;
		noptel->iface.reset_pin = pin;
	}

	noptel->value.counter = 0;
	noptel->value.distance[0] = noptel->value.distance[1] = noptel->value.distance[2] = 0;

	LOG("init done\r\n");
	return HAL_OK;
}

HAL_StatusTypeDef noptel_power(Noptel_t *noptel, const uint8_t power)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* set reset pin */
	if (noptel->iface.reset_port != NULL) {
		GPIO_PinState pinState = GPIO_PIN_RESET;
		if (power == 1)
			pinState = GPIO_PIN_SET;
		HAL_GPIO_WritePin(noptel->iface.reset_port, noptel->iface.reset_pin, pinState);
	}

	LOG("power: %d\r\n", power);

	return ret;
}

HAL_StatusTypeDef noptel_set_pointer(Noptel_t *noptel, const uint8_t power)
{
	_init_packet(&noptel->iface);
	_append_byte(&noptel->iface, NOPTEL_SET_POINTER_MODE);
	if (power == 1)
		_append_byte(&noptel->iface, NOPTEL_SET_POINTER_MODE_ON);
	else
		_append_byte(&noptel->iface, NOPTEL_SET_POINTER_MODE_OFF);
	_append_byte(&noptel->iface, _get_check_byte(&noptel->iface));

	_NOPTEL_write_packet_data_with_reply(&noptel->iface);

	LOG("pointer: %d\r\n", power);

	return HAL_OK;
}

static uint8_t _get_ack_sync_position(Noptel_interface_t *iface)
{
	uint8_t ret = 0;
	for ( int i = 0; i < iface->ilen; i++ ) {
		if (iface->ibuf[i] == NOPTEL_ACK_SYNC_HEADER_BYTE) {
			ret = i;
			break;
		}
	}

	return ret;
}

static HAL_StatusTypeDef _parse_incoming_packet(Noptel_interface_t *iface, uint8_t *startPos)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t cb = 0;
	if (iface->ilen > 2) {
		*startPos = _get_ack_sync_position(iface);
		for ( int i = *startPos; i < iface->ilen - 1; i++ )
			cb += iface->ibuf[i];

		cb ^= 0x50;
		if (cb == iface->ibuf[iface->ilen - 1])
			ret = HAL_OK;
	}

	return ret;
}

HAL_StatusTypeDef noptel_measure_start(Noptel_t *noptel)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t startPos = 0;
	Noptel_interface_t *iface = &noptel->iface;

	_init_packet(iface);
	_append_byte(iface, NOPTEL_SET_MEASURE_DISTANCE);
	_append_byte(iface, _get_check_byte(iface));

	LOG("send %dB: ", noptel->iface.olen);
	for ( int i = 0; i < noptel->iface.olen; i++ )
		printf("[%02X]", noptel->iface.oBuf[i]);
	printf("\r\n");

	if (_NOPTEL_write_packet_data_with_reply(iface) == HAL_OK) {
		if (_parse_incoming_packet(iface, &startPos) == HAL_OK) {
			if (iface->ibuf[startPos + 1] == NOPTEL_SET_MEASURE_DISTANCE) {
				noptel->value.counter++;

				noptel->value.distance[0] = (uint16_t) iface->ibuf[startPos + 3] << 8
						| iface->ibuf[startPos + 2];
				noptel->value.distance[1] = (uint16_t) iface->ibuf[startPos + 5] << 8
						| iface->ibuf[startPos + 4];
				noptel->value.distance[2] = (uint16_t) iface->ibuf[startPos + 7] << 8
						| iface->ibuf[startPos + 6];

				LOG("[c=%d]d= %ld %ld %ld\r\n", noptel->value.counter, noptel->value.distance[0],
						noptel->value.distance[1], noptel->value.distance[2]);
				ret = HAL_OK;
			}
		}
	}

	return ret;
}

HAL_StatusTypeDef noptel_measure_start_with_status(Noptel_t *noptel, uint8_t *status)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t startPos = 0;
	Noptel_interface_t *iface = &noptel->iface;
	uint16_t d[3] = { 0, 0, 0 };

	_init_packet(iface);
	_append_byte(iface, NOPTEL_SET_MEASURE_DISTANCE_STATUS);
	_append_byte(iface, _get_check_byte(iface));

//	LOG("send %dB: ", noptel->iface.olen);
//	for ( int i = 0; i < noptel->iface.olen; i++ )
//		printf("[%02X]", noptel->iface.oBuf[i]);
//	printf("\r\n");

	if (_NOPTEL_write_packet_data_with_reply(iface) == HAL_OK) {
		if (_parse_incoming_packet(iface, &startPos) == HAL_OK) {
			if (iface->ibuf[startPos + 1] == NOPTEL_SET_MEASURE_DISTANCE_STATUS) {
				d[0] = (uint16_t) iface->ibuf[startPos + 3] << 8 | iface->ibuf[startPos + 2];
				d[1] = (uint16_t) iface->ibuf[startPos + 5] << 8 | iface->ibuf[startPos + 4];
				d[2] = (uint16_t) iface->ibuf[startPos + 7] << 8 | iface->ibuf[startPos + 6];
				*status = iface->ibuf[startPos + 8];

				LOG("[st= %02X]\r\n", *status);

				if ((READ_BIT(*status,(1<<NOPTEL_STATUS_B3_ERR_bit)) == 0)
						&& (READ_BIT(*status,(1<<NOPTEL_STATUS_B3_NT_bit)) == 0)) {

					if (READ_BIT(*status,(1<<NOPTEL_STATUS_B3_MT_bit)) != 0)
						noptel->value.multi_target = 1;

					noptel->value.counter++;
					noptel->value.distance[0] = d[0];
					noptel->value.distance[1] = d[1];
					noptel->value.distance[2] = d[2];

					LOG("[c=%d]d= %ld %ld %ld\r\n", noptel->value.counter,
							noptel->value.distance[0], noptel->value.distance[1],
							noptel->value.distance[2]);

					ret = HAL_OK;
				}
			}
		}
	}

	return ret;
}

HAL_StatusTypeDef noptel_get_last_result(Noptel_t *noptel, uint32_t *last_distance)
{

	return HAL_OK;
}

HAL_StatusTypeDef noptel_get_state(Noptel_t *noptel, uint8_t *states)
{

	return HAL_OK;
}

HAL_StatusTypeDef noptel_get_range_window(Noptel_t *noptel, uint32_t *min, uint32_t *max)
{
	return HAL_OK;
}

HAL_StatusTypeDef noptel_set_range_window(Noptel_t *noptel, const uint32_t min, const uint32_t max)
{
	return HAL_OK;
}

HAL_StatusTypeDef noptel_set_IBIT_test(Noptel_t *noptel)
{
	return HAL_OK;
}

HAL_StatusTypeDef noptel_reset_comm_error_counter(Noptel_t *noptel)
{
	return HAL_OK;
}

HAL_StatusTypeDef noptel_reset_value(Noptel_t *noptel)
{
	return HAL_OK;
}
