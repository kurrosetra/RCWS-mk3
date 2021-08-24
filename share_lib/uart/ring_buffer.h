/*
 * kurro_buffer.h
 *
 *  Created on: Nov 24, 2018
 *      Author: miftakur
 *
 *	Changelog:
 *	20210818:
 *		- fix ring_buffer_peek function
 *	20210331:
 *		- add init
 *		- separate RX/TX max bufffer size
 *		- circular buffer based on pointer	<-- error
 *		- add peek, flush, read until function
 *
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include "ring_buffer_config.h"
#include <stdint-gcc.h>

typedef struct
{
	/* pointer to real buffer */
	uint8_t *storage;
	/* size of buffer */
	uint16_t size;
	/* TODO implement Lock/Unlock */
	HAL_LockTypeDef Lock;

	/* start write */
	volatile uint16_t head;
	/* start read */
	volatile uint16_t tail;
} Ring_Buffer_t;

void ring_buffer_init(Ring_Buffer_t *buffer, uint8_t *storage, const uint16_t size);
uint16_t ring_buffer_available(const Ring_Buffer_t *buffer);
uint16_t ring_buffer_free(const Ring_Buffer_t *buffer);
void ring_buffer_flush(Ring_Buffer_t *buffer);

HAL_StatusTypeDef ring_buffer_peek(Ring_Buffer_t *buffer, uint8_t *peek, const uint16_t pos);
HAL_StatusTypeDef ring_buffer_read(Ring_Buffer_t *buffer, uint8_t *str, const uint16_t size);
uint16_t ring_buffer_read_until(Ring_Buffer_t *buffer, uint8_t *str, const uint8_t stopByte);

void ring_buffer_write(Ring_Buffer_t *buffer, uint8_t *str, const uint16_t size);

#endif /* RING_BUFFER_H_ */
