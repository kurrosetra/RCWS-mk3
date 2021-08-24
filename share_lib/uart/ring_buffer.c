#include "ring_buffer.h"

#define RING_DEBUG	1

#if RING_DEBUG==1
#include <stdio.h>
extern UART_HandleTypeDef hlpuart1;
uint16_t dLen;
char dBuf[256];

#endif	//if RING_DEBUG==1

static uint16_t min(const uint16_t a, const uint16_t b)
{
	return (a <= b) ? a : b;
}

/* initialize buffer */
void ring_buffer_init(Ring_Buffer_t *buffer, uint8_t *storage, const uint16_t size)
{
	buffer->Lock = HAL_UNLOCKED;
	buffer->storage = storage;
	buffer->size = size;

	ring_buffer_flush(buffer);
}

/* size of  stored byte(s) */
uint16_t ring_buffer_available(const Ring_Buffer_t *buffer)
{
	uint16_t ret = 0;
	uint16_t _head = buffer->head;
	uint16_t _tail = buffer->tail;

	if (_head > _tail)
		ret = _head - _tail;
	else if (_head < _tail)
		ret = _head + (buffer->size - _tail);

	return ret;
}

/* size of free byte(s) */
uint16_t ring_buffer_free(const Ring_Buffer_t *buffer)
{
	return (buffer->size - ring_buffer_available(buffer));
}

/* flush buffer */
void ring_buffer_flush(Ring_Buffer_t *buffer)
{
	buffer->head = buffer->tail = 0;
}

/* read byte(s), not freeing the space */
HAL_StatusTypeDef ring_buffer_peek(Ring_Buffer_t *buffer, uint8_t *peek, const uint16_t pos)
{
	uint16_t len = ring_buffer_available(buffer);
	uint16_t posRelative;

	if (len > 0) {
		if (pos >= len)
			posRelative = buffer->tail + len - 1;
		else
			posRelative = buffer->tail + pos;

		if (posRelative >= buffer->size)
			posRelative -= buffer->size;
		*peek = buffer->storage[posRelative];

		return HAL_OK;
	}

	return HAL_ERROR;
}

/* read bytes & free space */
HAL_StatusTypeDef ring_buffer_read(Ring_Buffer_t *buffer, uint8_t *str, const uint16_t size)
{
	uint16_t _bufAvailable = ring_buffer_available(buffer);
	uint16_t minSize = min(_bufAvailable, size);

	if (_bufAvailable > 0) {
		for ( int i = 0; i < minSize; i++ ) {
			*(str + i) = buffer->storage[buffer->tail];
			buffer->tail = (buffer->tail + 1) % buffer->size;
		}

		return HAL_OK;
	}

	return HAL_ERROR;
}

/* read bytes until stopByte found or no more byte to be read */
uint16_t ring_buffer_read_until(Ring_Buffer_t *buffer, uint8_t *str, const uint8_t stopByte)
{
	uint8_t u08;
	for ( int i = 0; i < buffer->size; i++ ) {
		if (ring_buffer_read(buffer, &u08, 1) == HAL_OK) {
			str[i] = u08;
			if (u08 == stopByte) {
				str[i + 1] = '\0';
				return i;
			}
		}
		else {
			str[i] = '\0';
			return i;
		}
	}

	str[0] = '\0';
	return 0;
}

static HAL_StatusTypeDef ring_buffer_write_byte(Ring_Buffer_t *buffer, uint8_t val)
{
	uint16_t _head = buffer->head;
	uint16_t _tail = buffer->tail;
	uint16_t nextHead = (_head + 1) % buffer->size;

	if (nextHead != _tail) {
		buffer->storage[_head] = val;
		buffer->head = nextHead;

		return HAL_OK;
	}

	return HAL_ERROR;
}

/* write to buffer */
void ring_buffer_write(Ring_Buffer_t *buffer, uint8_t *str, const uint16_t size)
{
	uint16_t minSize = min(size, buffer->size);
	for ( int i = 0; i < minSize; i++ ) {
		if (ring_buffer_write_byte(buffer, str[i]) == HAL_ERROR)
			break;
	}
}

