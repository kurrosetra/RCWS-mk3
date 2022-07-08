/*
 * noptel.h
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#ifndef DRIVER_NOPTEL_NOPTEL_H_
#define DRIVER_NOPTEL_NOPTEL_H_

#include "main.h"
#include "noptel_conf.h"
#include "noptel_def.h"

/* timeout in ms */
#define NOPTEL_SERIAL_LONG_WAIT					10000
#define NOPTEL_SERIAL_WAIT						100

/* size of the local packet buffer */
#define NOPTEL_INPUT_BUFFER_SIZE				32
#define NOPTEL_OUTPUT_BUFFER_SIZE				8

/* This is the interface for the STM platform.
 */
typedef struct
{
	/* uart */
	UART_HandleTypeDef *port_fd;

	GPIO_TypeDef *reset_port;
	uint16_t reset_pin;

	/* input buffer */
	uint8_t ibuf[NOPTEL_INPUT_BUFFER_SIZE];
	uint8_t ilen;

	/* output buffer */
	uint8_t oBuf[NOPTEL_OUTPUT_BUFFER_SIZE];
	uint8_t olen;
} Noptel_interface_t;

typedef struct
{
	uint8_t counter;
	uint32_t distance[3];
	uint8_t multi_target;
//	uint8_t pointerState;
//
//	uint8_t statusBytes[3];
} Noptel_Value_Buffer_t;

typedef struct
{
	Noptel_interface_t iface;
	Noptel_Value_Buffer_t value;
} Noptel_t;

HAL_StatusTypeDef _NOPTEL_write_packet_data(Noptel_interface_t *iface, const uint8_t get_reply);

HAL_StatusTypeDef noptel_init(Noptel_t *noptel, UART_HandleTypeDef *huart, GPIO_TypeDef *port,
		const uint16_t pin);
HAL_StatusTypeDef noptel_power(Noptel_t *noptel, const uint8_t power);
HAL_StatusTypeDef noptel_set_pointer(Noptel_t *noptel, const uint8_t power);
HAL_StatusTypeDef noptel_measure_start(Noptel_t *noptel);
HAL_StatusTypeDef noptel_measure_start_with_status(Noptel_t *noptel,uint8_t *status);
HAL_StatusTypeDef noptel_get_last_result(Noptel_t *noptel, uint32_t *last_distance);
HAL_StatusTypeDef noptel_get_state(Noptel_t *noptel, uint8_t *states);
HAL_StatusTypeDef noptel_get_range_window(Noptel_t *noptel, uint32_t *min, uint32_t *max);
HAL_StatusTypeDef noptel_set_range_window(Noptel_t *noptel, const uint32_t min, const uint32_t max);
HAL_StatusTypeDef noptel_set_IBIT_test(Noptel_t *noptel);
HAL_StatusTypeDef noptel_reset_comm_error_counter(Noptel_t *noptel);
HAL_StatusTypeDef noptel_reset_value(Noptel_t *noptel);

#endif /* DRIVER_NOPTEL_NOPTEL_H_ */
