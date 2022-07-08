/*
 * lrf_hal.c
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#include "usart.h"
#include "lrf_hal.h"
#include "driver/noptel/noptel.h"

#define DEBUG_LRF_HAL			1
#if DEBUG_LRF_HAL==1
#include <stdio.h>

#	define LOG(str, ...) printf("[%s:%d] " str, __FILENAME_, __LINE__, ##__VA_ARGS__)
#	define LOG_E(str, ...) printf("[%s err:%d] " str, __FILENAME_, __LINE__, ##__VA_ARGS__)
#endif	//if DEBUG_LRF_HAL==1

static Noptel_t noptel;

HAL_StatusTypeDef lrf_init()
{
	return noptel_init(&noptel, &huart7, LRF_ENABLE_GPIO_Port, LRF_ENABLE_Pin);
}

HAL_StatusTypeDef lrf_deInit()
{
	return HAL_OK;
}

HAL_StatusTypeDef lrf_power(const uint8_t power)
{
	return noptel_power(&noptel, power);
}

HAL_StatusTypeDef lrf_set_pointer(const uint8_t power)
{
	return noptel_set_pointer(&noptel, power);
}

HAL_StatusTypeDef lrf_measure_start()
{
	return noptel_measure_start(&noptel);
}

HAL_StatusTypeDef lrf_measure_start_with_status(uint8_t *status)
{
	return noptel_measure_start_with_status(&noptel, status);
}

HAL_StatusTypeDef lrf_get_last_result(uint32_t *last_distance)
{
	return noptel_get_last_result(&noptel, noptel.value.distance);
}

HAL_StatusTypeDef lrf_get_state(uint8_t *states)
{
	return HAL_OK;
}

HAL_StatusTypeDef lrf_get_range_window(uint32_t *min, uint32_t *max)
{
	return HAL_OK;
}

HAL_StatusTypeDef lrf_set_range_window(const uint32_t min, const uint32_t max)
{
	return HAL_OK;
}

HAL_StatusTypeDef lrf_set_IBIT_test()
{
	return HAL_OK;
}

HAL_StatusTypeDef lrf_reset_comm_error_counter()
{
	return HAL_OK;
}

uint8_t lrf_get_counter()
{
	return noptel.value.counter;
}

void lrf_get_value(uint32_t *value)
{
	value[0] = noptel.value.distance[0];
	value[1] = noptel.value.distance[1];
	value[2] = noptel.value.distance[2];
}

