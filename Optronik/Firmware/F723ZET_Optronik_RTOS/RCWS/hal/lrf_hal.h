/*
 * lrf_hal.h
 *
 *  Created on: Jan 1, 2022
 *      Author: miftakur
 */

#ifndef HAL_LRF_HAL_H_
#define HAL_LRF_HAL_H_

#include "main.h"

HAL_StatusTypeDef lrf_init();
HAL_StatusTypeDef lrf_deInit();

HAL_StatusTypeDef lrf_power(const uint8_t power);
HAL_StatusTypeDef lrf_set_pointer(const uint8_t power);
HAL_StatusTypeDef lrf_measure_start();
HAL_StatusTypeDef lrf_measure_start_with_status(uint8_t *status);
//HAL_StatusTypeDef lrf_get_last_result(uint32_t *last_distance);
//HAL_StatusTypeDef lrf_get_state(uint8_t *states);
//HAL_StatusTypeDef lrf_get_range_window(uint32_t *min, uint32_t *max);
//HAL_StatusTypeDef lrf_set_range_window(const uint32_t min, const uint32_t max);
//HAL_StatusTypeDef lrf_set_IBIT_test();
//HAL_StatusTypeDef lrf_reset_comm_error_counter();

uint8_t lrf_get_counter();
void lrf_get_value(uint32_t *value);

#endif /* HAL_LRF_HAL_H_ */
