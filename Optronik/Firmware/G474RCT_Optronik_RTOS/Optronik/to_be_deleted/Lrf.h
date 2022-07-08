/*
 * Lrf.h
 *
 *  Created on: Dec 27, 2021
 *      Author: miftakur
 */

#ifndef LRF_H_
#define LRF_H_

#include "bus_button_config.h"

#include "common.h"

typedef struct
{
	uint8_t lrf_enable;
	Optronik_Lrf_state_t value;
} Lrf_State_t;

typedef struct
{
	Panel_lrf_imu_command_t command;
	Lrf_State_t state;
	uint8_t busy;
} Lrf_t;

uint8_t lrf_is_busy();
osStatus lrf_read(Lrf_t *lrf, const uint32_t timeout);
osStatus lrf_write(Lrf_t *lrf, const uint32_t timeout);

#endif /* LRF_H_ */
