/*
 * hal_bus.h
 *
 *  Created on: Jan 10, 2022
 *      Author: miftakur
 */

#ifndef HAL_HAL_BUS_H_
#define HAL_HAL_BUS_H_

#include "app/common.h"
#include "bus_fdcan.h"

void hal_bus_init();
HAL_StatusTypeDef hal_bus_send(const uint8_t *data);

#endif /* HAL_HAL_BUS_H_ */
