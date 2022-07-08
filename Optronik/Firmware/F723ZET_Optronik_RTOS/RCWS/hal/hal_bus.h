/*
 * hal_bus.h
 *
 *  Created on: Jan 10, 2022
 *      Author: miftakur
 */

#ifndef HAL_HAL_BUS_H_
#define HAL_HAL_BUS_H_

#include "app/common.h"
#include "driver/bus_can/bus_can.h"

void hal_bus_init();
HAL_StatusTypeDef hal_bus_send(Bus_Tx_Buffer_t *buffer);
HAL_StatusTypeDef hal_lrf_send(Bus_Tx_Buffer_t *buffer);

#endif /* HAL_HAL_BUS_H_ */
