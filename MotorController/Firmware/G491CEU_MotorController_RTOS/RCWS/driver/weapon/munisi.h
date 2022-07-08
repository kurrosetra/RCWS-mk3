/*
 * munisi.h
 *
 *  Created on: Jun 15, 2022
 *      Author: 62812
 */

#ifndef DRIVER_WEAPON_MUNISI_H_
#define DRIVER_WEAPON_MUNISI_H_

#include "main.h"

void munisi_reset();
uint16_t munisi_get_counter();
void munisi_set_state(const uint8_t power);
uint8_t munisi_get_state();

#endif /* DRIVER_WEAPON_MUNISI_H_ */
