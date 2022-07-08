/*
 * trigger.h
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#ifndef DRIVER_WEAPON_TRIGGER_H_
#define DRIVER_WEAPON_TRIGGER_H_

#include "app/common.h"

void trig_init();
void trig_set_power(const uint8_t act);
void trig_pulse_on();
void trig_pulse_off();
void trig_start();
void trig_all_stop();
void trig_s_stop();
void trig_h_stop();
uint8_t trig_is_pulse_off();

uint8_t trig_pulse_state();

extern volatile uint32_t t_js_counter;

#endif /* DRIVER_WEAPON_TRIGGER_H_ */
