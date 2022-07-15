/*
 * sla.h
 *
 *  Created on: Jun 24, 2022
 *      Author: 62812
 */

#ifndef SLA_H_
#define SLA_H_

#include "main.h"

#define DEBUG_SLA_ENABLE		1

typedef enum
{
	TRK_GATE_NONE = 0,
	TRK_GATE_BIGGER = 1,
	TRK_GATE_SMALLER = -1
} TrackGateResize_e;

typedef enum
{
	TRK_SEL_NONE = 0,
	TRK_SEL_NEXT = 1,
	TRK_SEL_PREV = -1
} TrackSelectTarget_e;

void sla_init();
void sla_handler();

void sla_set_attitude(const float az, const float el);
void sla_set_distance(const uint16_t d);

uint8_t sla_gsl_available();
void sla_gsl_get_command(uint8_t *counter, int32_t *az, int32_t *el, uint16_t *distance);
void sla_gsl_start();

void tracker_init();
HAL_StatusTypeDef tracker_available();
HAL_StatusTypeDef tracker_read(uint8_t *id, int16_t *dx, int16_t *dy);
void tracker_change_gate_size(const int8_t resize);
void tracker_change_target(const int8_t retarget);

void sla_print_text(const char *str, const uint16_t len);

#endif /* SLA_H_ */
