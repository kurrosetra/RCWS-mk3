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

//typedef struct
//{
//	uint8_t start :1;
//	uint8_t end :1;
//	uint8_t reservedBit :6;
//} __attribute__ ((packed)) gsl_state_t;
//
//typedef struct
//{
//	gsl_state_t state;
//	float startAE[2];
//	float targetAE[2];
//} gsl_t;

void sla_init();
void sla_handler();

void sla_set_attitude(const float az, const float el);
void sla_set_distance(const uint16_t d);

uint8_t sla_gsl_available();
void sla_gsl_get_command(uint8_t *counter, int32_t *az, int32_t *el, uint16_t *distance);
void sla_gsl_start();

#endif /* SLA_H_ */
