/*
 * track_control.h
 *
 *  Created on: Jul 14, 2022
 *      Author: Miftakur Rohman
 */

#ifndef DRIVER_TRACK_CONTROL_TRACK_CONTROL_H_
#define DRIVER_TRACK_CONTROL_TRACK_CONTROL_H_

#include "main.h"

void Kontrol_Konstanta_init();
void Kontrol_init();
void Kontrol_CalcQDot(int zoomLevel, float qaz, float qev, float qdaz, float qdev, int dx, int dy, float *qd_sp_az,
		float *qd_sp_ev);

#endif /* DRIVER_TRACK_CONTROL_TRACK_CONTROL_H_ */
