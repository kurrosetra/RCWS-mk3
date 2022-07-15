/*
 * track_control.c
 *
 *  Created on: Jul 14, 2022
 *      Author: Miftakur Rohman
 */

#include <math.h>
#include <stdlib.h>

#include "track_control.h"

typedef struct
{
	int zoomLevel;
	float kp1;
	float kp2;
	float kd1;
	float kd2;
	float ki1;
	float ki2;
	int thresholdPX;
} Konstanta_PID_t;

Konstanta_PID_t kPID[3];

const Konstanta_PID_t kPID_0 = { 0, 0.65, 0.5, 0.05, 1.2, 0.5, 0.8, 0 };
const Konstanta_PID_t kPID_1 = { 1, 0.1, 0.07, 1.0, 0.8, 4, 0.3, 0 };
const Konstanta_PID_t kPID_2 = { 2, 0.05, 0.03, 0.0, 0.0, 0.0, 0.0, 2 };

float q_acc0, q_acc1;
float theta1, theta2;
float theta1_d, theta2_d;
float Torq1, Torq2;
const float scale = 0.1;

static float toDeg(float a)
{
	return a * 180 / M_PI;
}

static float toRad(float a)
{
	return a * M_PI / 180;
}

static float wrapVal(float a, float b)
{
	if (a < -b)
		return -b;
	else if (a > b)
		return b;
	else
		return a;
}

void Kontrol_init()
{
	q_acc0 = 0;
	q_acc1 = 0;
}

void Kontrol_Konstanta_init()
{
	kPID[0] = kPID_0;
	kPID[1] = kPID_1;
	kPID[2] = kPID_2;
}

void Kontrol_CalcQDot(int zoomLevel, float qaz, float qev, float qdaz, float qdev, int dx, int dy, float *qd_sp_az,
		float *qd_sp_ev)
{
	theta1 = toRad(qaz);
	theta2 = toRad(qev);
	theta1_d = toRad(qdaz);
	theta2_d = toRad(qdev);

	// Gain Schedule
	if (zoomLevel < 0)
		zoomLevel = 0;
	else if (zoomLevel > 2)
		zoomLevel = 2;

	Konstanta_PID_t *pid = &kPID[zoomLevel];
	float kp1 = pid->kp1;
	float kp2 = pid->kp2;
	float kd1 = pid->kd1;
	float kd2 = pid->kd2;
	float ki1 = pid->ki1;
	float ki2 = pid->ki2;
	int THRESHOLD_PX = pid->thresholdPX;

	Torq1 = toRad(kp1 * dx - kd1 * theta1_d + ki1 * q_acc0);
	Torq2 = toRad(kp2 * dy - kd2 * theta2_d + ki2 * q_acc1);
	if (abs(dx) <= THRESHOLD_PX)
		Torq1 = 0;
	if (abs(dy) <= THRESHOLD_PX)
		Torq2 = 0;

	Torq1 = Torq1 * scale;
	Torq2 = Torq2 * scale;

	q_acc0 += dx;
	q_acc1 += dy;
	q_acc0 = wrapVal(q_acc0, 10000);
	q_acc1 = wrapVal(q_acc1, 10000);

	if (abs(dx) <= THRESHOLD_PX) {
		q_acc0 = 0;
	}
	if (abs(dy) <= THRESHOLD_PX) {
		q_acc1 = 0;
	}

	*qd_sp_az = wrapVal(toDeg(Torq1), 70.0f);
	*qd_sp_ev = wrapVal(toDeg(Torq2), 90.0f);
}
