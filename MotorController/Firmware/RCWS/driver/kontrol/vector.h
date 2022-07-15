/*
 * vector.h
 *
 *  Created on: Jul 15, 2022
 *      Author: Miftakur Rohman
 */

#ifndef DRIVER_KONTROL_VECTOR_H_
#define DRIVER_KONTROL_VECTOR_H_

#include <math.h>

float norm(float x, float y, float z);
float dot(float, float, float, float, float, float);
float vectorAngle(float, float, float, float, float, float);
float vectorAngle2(float, float, float, float, float, float);
float toDeg(float a);
float toRad(float a);
int sign(float a);
float wrapAngle(float rad);
float wrapVal(float a, float b);

#endif /* DRIVER_KONTROL_VECTOR_H_ */
