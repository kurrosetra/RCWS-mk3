/*
 * kokang.h
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#ifndef DRIVER_KOKANG_KOKANG_H_
#define DRIVER_KOKANG_KOKANG_H_

#include "app/common.h"

typedef enum
{
	CST_IDLE,
	CST_ERECT,
	CST_MAX_STOP,
	CST_RETRACT,
} Cock_State_e;

typedef enum
{
	C_TIM_ERECT = 5000,
	C_TIM_MAX_STOP = 1000,
	C_TIM_RETRACT = 5000,
} Cock_Timeout_e;

typedef enum
{
	C_ENC_MIN = 0,
	C_ENC_MAX = 1000,
	C_ENC_HYST = 50,
} Cock_Encoder_e;

typedef struct
{
	uint8_t enable;
	uint8_t start;
	Cock_State_e state;
	uint32_t timeout;
} Cock_t;

void cock_init(Cock_t *cock);
HAL_StatusTypeDef cock_handler(Cock_t *cock);
void cock_start(Cock_t *cock);
void cock_power(Cock_t *cock, const uint8_t act);

#endif /* DRIVER_KOKANG_KOKANG_H_ */
