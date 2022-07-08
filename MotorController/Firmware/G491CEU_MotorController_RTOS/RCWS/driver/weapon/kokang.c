/*
 * kokang.c
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#include "kokang.h"
#include "tim.h"

static void cock_cmd_on()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim2, C_ENC_MIN);
}

static void cock_cmd_off()
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
}

void cock_init(Cock_t *cock)
{
	cock->enable = 0;
	cock->start = 0;
	cock->state = CST_IDLE;
	cock->timeout = 0;

	cock_cmd_off();

	cock_power(cock, 0);
}

HAL_StatusTypeDef cock_handler(Cock_t *cock)
{
	int32_t cock_enc_value = (int32_t) __HAL_TIM_GET_COUNTER(&htim2);

	switch (cock->state)
	{
	case CST_ERECT:
		/* TODO reach the desired position or timeout */
		if ((cock_enc_value >= (C_ENC_MAX - C_ENC_HYST)) || (HAL_GetTick() >= cock->timeout)) {
			cock->state = CST_MAX_STOP;
			cock->timeout = HAL_GetTick() + C_TIM_MAX_STOP;
		}
		break;
	case CST_MAX_STOP:
		if (HAL_GetTick() >= cock->timeout)
			cock->state = CST_RETRACT;
		break;
	case CST_RETRACT:
		/* TODO reach the desired position or timeout */
		if ((cock_enc_value <= (C_ENC_MIN + C_ENC_HYST)) || (HAL_GetTick() >= cock->timeout)) {
			cock_cmd_off();
			cock->state = CST_IDLE;

			if (HAL_GetTick() >= cock->timeout)
				return HAL_ERROR;
			else
				return HAL_OK;
		}

		break;
	default:
		break;
	}

	return HAL_BUSY;
}

void cock_start(Cock_t *cock)
{
	cock->state = CST_ERECT;
	cock->timeout = HAL_GetTick() + C_TIM_ERECT;

	cock_cmd_on();
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
}

void cock_power(Cock_t *cock, const uint8_t act)
{
	uint8_t enable = act & 0b1;

	if (enable != 0)
		HAL_GPIO_WritePin(COCK_PWR_EN_GPIO_Port, COCK_PWR_EN_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(COCK_PWR_EN_GPIO_Port, COCK_PWR_EN_Pin, GPIO_PIN_RESET);

	cock->state = CST_IDLE;
	cock->enable = enable;
}

