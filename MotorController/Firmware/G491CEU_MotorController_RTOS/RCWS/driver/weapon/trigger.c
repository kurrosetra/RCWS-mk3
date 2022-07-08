/*
 * trigger.c
 *
 *  Created on: Jun 9, 2022
 *      Author: 62812
 */

#include "trigger.h"
#include "tim.h"

volatile uint32_t t_js_counter;

void trig_set_power(const uint8_t act)
{
	HAL_GPIO_WritePin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin, act);
}

void trig_start()
{
	HAL_GPIO_WritePin(T_START_GPIO_Port, T_START_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_SET);
}

void trig_s_stop()
{
	HAL_GPIO_WritePin(T_START_GPIO_Port, T_START_Pin, GPIO_PIN_RESET);
}

void trig_h_stop()
{
	HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_RESET);
}

uint8_t trig_pulse_state()
{
	if (HAL_GPIO_ReadPin(T_JS_PULSE_GPIO_Port, T_JS_PULSE_Pin) == GPIO_PIN_RESET)
		return 1;
	else
		return 0;
}

uint8_t trig_is_pulse_off()
{
	if (HAL_GPIO_ReadPin(T_JS_PULSE_GPIO_Port, T_JS_PULSE_Pin) == 0) {
		trig_pulse_off();
		return 1;
	}

	return 0;
}

void trig_all_stop()
{
	trig_s_stop();
	trig_h_stop();
}

void trig_pulse_on()
{
	if (HAL_GPIO_ReadPin(TRIGGER_ENABLE_GPIO_Port, TRIGGER_ENABLE_Pin) == GPIO_PIN_SET) {
		trig_start();

		MAIL_Weapon_t *pTMail;
		// allocate memory; receiver must be free it
		pTMail = osMailAlloc(mtr_get_mail(T_Weapon_id), 0);
		pTMail->sender_id = Weapon_Sender_Sensor_Trigger_id;
		*(uint8_t*) &pTMail->param.sensor.trigger = 0;
		pTMail->param.sensor.trigger.pulse_on = 1;

		osMailPut(mtr_get_mail(T_Weapon_id), pTMail);
	}
}

void trig_pulse_off()
{
////	HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_RESET);
//	MAIL_Weapon_t *pTMail;
//	// allocate memory; receiver must be free it
//	pTMail = osMailAlloc(mtr_get_mail(T_Weapon_id), 0);
//	pTMail->sender_id = Weapon_Sender_Sensor_Trigger_id;
//	*(uint8_t*) &pTMail->param.sensor.trigger = 0;
//	pTMail->param.sensor.trigger.pulse_off = 1;
//
//	osMailPut(mtr_get_mail(T_Weapon_id), pTMail);
}

//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == htim15.Instance) {
////		if (firing_mode != FIRE_MODE_INF) {
////			__HAL_TIM_SET_COUNTER(&htim15, 0);
////			HAL_TIM_Base_Start_IT(&htim15);
////		}
//		HAL_GPIO_WritePin(T_HOLD_GPIO_Port, T_HOLD_Pin, GPIO_PIN_SET);
//		LOG("Trigger start!\r\n");
//	}
//}
