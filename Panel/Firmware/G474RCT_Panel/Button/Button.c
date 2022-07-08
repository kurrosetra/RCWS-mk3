/*
 * Button.c
 *
 *  Created on: Sep 22, 2021
 *      Author: miftakur
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Button.h"
#include "tim.h"
#include "adc.h"
#include "rws_config.h"
#include "busPanel.h"
#include "pc.h"
#include "movement.h"

uint8_t rxBtnBuffer[RING_BUFFER_RX_SIZE];
uint8_t txBtnBuffer[RING_BUFFER_TX_SIZE];
TSerial serialButton;

char btnStr[RING_BUFFER_RX_SIZE];
uint32_t adcVal[5];
TButton button;

static uint8_t adcConvCompleted = 0;

static long constrain(long x, const long min, const long max)
{
	if (x < min)
		x = min;
	else if (x > max)
		x = max;

	return x;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void button_init()
{
	button.serial = &serialButton;
	serial_init(button.serial, (char*) &rxBtnBuffer, sizeof(rxBtnBuffer), (char*) &txBtnBuffer,
			sizeof(txBtnBuffer), &huart2);

	*(uint8_t*) &button.JRight.button = 0;
	*(uint8_t*) &button.JLeft.button = 0;
	button.JRight.az = 0;
	button.JRight.az_tem = 0;
	button.JRight.el = 0;
	button.JRight.el_tem = 0;
	button.JLeft.az = 0;
	button.JLeft.az_tem = 0;
	button.JLeft.el = 0;
	button.JLeft.el_tem = 0;
	button.JRight.az_max = RWS_MOTOR_PAN_MAX_SPEED;
	button.JRight.el_max = RWS_MOTOR_TILT_MAX_SPEED;
	button.JRight.alpha = 0.5f;
	button.JRight.betha = 1.0f - button.JRight.alpha;
	button.JLeft.az_max = 1000UL;
	button.JLeft.el_max = 1000UL;
	button.JLeft.alpha = 1.0f;
	button.JLeft.betha = 1.0f - button.JLeft.alpha;
	*(uint16_t*) &button.panelButton.input = 0;
	*(uint8_t*) &button.panelButton.output = 0xF;

	/* enable timer2 */
	HAL_TIM_Base_Start(&htim2);
	/* start ADC as DMA */
	HAL_ADC_Start_DMA(&hadc2, adcVal, 5);

	/* tim3 psc = 16999 */
	button.fire_mode_arr.mode_1_arr = 1000UL;
	button.fire_mode_arr.mode_3_arr = 10000UL;
}

static int32_t adc_joystick_azel_conversion(const uint32_t adcVal, const uint32_t maxValue)
{
	int _i;
	float _f = 0.0f;
	long in_min = JS_ADC_MIN - JS_ADC_MID;
	long in_max = JS_ADC_MAX - JS_ADC_MID;
	const int32_t out_min = -900; /* -90.0 degrees */
	const int32_t out_max = 900; /* 90.0 degrees */
	long _l;
	int32_t ret = 0;

	_l = adcVal - JS_ADC_MID;
	if (labs(_l) < JS_ADC_HYST)
		_l = 0;
	_i = (int) constrain(map(_l, in_min, in_max, out_min, out_max), out_min, out_max);

	if (abs(_i) > 50) {
		/* if below 30 degrees */
		if (abs(_i) < 300)
			_i /= 5;
		_f = (float) _i / 10.0f;
		ret = (int32_t) (((1.0 - cos(_f * M_PI / 180.0)) * maxValue));
		if (ret >= (maxValue * 90 / 100))
			ret = maxValue;
	}
	if (_f < 0)
		ret = 0 - ret;

	return ret;
}

static void adc_vbat_conversion(TButton *btn, const uint32_t adcVal)
{
	long in_min = 0, in_max = 3410;
	long out_min = 0, out_max = 300 /* 30.0V */;

	btn->vBat = constrain(map(adcVal, in_min, in_max, out_min, out_max), out_min, out_max);
	/* Vf diode removal, 0.5V */
	if (btn->vBat >= 5)
		btn->vBat -= 5;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hadc);
	adcConvCompleted = 1;
}

static int32_t btnParsingData(char *s)
{
	int32_t ret = -1;
	uint8_t startHeader = 0, startContent = 0;
	uint16_t len;
	char hBuf[8] = { 0 }, cBuf[16] = { 0 };
	char c[2] = { 0, 0 };

	len = strlen(s);
	for ( int i = 0; i < len; i++ ) {
		c[0] = s[i];

		if (c[0] == '*')
			break;

		if (startHeader > 0) {
			if (startContent == 0) {
				if (c[0] == ',')
					startContent = 1;
				else
					strcat(hBuf, c);
			}
			else
				strcat(cBuf, c);
		}
		else {
			if (c[0] == '$')
				startHeader = 1;
		}
	}

	int cmp = strcmp("BTN", hBuf);
	if (cmp == 0)
		ret = strtol(cBuf, NULL, 16);

#if BUTTON_DEBUG==1
	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen = sprintf(buf, "btn= %d %s -> %lX;TE= %d=%d\r\n", cmp, s, ret,
			bus.panelButtonCommand.weaponCommand.triggerEnable,
			button.panelButton.input.trig_enable);
	serial_write_str(&debug, buf, bufLen);
#endif	//if BUTTON_DEBUG==1

	return ret;
}

static void panel_button_update()
{
	static uint16_t _prevPanelButton = 0;
	static uint8_t _lrfManCounter = 0;

	if (_prevPanelButton != *(uint16_t*) &button.panelButton.input) {
		sModeGpioInput_t *in = &button.panelButton.input;
		Panel_button_command_t *send = &bus.panelButtonCommand;

		/* TRIGGER */
		send->weaponCommand.triggerEnable = in->trig_enable;
		/* firing mode */
		if (in->firing_mode == FIRING_MODE_1)
			__HAL_TIM_SET_AUTORELOAD(&htim3, button.fire_mode_arr.mode_1_arr - 1);
		else if (in->firing_mode == FIRING_MODE_3)
			__HAL_TIM_SET_AUTORELOAD(&htim3, button.fire_mode_arr.mode_3_arr - 1);
		/* max manual speed */
		if (in->spd_select == SPD_SELECT_MAX) {
			button.JRight.az_max = MTR_AZ_SPEED_3;
			button.JRight.el_max = MTR_EL_SPEED_3;
		}
		else if (in->spd_select == SPD_SELECT_MID) {
			button.JRight.az_max = MTR_AZ_SPEED_2;
			button.JRight.el_max = MTR_EL_SPEED_2;
		}
		else {
			button.JRight.az_max = MTR_AZ_SPEED_1;
			button.JRight.el_max = MTR_EL_SPEED_1;
		}
		/* cam select */
		if (send->cameraCommand.cameraActive != in->cam_select) {
			send->cameraCommand.zoom = comp.send.state.zoom = OPT_ZOOM_LEVEL_1;
			send->cameraCommand.cameraActive = comp.send.state.camera = in->cam_select;
		}

		/* focus mode */
		send->cameraCommand.focus = in->focus_mode;
		/* lrf */
		send->lrfImuCommand.lrfEnable = in->lrf_enable;
		send->lrfImuCommand.lrfStart = in->lrf_start;
		if (in->lrf_man_mode == LRF_MAN_UP) {
			if (_lrfManCounter == 0) {
				comp.send.lrfRange /= 50;
				comp.send.lrfRange = (comp.send.lrfRange * 50) + 50;
				if (++_lrfManCounter > 4)
					_lrfManCounter = 0;
			}
		}
		else if (in->lrf_man_mode == LRF_MAN_DOWN) {
			if (_lrfManCounter == 0) {
				comp.send.lrfRange /= 50;
				if (comp.send.lrfRange > 0)
					comp.send.lrfRange = (comp.send.lrfRange * 50) - 50;
				if (++_lrfManCounter > 4)
					_lrfManCounter = 0;
			}
		}
		else
			_lrfManCounter = 0;
		/* target selection */
		comp.send.state.trackerTargetSelect = in->target_select;

		_prevPanelButton = *(uint16_t*) &button.panelButton.input;
	}
}

static void joystick_azel_smoothing(TJoystick *js, const int32_t new_az_val,
		const int32_t new_el_val)
{
	js->az = (int32_t) ((js->alpha * new_az_val) + (js->betha * js->az_tem));
	js->az_tem = js->az;

	js->el = (int32_t) ((js->alpha * new_el_val) + (js->betha * js->el_tem));
	js->el_tem = js->el;

}

void button_handler()
{
	static uint32_t _writeLedTimer = 0;
	char _c[2] = { 0, 0 };
	uint8_t _btnCompleted = 0;
	uint8_t _pinState = RESET;
	uint8_t _deadmanState = RESET;
	Panel_camera_command_t *camCmd = &bus.panelButtonCommand.cameraCommand;
	const int32_t jLeftLowVal = 400;
	const int32_t jLeftHighVal = 600;
	int32_t _i32;

	char buf[RING_BUFFER_TX_SIZE];
	uint16_t bufLen;

#if BUTTON_DEBUG==1
	static uint8_t displayCounter = 0;
#endif	//if BUTTON_DEBUG==1

	/* Joystick handler */
	if (adcConvCompleted > 0) {
		adcConvCompleted = 0;

		/* trigger get high priority */
		button.JRight.button.deadman = HAL_GPIO_ReadPin(JSR_DEADMAN_GPIO_Port, JSR_DEADMAN_Pin);
		button.JLeft.button.deadman = HAL_GPIO_ReadPin(JSL_DEADMAN_GPIO_Port, JSL_DEADMAN_Pin);
		if ((button.JRight.button.deadman == SET) || (button.JLeft.button.deadman == SET))
			_deadmanState = SET;

		_pinState = HAL_GPIO_ReadPin(JSR_TRIG_GPIO_Port, JSR_TRIG_Pin);
		if (button.JRight.button.trigger != _pinState) {
			/* trigger-off get high priority */
			if (_pinState == RESET) {
				tim4_out_off(TRIG_CH);
				HAL_TIM_Base_Stop_IT(&htim3);
				tim4_out_off(TRIG_CH); /* make sure firing is off */
			}
			else {
				if ((_deadmanState != RESET) && (button.panelButton.input.trig_enable != RESET)) {
					tim4_out_on(TRIG_CH);
//					if ((button.panelButton.input.firing_mode == FIRING_MODE_1)
//							|| (button.panelButton.input.firing_mode == FIRING_MODE_3))
//						HAL_TIM_Base_Start_IT(&htim3);
				}
				else {
					tim4_out_off(TRIG_CH);
//					HAL_TIM_Base_Stop_IT(&htim3);
//					tim4_out_off(TRIG_CH);
				}
			}
			button.JRight.button.trigger = _pinState;
		}

#if BUTTON_DEBUG==1
		if (displayCounter == 0) {
			bufLen = sprintf(buf, "[%ld]adcVal= %ld %ld %ld %ld %ld\r\n", HAL_GetTick(), adcVal[0],
					adcVal[1], adcVal[2], adcVal[3], adcVal[4]);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if BUTTON_DEBUG==1

		button.JRight.az = adc_joystick_azel_conversion(adcVal[0], button.JRight.az_max);
		button.JRight.el = adc_joystick_azel_conversion(adcVal[1], button.JRight.el_max);
//		int32_t az = adc_joystick_azel_conversion(adcVal[0], button.JRight.az_max);
//		int32_t el = adc_joystick_azel_conversion(adcVal[1], button.JRight.el_max);
//		joystick_azel_smoothing(&button.JRight, az, el);

		button.JLeft.az = adc_joystick_azel_conversion(adcVal[2], button.JLeft.az_max);
		button.JLeft.el = adc_joystick_azel_conversion(adcVal[3], button.JLeft.el_max);

		/* ZOOM command */
		_i32 = abs(button.JLeft.el);
		if (camCmd->zoom == ZF_STOP) {
			if (_i32 >= jLeftHighVal) {
				if (button.JLeft.el > 0)
					bus.panelButtonCommand.cameraCommand.zoom = ZF_IN;
				else
					bus.panelButtonCommand.cameraCommand.zoom = ZF_OUT;
			}
		}
		else {
			if (_i32 <= jLeftLowVal)
				bus.panelButtonCommand.cameraCommand.zoom = ZF_STOP;
		}

		/* TRACK GATE resize command */
		_i32 = abs(button.JLeft.az);
		if (comp.send.state.trackerGateResize == PC_TRK_GATE_STOP) {
			if (_i32 >= jLeftHighVal) {
				if (button.JLeft.az > 0)
					comp.send.state.trackerGateResize = PC_TRK_GATE_BIGGER;
				else
					comp.send.state.trackerGateResize = PC_TRK_GATE_SMALLER;
			}
		}
		else {
			if (_i32 <= jLeftLowVal)
				comp.send.state.trackerGateResize = PC_TRK_GATE_STOP;
		}

		adc_vbat_conversion(&button, adcVal[4]);
#if BUTTON_DEBUG==1
		if (displayCounter == 0) {
			bufLen = sprintf(buf, "JS adc= %ld %ld %ld %ld %ld\r\n", button.JRight.az,
					button.JRight.el, button.JLeft.az, button.JLeft.el, button.vBat);
			serial_write_str(&debug, buf, bufLen);
		}
#endif	//if BUTTON_DEBUG==1

		_pinState = HAL_GPIO_ReadPin(JSL_TRIG_GPIO_Port, JSL_TRIG_Pin);
		if (button.JLeft.button.trigger != _pinState) {
			/* TODO balistic mode*/

			button.JLeft.button.trigger = _pinState;
		}

		_pinState = HAL_GPIO_ReadPin(JSL_B_GPIO_Port, JSL_B_Pin);
		if (button.JLeft.button.bButton != _pinState) {
			if ((_deadmanState != RESET) && (_pinState == SET)) {
				/* TODO memory mode*/
				if (move.mode != MODE_MEMORY)
					move_memory_mode(MODE_START);
				else
					move_memory_mode(MODE_STOP);
			}

			button.JLeft.button.bButton = _pinState;
		}

		_pinState = HAL_GPIO_ReadPin(JSL_A_GPIO_Port, JSL_A_Pin);
		if (button.JLeft.button.aButton != _pinState) {
			if ((_deadmanState != RESET) && (_pinState == SET)) {
				/* TODO start tracking mode*/
				if ((move.mode != MODE_MEMORY)) {
					if (move.mode != MODE_TRACK)
						move_track_mode(MODE_START);
					else
						move_track_mode(MODE_STOP);
				}
			}

			button.JLeft.button.aButton = _pinState;
		}

		_pinState = HAL_GPIO_ReadPin(JSR_B_GPIO_Port, JSR_B_Pin);
		if (button.JRight.button.bButton != _pinState) {
			if ((_deadmanState != RESET) && (_pinState == SET)) {
				/* TODO stabilize mode*/
				if ((move.mode != MODE_MEMORY) || (move.mode != MODE_TRACK)) {
					if (move.mode == MODE_STABILIZE)
						move_stabilize_mode(MODE_STOP);
					else
						move_stabilize_mode(MODE_START);
				}
			}

			button.JRight.button.bButton = _pinState;
		}

		_pinState = HAL_GPIO_ReadPin(JSR_A_GPIO_Port, JSR_A_Pin);
		if (button.JRight.button.aButton != _pinState) {
			if ((_deadmanState != RESET) && (_pinState == SET)) {
				/* TODO travel mode*/
				if (move.mode == MODE_TRAVEL)
					move_travel_mode(MODE_STOP);
				else if (move.mode == MODE_MANUAL)
					move_travel_mode(MODE_START);
			}

			button.JRight.button.aButton = _pinState;
		}

#if BUTTON_DEBUG==1
		if (displayCounter == 0) {
			bufLen = sprintf(buf, "JS btn= %X %X\r\n", *(uint8_t*) &button.JRight.button,
					*(uint8_t*) &button.JLeft.button);
			serial_write_str(&debug, buf, bufLen);
		}
		if (++displayCounter >= 10)
			displayCounter = 0;
#endif	//if BUTTON_DEBUG==1
	}

	if (HAL_GetTick() >= _writeLedTimer) {
		_writeLedTimer = HAL_GetTick() + button_update_timeout_ms;

		bufLen = sprintf(buf, "$PNL,%X*", *(uint8_t*) &button.panelButton.output);
		serial_write_str(button.serial, buf, bufLen);
	}

	if (serial_available(button.serial) > 0) {
		_c[0] = serial_read(button.serial);

		if (_c[0] == '$')
			memset(btnStr, 0, RING_BUFFER_RX_SIZE);
		else if (_c[0] == '*')
			_btnCompleted = 1;
		strcat(btnStr, _c);
	}

	if (_btnCompleted > 0) {
		int32_t parseIn = btnParsingData(btnStr);
		if (parseIn >= 0) {
			*(uint16_t*) &button.panelButton.input = (uint16_t) parseIn;
			panel_button_update();
		}
	}
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&serialButton);
}

void tim4_out_on(const uint32_t ch)
{
	__HAL_TIM_SET_COMPARE(&htim4, ch, TIM4_ON);
}

void tim4_out_off(const uint32_t ch)
{
	__HAL_TIM_SET_COMPARE(&htim4, ch, TIM4_OFF);
}

void tim4_out_toggle(const uint32_t ch)
{
	uint32_t tmp = __HAL_TIM_GET_COMPARE(&htim4, ch);
	if (tmp != TIM4_OFF)
		tmp = TIM4_OFF;
	else
		tmp = TIM4_ON;
	__HAL_TIM_SET_COMPARE(&htim4, ch, tmp);
}

void TIM3_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim3;

	/* TIM Update event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

			tim4_out_off(TRIG_CH);
			HAL_TIM_Base_Stop_IT(&htim3);
		}
	}
}
