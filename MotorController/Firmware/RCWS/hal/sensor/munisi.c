/*
 * munisi.c
 *
 *  Created on: Jun 15, 2022
 *      Author: 62812
 */

#include "munisi.h"

volatile uint16_t m_counter = 0;
uint8_t state_active = 0;

void munisi_reset()
{
	m_counter = 0;
}

void munisi_set_state(const uint8_t power)
{
	state_active = power;
}

uint8_t munisi_get_state()
{
	return state_active;
}

uint16_t munisi_get_counter()
{
	return m_counter;
}

void EXTI15_10_IRQHandler(void)
{
	/* EXTI line interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(MUNC_A_Pin) != 0x00u) {
		__HAL_GPIO_EXTI_CLEAR_IT(MUNC_A_Pin);

		if (state_active != 0)
			m_counter++;
	}
}
