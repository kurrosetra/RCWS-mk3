/*
 * stab.c
 *
 *  Created on: May 27, 2022
 *      Author: 62812
 */

#include "t_motor.h"

void t_motor_stab(void const * argument)
{
  /* USER CODE BEGIN t_motor_stab */
	*(uint8_t*) &motor.mode_state = 0;
	motor.mode_state.movementMode = MOVE_MODE_STAB;

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
	/* USER CODE END t_motor_stab */
}

void t_motor_travel(void const * argument)
{
  /* USER CODE BEGIN t_motor_travel */
	*(uint8_t*) &motor.mode_state = 0;
	motor.mode_state.movementMode = MOVE_MODE_TRAVEL;

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
	/* USER CODE END t_motor_travel */
}
