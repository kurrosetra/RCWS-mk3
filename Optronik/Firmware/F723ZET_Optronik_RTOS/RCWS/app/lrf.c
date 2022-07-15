/*
 * lrf.c
 *
 *  Created on: Jan 17, 2022
 *      Author: miftakur
 */

#include <stdio.h>

#include "common.h"
#include "hal/lrf_hal.h"

/*** Internal Const Values, Macros ***/
#if DEBUG_LRF==1
#define LOG(str, ...) printf("[%ld TLrf:%d] " str, osKernelSysTick(), __LINE__, ##__VA_ARGS__)
#define LOG_E(str, ...) printf("[TLrf_Err:%d] " str, __LINE__, ##__VA_ARGS__)
#else
#	define LOG(str, ...)
#	define LOG_E(str, ...)
#endif	//if DEBUG_LRF==1

uint8_t g_lrf_ready = 0;
static const uint16_t managerNotif = T_Lrf_id;
static Lrf_t lrf_info;

static void lrf_timer_start()
{
	osTimerStart(opt_get_timer(Tim_Lrf_Notif_id), 500);
	osTimerStart(opt_get_timer(Tim_Lrf_Timeout_id), 10000);
	LOG("timer start\r\n");
}

static void lrf_timer_stop()
{
	osTimerStop(opt_get_timer(Tim_Lrf_Notif_id));
	osTimerStop(opt_get_timer(Tim_Lrf_Timeout_id));
	LOG("timer stop\r\n");
}

void osTimerLrfNotifCallback(void const *argument)
{
	/* USER CODE BEGIN osTimerCallback */
	(void) argument;

	/* send notif to task manager that this thread is still running */
	osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);

	/* USER CODE END osTimerCallback */
}

void osTimerLrfTimeoutCallback(void const *argument)
{
	/* USER CODE BEGIN osTimerCallback */
	(void) argument;

	lrf_timer_stop();
	LOG_E("timeout!\r\n");

	/* USER CODE END osTimerCallback */
}

void t_lrf(void const *argument)
{
	/* USER CODE BEGIN t_lrf */

	LOG("created!\r\n");
	lrf_init();

	/* TODO Built-In Test */
	g_lrf_ready = 1;

	osDelay(T_Lrf_id);

	/* Infinite loop */
	for ( ;; ) {
		Panel_lrf_command_t notif;
		osEvent event = osMessageGet(opt_get_queue(Q_LRF_NOTIF), 1000);
		if (event.status == osEventMessage) {
			*(uint8_t*) &notif = (uint8_t) event.value.v;
			if (*(uint8_t*) &lrf_info.command != *(uint8_t*) &notif) {
				if (osMutexWait(opt_get_mutex(M_LRF_STATE), 100) == osOK) {
					lrf_info.busy = 1;
					/* redirect notif to task manager by timer */
					lrf_timer_start();

					/* lrf power changed */
					if (lrf_info.command.lrfEnable != notif.lrfEnable) {
						LOG("lrf power set to %d\r\n", notif.lrfEnable);

						/* set lrf to enable */
						if (notif.lrfEnable == 1) {
							/* set and wait lrf to active */
							if (lrf_power(1) == HAL_OK) {
								/* init lrf_command */
								lrf_info.command.lrfEnable = 1;
								/* update lrf state */
								lrf_info.state.lrf_enable = 1;
								/* set pointer active for test purpose */
								osDelay(300);
								lrf_set_pointer(1);
							}

						}
						/* set lrf to disable */
						else {
							osDelay(100);
							lrf_power(0);
							lrf_info.command.lrfEnable = lrf_info.state.lrf_enable = 0;
						}
					}
					/* other command */
					else {
						/* only if lrf is enabled */
						if (lrf_info.state.lrf_enable == 1) {
							LOG("new lrf command: %d\r\n", *(uint8_t* )&notif);
							/* start measurement */
							if (lrf_info.command.lrfStart != notif.lrfStart) {
								/* TODO start measurement */
//								if (lrf_measure_start() == HAL_OK) {
								uint8_t status = 0;
								if (lrf_measure_start_with_status(&status) == HAL_OK) {
									lrf_info.state.value.counter = lrf_get_counter();
									lrf_get_value((uint32_t*) lrf_info.state.value.d);
									LOG("c=%d;d[3]= %d %d %d\r\n", lrf_get_counter(), lrf_info.state.value.d[0],
											lrf_info.state.value.d[1], lrf_info.state.value.d[2]);
								}
							}
							/* pointer */
							if (lrf_info.command.lrfPointerActive != notif.lrfPointerActive) {
								LOG("pointer set to: %d\r\n", notif.lrfPointerActive);
								lrf_set_pointer(notif.lrfPointerActive);

								lrf_info.command.lrfPointerActive = notif.lrfPointerActive;
							}
							/* continuous measurement */
							if (lrf_info.command.lrfContinousMode != notif.lrfContinousMode) {
								/* TODO set continuous mode */
								lrf_info.command.lrfContinousMode = notif.lrfContinousMode;
							}
						}
					}

					/* stop timer */
					lrf_timer_stop();
					lrf_info.busy = 0;
					osMutexRelease(opt_get_mutex(M_LRF_STATE));
				}
				else
					LOG_E("mutex timeout\r\n");
			}
		}

		/* send notif to task manager that this thread is still running */
		osMessagePut(opt_get_queue(Q_MANAGER_NOTIF), managerNotif, 0);
	}
	/* USER CODE END t_lrf */
}

uint8_t lrf_is_busy()
{
	return lrf_info.busy;
}

osStatus lrf_read(Lrf_t *lrf, const uint32_t timeout)
{
	osStatus ret = osMutexWait(opt_get_mutex(M_LRF_STATE), timeout);
	if (ret == osOK) {
		*lrf = lrf_info;

		ret = osMutexRelease(opt_get_mutex(M_LRF_STATE));
	}

	return ret;
}

osStatus lrf_write(Lrf_t *lrf, const uint32_t timeout)
{
	osStatus ret = osMutexWait(opt_get_mutex(M_LRF_STATE), timeout);
	if (ret == osOK) {
		lrf_info = *lrf;

		ret = osMutexRelease(opt_get_mutex(M_LRF_STATE));
	}

	return ret;
}

