/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app/common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMailQId mail_busHandle;
osMailQId mail_motorHandle;
osMailQId mail_motorExtHandle;
osMailQId mail_weaponHandle;

/* USER CODE END Variables */
osThreadId TManagerHandle;
osThreadId TBusHandle;
osThreadId TMotorHandle;
osThreadId TWeaponHandle;
osMessageQId q_managerHandle;
osTimerId timWeaponHandle;
osTimerId timMotorHandle;
osMutexId mx_motor_stateHandle;
osMutexId mx_weapon_stateHandle;
osMutexId mx_motor_ext_stateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void mail_setup();

/* USER CODE END FunctionPrototypes */

void t_manager(void const * argument);
extern void t_bus(void const * argument);
extern void t_motor(void const * argument);
extern void t_weapon(void const * argument);
extern void tim_weapon_callback(void const * argument);
extern void tim_motor_callback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of mx_motor_state */
  osMutexDef(mx_motor_state);
  mx_motor_stateHandle = osMutexCreate(osMutex(mx_motor_state));

  /* definition and creation of mx_weapon_state */
  osMutexDef(mx_weapon_state);
  mx_weapon_stateHandle = osMutexCreate(osMutex(mx_weapon_state));

  /* definition and creation of mx_motor_ext_state */
  osMutexDef(mx_motor_ext_state);
  mx_motor_ext_stateHandle = osMutexCreate(osMutex(mx_motor_ext_state));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of timWeapon */
  osTimerDef(timWeapon, tim_weapon_callback);
  timWeaponHandle = osTimerCreate(osTimer(timWeapon), osTimerPeriodic, NULL);

  /* definition and creation of timMotor */
  osTimerDef(timMotor, tim_motor_callback);
  timMotorHandle = osTimerCreate(osTimer(timMotor), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of q_manager */
  osMessageQDef(q_manager, 6, uint16_t);
  q_managerHandle = osMessageCreate(osMessageQ(q_manager), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	mail_setup();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TManager */
  osThreadDef(TManager, t_manager, osPriorityHigh, 0, 512);
  TManagerHandle = osThreadCreate(osThread(TManager), NULL);

  /* definition and creation of TBus */
  osThreadDef(TBus, t_bus, osPriorityLow, 0, 1024);
  TBusHandle = osThreadCreate(osThread(TBus), NULL);

  /* definition and creation of TMotor */
  osThreadDef(TMotor, t_motor, osPriorityBelowNormal, 0, 1024);
  TMotorHandle = osThreadCreate(osThread(TMotor), NULL);

  /* definition and creation of TWeapon */
  osThreadDef(TWeapon, t_weapon, osPriorityAboveNormal, 0, 1024);
  TWeaponHandle = osThreadCreate(osThread(TWeapon), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_t_manager */
/**
 * @brief  Function implementing the TManager thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_t_manager */
__weak void t_manager(void const * argument)
{
  /* USER CODE BEGIN t_manager */
	/* Infinite loop */
	for ( ;; ) {
		osDelay(1);
	}
  /* USER CODE END t_manager */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
osMessageQId opt_get_queue(const Queue_ID qID)
{
	switch (qID)
	{
	case Q_MANAGER_NOTIF:
		return q_managerHandle;
	default:
		return 0;
	}
}

osMailQId mtr_get_mail(const Mail_ID id)
{
	if (id == Mail_Bus_id)
		return mail_busHandle;
	else if (id == Mail_Motor_id)
		return mail_motorHandle;
	else if (id == Mail_Weapon_id)
		return mail_weaponHandle;
	else if (id == Mail_Motor_Ext_id)
		return mail_motorExtHandle;

	return NULL;
}

osTimerId mtr_get_timer(const Timer_ID tID)
{
	switch (tID)
	{
	case Tim_Weapon_id:
		return timWeaponHandle;
	case Tim_Motor_id:
		return timMotorHandle;
	default:
		return NULL;
	}
}

osMutexId mtr_get_mutex(const Mutex_ID mId)
{
	switch (mId)
	{
	case Mutex_Motor_id:
		return mx_motor_stateHandle;
	case Mutex_motor_ext_id:
		return mx_motor_ext_stateHandle;
	case Mutex_Weapon_id:
		return mx_weapon_stateHandle;
	default:
		return NULL;
	}
}

static void mail_setup()
{
	/* Create the mail queue used by the command_proc tasks to pass the struct MAIL_Bus_t */
	osMailQDef(mail_bus, 10, MAIL_Bus_t); /* Define mail queue */
	mail_busHandle = osMailCreate(osMailQ(mail_bus), NULL); /* create mail queue */

	/* Create the mail queue used by the command_proc tasks to pass the struct MAIL_Motor_t */
	osMailQDef(mail_motor, 20, MAIL_Motor_t); /* Define mail queue */
	mail_motorHandle = osMailCreate(osMailQ(mail_motor), NULL); /* create mail queue */

	/* Create the mail queue used by the command_proc tasks to pass the struct MAIL_Motor_t */
	osMailQDef(mail_motor_ext, 10, MAIL_Motor_Ext_t); /* Define mail queue */
	mail_motorExtHandle = osMailCreate(osMailQ(mail_motor_ext), NULL); /* create mail queue */

	/* Create the mail queue used by the command_proc tasks to pass the struct MAIL_Weapon_t */
	osMailQDef(mail_weapon, 10, MAIL_Weapon_t); /* Define mail queue */
	mail_weaponHandle = osMailCreate(osMailQ(mail_weapon), NULL); /* create mail queue */

}
/* USER CODE END Application */

