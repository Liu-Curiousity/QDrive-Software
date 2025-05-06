/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "task_public.h"
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

/* USER CODE END Variables */
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for FOCTask */
osThreadId_t FOCTaskHandle;
const osThreadAttr_t FOCTask_attributes = {
  .name = "FOCTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
/* Definitions for CommunicateTask */
osThreadId_t CommunicateTaskHandle;
const osThreadAttr_t CommunicateTask_attributes = {
  .name = "CommunicateTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void App_DebugTask(void *argument);
extern void StartFOCTask(void *argument);
extern void StartCommunicateTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(App_DebugTask, NULL, &DebugTask_attributes);

  /* creation of FOCTask */
  FOCTaskHandle = osThreadNew(StartFOCTask, NULL, &FOCTask_attributes);

  /* creation of CommunicateTask */
  CommunicateTaskHandle = osThreadNew(StartCommunicateTask, NULL, &CommunicateTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_App_DebugTask */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_App_DebugTask */
__weak void App_DebugTask(void *argument)
{
  /* USER CODE BEGIN App_DebugTask */
    for (;;){

    }
  /* USER CODE END App_DebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

