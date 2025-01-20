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
#include <math.h>
#include <string.h>
#include "tim.h"
#include "spi.h"
#include "retarget.h"
#include "usart.h"
#include "adc.h"
#include "FOC.h"
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
//__attribute__((section(".ccmram")))
FOC_HandleTypeDef FOC;

uint16_t I_Values[3];

/* USER CODE END Variables */
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void FOC_MSP_Init(void);

/* USER CODE END FunctionPrototypes */

void App_DebugTask(void *argument);

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
void App_DebugTask(void *argument)
{
  /* USER CODE BEGIN App_DebugTask */
    ///1.初始化
    RetargetInit(&huart3);
    FOC_MSP_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);    //校准ADC
    ///2.启动
    HAL_TIM_Base_Start_IT(&htim6);                                      //开启速度环位置环中断控制
    FOC_Start(&FOC);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);                   //开启PWM输出,用于触发ADC采样
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) I_Values, 2);        //开启ADC采样




    /* Infinite loop */
    FOC_Ctrl(&FOC, FOC_PositionCtrl, M_PI_2);                   //设置目标位置
    // FOC_Ctrl(&FOC, FOC_SpeedCtrl, 500);
    // FOC_Ctrl(&FOC, FOC_CurrentCtrl, 100);
    for (;;) {
        // static uint8_t buffer[50];
        // sprintf(buffer, "%.4f,%.4f,%.4f,%.4f\n", FOC.Iq, FOC.Id, FOC.Speed, FOC.Angle);
        // HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
//        PID_SetTarget(&FOC.PID_CurrentQ, 950 * cosf(FOC.Angle + 5.7));
        // FOC_Ctrl(&FOC, FOC_SpeedCtrl, 500);
        // osDelay(500);
        // FOC_Ctrl(&FOC, FOC_SpeedCtrl, -500);
        // FOC_Ctrl(&FOC, FOC_PositionCtrl, M_PI_2);
        // osDelay(1000);
        // FOC_Ctrl(&FOC, FOC_PositionCtrl, M_PI);
        osDelay(1000);
    }
  /* USER CODE END App_DebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static bool flag = false;
    static float angle = 0;
    if (GPIO_Pin == KEY1_Pin) {
        if (flag == false) {
            FOC_Ctrl(&FOC, FOC_PositionCtrl, M_PI_2);
//            FOC_Ctrl(&FOC, FOC_SpeedCtrl, 300);
//            FOC_Ctrl(&FOC, FOC_CurrentCtrl, 50);
            flag = true;
        } else {
            FOC_Ctrl(&FOC, FOC_PositionCtrl, M_PI_2 * 2);
//            FOC_Ctrl(&FOC, FOC_SpeedCtrl, -300);
//            FOC_Ctrl(&FOC, FOC_CurrentCtrl, -50);
            flag = false;
        }
//        angle += 0.7f;
//        FOC_Ctrl(&FOC, FOC_PositionCtrl, angle);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        volatile float Iu = (float) I_Values[0] - 2048;
        volatile float Iv = (float) I_Values[1] - 2048;
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_SET);
        FOC_CurrentLoopCtrl_ISR(&FOC, Iu, Iv);
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_RESET);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
//        FOC_Ctrl(&FOC, FOC_CurrentCtrl, 1.5 * logf(fabsf(FOC.Speed) + 1) * (FOC.Speed > 0 ? 1.0f : -1.0f));
//        FOC_Ctrl(&FOC, FOC_CurrentCtrl, -FOC.Speed * 1.5f);
        FOC_Ctrl_ISR(&FOC);
    }
}

void FOC_MSP_Init() {
    BLDC_Driver_InitTypeDef Driver_InitStructure = {
            .htim = &htim1,
            .MaxDuty = 2125
    };
    Encoder_Driver_InitTypeDef Encoder_InitStructure = {
            .hspi = &hspi1,
            .CS_GPIO_Port_ = SPI1_CSn_GPIO_Port,
            .CS_GPIO_Pin_ = SPI1_CSn_Pin,
            .ZeroPosition_Calibration = 1275
    };
    FOC_InitTypeDef InitStructure = {
            .Driver_InitStructure = Driver_InitStructure,
            .Encoder_InitStructure = Encoder_InitStructure,
            .PolePairs = 7,
            .CtrlFrequency = 1000,
            .PID_CurrentQ_InitStructure = {
                    .PID_Type = PID_TYPE_DELTA,
                    .Target = 0,
                    .Kp = -3e-4f,
                    .Ki = -1.4e-5f,
                    .Kd = 0,
                    .Limit_Output = 1.0f
            },
            .PID_CurrentD_InitStructure = {
                    .PID_Type = PID_TYPE_DELTA,
                    .Target = 0,
                    .Kp = -3e-4f,
                    .Ki = -1.5e-5f,
                    .Kd = 0,
            },
            .PID_Speed_InitStructure = {
                    .PID_Type = PID_TYPE_POSITION,
                    .Target = 0,
                    // .Kp = -2.4f,
                    // .Ki = -0.025f,
                    /*超强*/
                    .Kp = -9.0f,
                    .Ki = -0.05f,
                    .Kd = 0.0f,
                    .Limit_SumError = 5e3f
            },
            .PID_Position_InitStructure = {
                    .PID_Type = PID_TYPE_DELTA,
                    .Target = 0,
                    // .Kp = -600.0f,
                    /*超强*/
                    .Kp = -800.0f,
                    .Ki = 0.0f,
                    .Kd = 0,
            },
            .CurrentFilter = 0.8f,
            .SpeedFilter = 0.5f
    };
    FOC_Init(&FOC, &InitStructure);
}
/* USER CODE END Application */

