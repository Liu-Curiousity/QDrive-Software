#include "sys_public.h"
#include "task_public.h"
#include "cmath"

#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

#include "FOC.h"


BLDC_Driver bldc_driver(&htim1, 2125);
Encoder bldc_encoder(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1, 1275);
PID PID_CurrentQ(PID::delta_type, -3e-4f, -1.4e-5f, 0, 0, 0, 1.0f, -1.0f);
PID PID_CurrentD(PID::delta_type, -3e-4f, -1.5e-5f, 0);
// PID PID_Speed(PID::position_type, -2.4f, -0.025f, 0, 5e3f, -5e3f);
PID PID_Speed(PID::position_type, -9.0f, -0.05f, 0, 5e3f, -5e3f);
// PID PID_Position(PID::delta_type, -800.0f, 0, 0);
PID PID_Position(PID::delta_type, -600.0f, 0, 0);

//__attribute__((section(".ccmram")))
FOC foc(7, 1000, 0.8f, 0.5f, bldc_driver, bldc_encoder,
        PID_CurrentQ, PID_CurrentD, PID_Speed, PID_Position);

uint16_t I_Values[3];

void App_DebugTask(void *argument) {
    ///1.串口重定向
    RetargetInit(&huart3);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); //校准ADC
    ///2.启动
    HAL_TIM_Base_Start_IT(&htim6); //开启速度环位置环中断控制
    foc.start();                   //启动FOC

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);                             //开启PWM输出,用于触发ADC采样
    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(I_Values), 2); //开启ADC采样


    /* Infinite loop */
    foc.Ctrl(FOC::CtrlType::PositionCtrl, M_PI_2); //设置目标位置
    // foc.Ctrl(FOC::CtrlType::SpeedCtrl, 500);
    // foc.Ctrl(FOC::CtrlType::CurrentCtrl, 100);
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
        delay(1000);
    }
    /* USER CODE END App_DebugTask */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static bool flag = false;
    static float angle = 0;
    if (GPIO_Pin == KEY1_Pin) {
        if (flag == false) {
            foc.Ctrl(FOC::CtrlType::PositionCtrl, M_PI_2);
            //            FOC_Ctrl(&FOC, FOC_SpeedCtrl, 300);
            //            FOC_Ctrl(&FOC, FOC_CurrentCtrl, 50);
            flag = true;
        } else {
            foc.Ctrl(FOC::CtrlType::PositionCtrl, M_PI_2 * 2);
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
        volatile float Iu = (float)I_Values[0] - 2048;
        volatile float Iv = (float)I_Values[1] - 2048;
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_SET);
        foc.CurrentLoopCtrl_ISR(Iu, Iv);
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_RESET);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
        //        FOC_Ctrl(&FOC, FOC_CurrentCtrl, 1.5 * logf(fabsf(FOC.Speed) + 1) * (FOC.Speed > 0 ? 1.0f : -1.0f));
        //        FOC_Ctrl(&FOC, FOC_CurrentCtrl, -FOC.Speed * 1.5f);
        foc.Ctrl_ISR();
    }
}
