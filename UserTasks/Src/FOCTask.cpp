#include "task_public.h"
#include "FOC.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "Encoder_AS5047P.h"
#include "BLDC_Driver_FD6288.h"

uint16_t I_Values[3];

BLDC_Driver_DRV8300 bldc_driver(&htim8, 2125);
// Encoder bldc_encoder(SPI2_CSn_GPIO_Port, SPI2_CSn_Pin, &hspi2, 865);
Encoder_AS5047P bldc_encoder(SPI2_CSn_GPIO_Port, SPI2_CSn_Pin, &hspi2, 955);
PID PID_CurrentQ(PID::delta_type, -1e-3f, -1.0e-4f, 0, 0, 0, 1.0f, -1.0f);
PID PID_CurrentD(PID::delta_type, -1e-3f, -1.0e-4f, 0, 0, 0, 1.0f, -1.0f);
// PID PID_Speed(PID::position_type, 2.4f, 0.018f, 0, 5e3f, -5e3f);
PID PID_Speed(PID::position_type, 4.0f, 0.02f, 0, 5e3f, -5e3f);
// PID PID_Position(PID::delta_type, -900.0f, 0, 0);
PID PID_Position(PID::delta_type, -1200.0f, 0, 0);
// PID PID_Position(PID::delta_type, 10000, 2, 80000); //位置环直接控制电流

__attribute__((section(".ccmram")))
FOC foc(14, 1000, 0.8f, 0.5f, bldc_driver, bldc_encoder,
        PID_CurrentQ, PID_CurrentD, PID_Speed, PID_Position);

void StartFOCTask(void *argument) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); //校准ADC
    ///2.启动
    HAL_TIM_Base_Start_IT(&htim6); //开启速度环位置环中断控制
    foc.init();                    //启动FOC
    foc.start();                   //启动FOC

    //TODO: 该采样方式存在同步问题,需要优化
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);                                 //开启PWM输出,用于触发ADC采样
    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(I_Values), 1);     //开启ADC采样
    HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t *>(I_Values + 1), 1); //开启ADC采样


    /* Infinite loop */
    // foc.Ctrl(FOC::CtrlType::PositionCtrl, M_PI_2); //设置目标位置
    // foc.Ctrl(FOC::CtrlType::SpeedCtrl, 30);
    foc.Ctrl(FOC::CtrlType::CurrentCtrl, 80);
    while (true) {
        osDelay(10);
    }
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
        // angle += 0.7f;
        // FOC_Ctrl(&FOC, FOC_PositionCtrl, angle);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        const float Iu = static_cast<float>(I_Values[0]) - 2045;
        const float Iv = (static_cast<float>(I_Values[1]) - 1982) * 1.03f;
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_SET);
        foc.CurrentLoopCtrl_ISR(Iu, Iv);
        HAL_GPIO_WritePin(TestPin_GPIO_Port, TestPin_Pin, GPIO_PIN_RESET);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
        // 助力
        // foc.Ctrl(FOC::CtrlType::CurrentCtrl, 1.5f * logf(fabsf(foc.speed()) + 1) * (foc.speed() > 0 ? -1.0f : 1.0f));
        // 阻尼
        // foc.Ctrl(FOC::CtrlType::CurrentCtrl, foc.speed() * 1.5f);
        // 重力补偿
        // foc.Ctrl(FOC::CtrlType::CurrentCtrl, 580 * cosf(foc.angle() - 2.76f));
        foc.Ctrl_ISR();
    }
}
