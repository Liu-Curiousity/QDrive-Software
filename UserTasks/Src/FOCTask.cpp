#include "task_public.h"
#include "FOC.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "Encoder_AS5047P.h"
#include "BLDC_Driver_FD6288.h"
#include "Storage_EmbeddedFlash.h"
#include "filters.h"

Storage_EmbeddedFlash storage;
BLDC_Driver_DRV8300 bldc_driver(&htim8, 2125);
Encoder_AS5047P bldc_encoder(SPI2_CSn_GPIO_Port, SPI2_CSn_Pin, &hspi2);
PID PID_CurrentQ(PID::delta_type, 8e-3f, 8e-4f, 0, 0, 0, 1.0f, -1.0f);
PID PID_CurrentD(PID::delta_type, 8e-3f, 8e-4f, 0, 0, 0, 1.0f, -1.0f);
PID PID_Speed(PID::position_type, 4.0f/*最大6,不能再大了*/, 0.02f, 0, 5e3f, -5e3f);
PID PID_Position(PID::delta_type, 1200.0f, 0, 0);

LowPassFilter_2_Order CurrentQFilter(0.00005f, 1500); // 20kHz
LowPassFilter_2_Order CurrentDFilter(0.00005f, 1500); // 20kHz
LowPassFilter_2_Order SpeedFilter(0.00005f, 160);     // 20kHz
__attribute__((section(".ccmram")))
FOC foc(14, 1000, 20000, CurrentQFilter, CurrentDFilter, SpeedFilter,
        bldc_driver, bldc_encoder, storage, PID_CurrentQ, PID_CurrentD, PID_Speed, PID_Position);

void StartFOCTask(void *argument) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); //校准ADC
    HAL_TIM_Base_Start_IT(&htim6);                         // 开启速度环位置环中断控制
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);              //开启PWM输出,用于触发ADC采样
    ADC_Enable(&hadc1);
    ADC_Enable(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1); //开启ADC采样
    foc.init();                         // 初始化FOC
    while (true) {
        if (!LL_ADC_REG_IsConversionOngoing(hadc1.Instance)) {
            LL_ADC_REG_StartConversion(hadc1.Instance);
            foc.updateVbus(hadc2.Instance->DR / 4095.0f * 3.3f / 2 * 17 + 0.6f/*校准*/);
            LL_ADC_REG_StopConversion(hadc1.Instance);
        }
        delay(1);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        const float Iu = static_cast<float>(hadc2.Instance->JDR1) - 1982.5f;
        const float Iv = (static_cast<float>(hadc1.Instance->JDR1) - 2042.5f) * 1.03f;
        foc.loopCtrl(Iu, Iv);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
        foc.Ctrl_ISR();
    }
}
