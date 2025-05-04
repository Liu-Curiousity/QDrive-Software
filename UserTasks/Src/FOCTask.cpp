#include "task_public.h"
#include "FOC.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "Encoder_AS5047P.h"
#include "BLDC_Driver_FD6288.h"
#include "Storage_EmbeddedFlash.h"
#include "CurrentSensor_Embed.h"
#include "filters.h"

Storage_EmbeddedFlash storage;
BLDC_Driver_DRV8300 bldc_driver(&htim8, 2125);
Encoder_AS5047P bldc_encoder(SPI2_CSn_GPIO_Port, SPI2_CSn_Pin, &hspi2);
CurrentSensor_Embed current_sensor(&hadc1, &hadc2);

LowPassFilter_2_Order CurrentQFilter(0.00005f, 1500); // 20kHz
LowPassFilter_2_Order CurrentDFilter(0.00005f, 1500); // 20kHz
LowPassFilter_2_Order SpeedFilter(0.00005f, 160);     // 20kHz

__attribute__((section(".ccmram")))
FOC foc(14, 1000, 20000,
        CurrentQFilter, CurrentDFilter, SpeedFilter,
        bldc_driver, bldc_encoder, storage, current_sensor,
        PID(PID::delta_type, 10, 1, 0, 0, 0, 1.0f, -1.0f),
        PID(PID::delta_type, 10, 1, 0, 0, 0, 1.0f, -1.0f),
        PID(PID::position_type, 3.2234e-3f, 1.6117e-5f, 0, 5e3f, -5e3f),
        PID(PID::delta_type, 1200.0f, 0, 0));

void StartFOCTask(void *argument) {
    HAL_TIM_Base_Start_IT(&htim6);            // 开启速度环位置环中断控制
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); //开启PWM输出,用于触发ADC采样
    foc.init();                               // 初始化FOC
    while (true) {
        if (!LL_ADC_REG_IsConversionOngoing(hadc1.Instance)) {
            LL_ADC_REG_StartConversion(hadc1.Instance);
            foc.updateVbus(hadc2.Instance->DR / 4095.0f * 3.3f / 2 * 17);
            LL_ADC_REG_StopConversion(hadc1.Instance);
        }
        delay(1);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        current_sensor.update();
        foc.loopCtrl();
    }
}

__attribute__((section(".ccmram_func")))
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
        foc.Ctrl_ISR();
    }
}
