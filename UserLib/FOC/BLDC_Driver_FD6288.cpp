/**
 * @brief   BLDC驱动库_FD6288版
 * @details BLDC驱动库封装并提供以下接口:
 *          start()    启动BLDC驱动
 *          stop()     关闭BLDC驱动
 *          setDuty()  设置BLDC三相占空比,归一化
 * @author  LiuHaoqi
 * @date    2025-1-20
 * @version V2.0.0
 * @note
 * @warning
 * @par     历史版本:
            V1.0.0创建于2024-5-12
            V2.0.0创建于2025-1-20,使用C++重构
 * */

#include "BLDC_Driver_FD6288.h"

void BLDC_Driver::start() const {
    //打开所有PWM通道输出
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
}

void BLDC_Driver::stop() const {
    //关闭所有PWM通道输出
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}

__attribute__((section(".ccmram_func"))) inline
void BLDC_Driver::setDuty(float u, float v, float w) const {
    u *= static_cast<float>(MaxDuty);
    v *= static_cast<float>(MaxDuty);
    w *= static_cast<float>(MaxDuty);
    //设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, u);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, v);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, w);
}
