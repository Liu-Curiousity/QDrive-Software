/**
 * @brief   BLDC_Driver base class
 * @details BLDC驱动库封装并提供以下接口:
 *          start()    启动BLDC驱动
 *          stop()     关闭BLDC驱动
 *          set_duty()  设置BLDC三相占空比,归一化
 * @author  LiuHaoqi
 * @date    2025-4-8
 * @version V3.0.0
 * @note
 * @warning
 * @par     history:
		    V1.0.0 on 2024-5-12
		    V2.0.0 on 2025-1-20,refactor by C++
		    V3.0.0 on 2025-4-8,redesign refer to SimpleFOC
 * */

#ifndef BLED_Driver_DRV8300_H
#define BLED_Driver_DRV8300_H

#include <cstdint>
#include "tim.h"
#include "BLDC_Driver.h"

class BLDC_Driver_DRV8300 final : public BLDC_Driver {
public:
    ~BLDC_Driver_DRV8300() override = default;

    BLDC_Driver_DRV8300(TIM_HandleTypeDef *htim, const uint16_t MaxDuty) :
        htim(htim), MaxDuty(MaxDuty) { initialized = true; }

    void init() override { initialized = true; }

    void enable() override {
        enabled = true;
        //打开所有PWM通道输出
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
    }

    void disable() override {
        enabled = false;
        // 设置占空比为0
        set_duty(0, 0, 0);
        // 关闭所有PWM通道输出
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
    }

    void set_duty(float u, float v, float w) override {
        if (enabled) {
            u *= static_cast<float>(MaxDuty);
            v *= static_cast<float>(MaxDuty);
            w *= static_cast<float>(MaxDuty);
            //设置PWM占空比
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, u);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, v);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, w);
        }
    }

private:
    TIM_HandleTypeDef *htim;
    uint16_t MaxDuty;
};

#endif //BLED_Driver_DRV8300_H
