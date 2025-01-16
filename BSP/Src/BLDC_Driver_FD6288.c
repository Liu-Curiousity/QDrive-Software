/**
 * @brief   BLDC驱动库_FD6288版
 * @details BLDC驱动库封装并提供以下接口:
 *          BLDC_Init()     初始化BLDC句柄,具体接口因方案而异
 *          BLDC_Start()    启动BLDC驱动
 *          BLDC_Stop()     关闭BLDC驱动
 *          BLDC_SetDuty()  设置BLDC三相占空比
 * @author  LiuHaoqi
 * @date    2024-5-12
 * @version V1.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-5-12
 * */

#include "BLDC_Driver_FD6288.h"

int BLDC_Init(BLDC_Driver_HandleTypeDef *Driver, BLDC_Driver_InitTypeDef *Driver_InitStructure) {
    Driver->Init = *Driver_InitStructure;
    return 0;
}

int BLDC_Start(BLDC_Driver_HandleTypeDef *Driver) {
    //打开所有PWM通道输出
    HAL_TIM_PWM_Start(Driver->Init.htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(Driver->Init.htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(Driver->Init.htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(Driver->Init.htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(Driver->Init.htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(Driver->Init.htim, TIM_CHANNEL_3);
    return 0;
}

int BLDC_Stop(BLDC_Driver_HandleTypeDef *Driver) {
    //关闭所有PWM通道输出
    HAL_TIM_PWM_Stop(Driver->Init.htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(Driver->Init.htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(Driver->Init.htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(Driver->Init.htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(Driver->Init.htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(Driver->Init.htim, TIM_CHANNEL_3);
    return 0;
}

__attribute__((section(".ccmram_func"))) inline
void BLDC_SetDuty(BLDC_Driver_HandleTypeDef *Driver, float u, float v, float w) {
    u *= (float) Driver->Init.MaxDuty;
    v *= (float) Driver->Init.MaxDuty;
    w *= (float) Driver->Init.MaxDuty;
    //设置PWM占空比
    __HAL_TIM_SET_COMPARE(Driver->Init.htim, TIM_CHANNEL_1, u);
    __HAL_TIM_SET_COMPARE(Driver->Init.htim, TIM_CHANNEL_2, v);
    __HAL_TIM_SET_COMPARE(Driver->Init.htim, TIM_CHANNEL_3, w);
}
