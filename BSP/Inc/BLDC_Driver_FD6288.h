/**
 * @brief   BLDC驱动库_FD6288版
 * @details BLDC驱动库封装并提供以下接口:
 *          BLDC_Init()     初始化BLDC句柄,具体接口因方案而异
 *          BLDC_Start()    启动BLDC驱动
 *          BLDC_Stop()     关闭BLDC驱动
 *          BLDC_SetDuty()  设置BLDC三相占空比,归一化
 * @author  LiuHaoqi
 * @date    2024-5-12
 * @version V1.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-5-12
 * */


#ifndef BLED_Driver_FD6288_H
#define BLED_Driver_FD6288_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t MaxDuty;
} BLDC_Driver_InitTypeDef;

typedef struct {
    BLDC_Driver_InitTypeDef Init;
} BLDC_Driver_HandleTypeDef;

extern int BLDC_Init(BLDC_Driver_HandleTypeDef *Driver, BLDC_Driver_InitTypeDef *Driver_InitStructure);

extern int BLDC_Start(BLDC_Driver_HandleTypeDef *Driver);

extern int BLDC_Stop(BLDC_Driver_HandleTypeDef *Driver);

extern void BLDC_SetDuty(BLDC_Driver_HandleTypeDef *Driver, float u, float v, float w);

#endif //BLED_Driver_FD6288_H
