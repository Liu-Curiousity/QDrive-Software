/**
 * @brief   BLDC驱动库_FD6288版
 * @details BLDC驱动库封装并提供以下接口:
 *          start()    启动BLDC驱动
 *          stop()     关闭BLDC驱动
 *          set_duty()  设置BLDC三相占空比,归一化
 * @author  LiuHaoqi
 * @date    2025-1-20
 * @version V2.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-5-12
		    V2.0.0创建于2025-1-20,使用C++重构
 * */

#ifndef BLED_Driver_FD6288_H
#define BLED_Driver_FD6288_H

#include "main.h"

class BLDC_Driver {
public:
    BLDC_Driver(TIM_HandleTypeDef *htim, uint16_t MaxDuty) : htim(htim), MaxDuty(MaxDuty) {}

    void start() const;
    void stop() const;
    void set_duty(float u, float v, float w) const;
private:
    TIM_HandleTypeDef *htim;
    uint16_t MaxDuty;
};

#endif //BLED_Driver_FD6288_H
