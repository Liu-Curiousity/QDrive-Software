/**
* @brief   编码器驱动库_AS5047P版
 * @details 编码器驱动库封装并提供以下接口:
 *          start()    启动编码器
 *          stop()     关闭编码器
 *          read_angle()读取编码器角度,弧度制
 * @author  LiuHaoqi
 * @date    2025-1-20
 * @version V2.0.0
 * @note
 * @warning
 * @par     历史版本:
            V1.0.0创建于2024-7-3
            V2.0.0创建于2025-1-20,使用C++重构
 * */

#include <cmath>
#include "Encoder_Driver_AS5047P.h"

void Encoder::start() {}
void Encoder::stop() {}

// __attribute__((section(".ccmram_func"))) inline
float Encoder::read_angle() {
    static uint16_t rxData;
    AS5047P_ReadAngleContinuously(&AS5047P, &rxData);
    rxData = (rxData + ZeroPosition_Calibration) % 0x3fff; //矫正零点
    return 2 * M_PI - static_cast<double>(rxData) / 0x3fff * 2 * M_PI; //转化为弧度制
}
