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

#ifndef ENCODER_DRIVER_AS5047P_H
#define ENCODER_DRIVER_AS5047P_H

#include <cmath>

#include "AS5047P.h"
#include "Encoder.h"

class Encoder_AS5047P final : public Encoder {
public:
    ~Encoder_AS5047P() override = default;

    Encoder_AS5047P(GPIO_TypeDef *CS_GPIO_Port, const uint16_t CS_GPIO_Pin, SPI_HandleTypeDef *hspi,
                    const int16_t ZeroPosition_Calibration) :
        CS_GPIO_Port(CS_GPIO_Port),
        CS_GPIO_Pin(CS_GPIO_Pin),
        hspi(hspi),
        ZeroPosition_Calibration(ZeroPosition_Calibration) {}

    void init() override {
        AS5047P_Init(&AS5047P, hspi, CS_GPIO_Port, CS_GPIO_Pin);
        initialized = true;
    }

    void enable() override { enabled = true; }
    void disable() override { enabled = false; }

    float get_angle() override {
        static uint16_t rxData;
        AS5047P_ReadAngleContinuously(&AS5047P, &rxData);
        rxData = (rxData + ZeroPosition_Calibration) % 0x3fff;  //矫正零点
        return static_cast<double>(rxData) / 0x3fff * 2 * M_PI; //转化为弧度制
    }

private:
    AS5047P_TypeDef AS5047P{};
    GPIO_TypeDef *CS_GPIO_Port = nullptr;
    uint16_t CS_GPIO_Pin = 0;
    SPI_HandleTypeDef *hspi = nullptr;
    int16_t ZeroPosition_Calibration{0}; //零位校准值
};

#endif //ENCODER_DRIVER_AS5047P_H
