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

#include "AS5047P.h"

class Encoder {
public:
    Encoder(GPIO_TypeDef *CS_GPIO_Port_, const uint16_t CS_GPIO_Pin_, SPI_HandleTypeDef *hspi,
            const int16_t ZeroPosition_Calibration) :
        ZeroPosition_Calibration(ZeroPosition_Calibration) {
        AS5047P_Init(&AS5047P, hspi, CS_GPIO_Port_, CS_GPIO_Pin_);
    }

    void start();
    void stop();
    float read_angle();

private:
    AS5047P_TypeDef AS5047P{};
    int16_t ZeroPosition_Calibration{0}; //零位校准值
};

#endif //ENCODER_DRIVER_AS5047P_H
