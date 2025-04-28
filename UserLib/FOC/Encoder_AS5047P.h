/**
 * @brief   Encoder AS5047P Version
 * @details
 * @author  Haoqi Liu
 * @date    2025-4-16
 * @version V3.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-7-3
		    V2.0.0 on 2025-1-20,refactor by C++
		    V3.0.0 on 2025-4-16,delete ZeroPosition_Calibration and put it in FOC Class
 * */

#ifndef ENCODER_DRIVER_AS5047P_H
#define ENCODER_DRIVER_AS5047P_H

#include <numbers>
#include "Encoder.h"
#include "gpio.h"

class Encoder_AS5047P final : public Encoder {
public:
    ~Encoder_AS5047P() override = default;

    Encoder_AS5047P(GPIO_TypeDef *CS_GPIO_Port,
                    const uint16_t CS_GPIO_Pin,
                    SPI_HandleTypeDef *hspi):
        hspi(hspi),
        CS_GPIO_Port(CS_GPIO_Port),
        CS_GPIO_Pin(CS_GPIO_Pin) {}

    void init() override {
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        initialized = true;
    }

    void enable() override { enabled = true; }
    void disable() override { enabled = false; }

    float get_angle() override {
        static uint16_t rxData;
        static uint16_t txData = 0xFFFF;

        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, reinterpret_cast<uint8_t *>(&txData),
                                reinterpret_cast<uint8_t *>(&rxData), 1, 100);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        rxData &= 0x3FFF;
        return static_cast<float>(rxData) / 0x3fff * 2 * std::numbers::pi_v<float>; //转化为弧度制
    }

private:
    SPI_HandleTypeDef *hspi = nullptr;
    GPIO_TypeDef *CS_GPIO_Port = nullptr;
    uint16_t CS_GPIO_Pin = 0;
};

#endif //ENCODER_DRIVER_AS5047P_H
