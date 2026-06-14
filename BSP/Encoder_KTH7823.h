/**
 * @brief   Encoder KTH7823 Version
 * @details
 * @author  Haoqi Liu
 * @date    2026-6-114
 * @version V3.1.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-7-3
		    V2.0.0 on 2025-1-20,refactor by C++
		    V3.0.0 on 2025-4-16,delete ZeroPosition_Calibration and put it in FOC Class
		    V3.1.0 on 2026-6-14,add resolution
 * @copyright   (c) 2026 QDrive
 * */

#ifndef ENCODER_DRIVER_KTH7823_H
#define ENCODER_DRIVER_KTH7823_H

#include <numbers>
#include "Encoder.h"
#include "spi.h"
#include "gpio.h"

class Encoder_KTH7823 final : public Encoder {
public:
    ~Encoder_KTH7823() override = default;

    Encoder_KTH7823(GPIO_TypeDef *CS_GPIO_Port,
                    const uint16_t CS_GPIO_Pin,
                    SPI_HandleTypeDef *hspi) :
        hspi(hspi),
        CS_GPIO_Port(CS_GPIO_Port),
        CS_GPIO_Pin(CS_GPIO_Pin) {}

    void init() override {
        resolution = 2 * std::numbers::pi_v<float> / 65535.0f;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        initialized = true;
    }

    void enable() override {
        uint16_t txData = 0b11'101000'00000010; // 屏蔽寄存器设置功能
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, reinterpret_cast<uint8_t *>(&txData), 1, 100);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        enabled = true;
    }

    void disable() override {
        if (!initialized) return;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        enabled = false;
    }

    float get_angle() override {
        static uint16_t rxData;
        static uint16_t txData = 0x0000;
        if (!enabled) return 0;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, reinterpret_cast<uint8_t *>(&txData),
                                reinterpret_cast<uint8_t *>(&rxData), 1, 100);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        return rxData * resolution; //转化为弧度制
    }

private:
    SPI_HandleTypeDef *hspi = nullptr;
    GPIO_TypeDef *CS_GPIO_Port = nullptr;
    uint16_t CS_GPIO_Pin = 0;
};

#endif //ENCODER_DRIVER_KTH7823_H
