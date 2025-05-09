/**
 * @brief   Encoder MT6825 Version
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

#ifndef ENCODER_DRIVER_MT6825_H
#define ENCODER_DRIVER_MT6825_H

#include <numbers>
#include "Encoder.h"
#include "spi.h"
#include "gpio.h"

class Encoder_MT6825 final : public Encoder {
public:
    ~Encoder_MT6825() override = default;

    Encoder_MT6825(GPIO_TypeDef *CS_GPIO_Port,
                   const uint16_t CS_GPIO_Pin,
                   SPI_HandleTypeDef *hspi):
        hspi(hspi),
        CS_GPIO_Port(CS_GPIO_Port),
        CS_GPIO_Pin(CS_GPIO_Pin) {}

    void init() override {
        static uint8_t txData = 0x83;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        HAL_SPI_Transmit(hspi, &txData, 1, HAL_MAX_DELAY); // 这句必须加,不然CSn片选时MOSI还是高电平
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, &txData, 1, HAL_MAX_DELAY);
        initialized = true;
    }

    void enable() override {
        if (!initialized) return;
        enabled = true;
    }

    void disable() override {
        if (!initialized) return;
        enabled = false;
    }

    float get_angle() override {
        static uint8_t rxData[3]{};
        if (!enabled) return 0;
        HAL_SPI_Receive(hspi, rxData, 3,HAL_MAX_DELAY);
        return (rxData[0] << 10 | (rxData[1] & 0xFC) << 2 | rxData[2] >> 4) / 262144.0f
               * 2 * std::numbers::pi_v<float>;
    }

private:
    SPI_HandleTypeDef *hspi = nullptr;
    GPIO_TypeDef *CS_GPIO_Port = nullptr;
    uint16_t CS_GPIO_Pin = 0;
};

#endif //ENCODER_DRIVER_MT6825_H
