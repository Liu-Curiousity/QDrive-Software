/**
 * @brief   Encoder MT6826S Version
 * @details
 * @author  Haoqi Liu
 * @date    2026-6-14
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

#ifndef ENCODER_DRIVER_MT6826S_H
#define ENCODER_DRIVER_MT6826S_H

#include <numbers>
#include "Encoder.h"
#include "spi.h"
#include "gpio.h"

class Encoder_MT6826S final : public Encoder {
public:
    ~Encoder_MT6826S() override = default;

    Encoder_MT6826S(GPIO_TypeDef *CS_GPIO_Port,
                    const uint16_t CS_GPIO_Pin,
                    SPI_HandleTypeDef *hspi) :
        hspi(hspi),
        CS_GPIO_Port(CS_GPIO_Port),
        CS_GPIO_Pin(CS_GPIO_Pin) {}

    void init() override {
        resolution = 2 * std::numbers::pi_v<float> / 32768.0f;
        initialized = true;
    }

    void enable() override {
        if (!initialized) return;
        static uint8_t txData[2]{0xA0, 0x03};
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        HAL_SPI_Transmit(hspi, txData, 1, HAL_MAX_DELAY); // 这句必须加,不然CSn片选时MOSI还是高电平
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, txData, 2, HAL_MAX_DELAY);
        enabled = true;
    }

    void disable() override {
        if (!initialized) return;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        enabled = false;
    }

    float get_angle() override {
        static uint8_t rxData[4]{};
        if (!enabled) return 0;
        HAL_SPI_Receive(hspi, rxData, 4,HAL_MAX_DELAY);
        return (rxData[0] << 7 | rxData[1] >> 1) * resolution;
    }

    void start_self_calibrate() const {
        if (enabled) return;

        // 1.配置校准转速
        write_reg(0x00E, (read_reg(0x00E) & 0b1000'1111) | 0b0011'0000); // 011: 200-400rpm
        // 2.启动校准
        write_reg(0x155, 0x5E);
        // HAL_GPIO_WritePin(CAL_EN_GPIO_Port, CAL_EN_Pin, GPIO_PIN_SET);
        // 3.等待校准完成
        // while (read_reg(0x113) >> 6 == 0x01) {
        //     delay(10);
        // }
    }

    [[nodiscard]] uint8_t self_calibrate_status() const {
        if (enabled) return 0x00;
        return read_reg(0x113) >> 6;
    }

private:
    SPI_HandleTypeDef *hspi = nullptr;
    GPIO_TypeDef *CS_GPIO_Port = nullptr;
    uint16_t CS_GPIO_Pin = 0;

    [[nodiscard]] uint8_t read_reg(const uint16_t reg) const {
        if (enabled) return 0;

        static uint8_t txData[2]{0x30, 0x00};
        static uint8_t rxData{0};
        txData[0] = 0x30 | ((reg >> 8) & 0x0F);
        txData[1] = reg;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, txData, 2, HAL_MAX_DELAY);
        HAL_SPI_Receive(hspi, &rxData, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
        return rxData;
    }

    void write_reg(const uint16_t reg, const uint8_t data) const {
        if (enabled) return;
        static uint8_t txData[3]{0x60, 0x00, 0x00};
        txData[0] = 0x60 | ((reg >> 8) & 0x0F);
        txData[1] = reg;
        txData[2] = data;
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, txData, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
    }
};

#endif //ENCODER_DRIVER_MT6826S_H
