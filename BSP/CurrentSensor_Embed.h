//
// Created by 26757 on 25-5-4.
//
#pragma once

#include "CurrentSensor.h"
#include "adc.h"

class CurrentSensor_Embed final : public CurrentSensor {
public:
    CurrentSensor_Embed(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2) :
        hadc1(hadc1), hadc2(hadc2) {}

    void init() override {
        HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED); //校准ADC
        HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED); //校准ADC
        initialized = true;
    }

    void enable() override {
        ADC_Enable(hadc1);
        ADC_Enable(hadc2);
        HAL_ADCEx_InjectedStart_IT(hadc1); //开启ADC采样
        enabled = true;
    }

    void disable() override {
        ADC_Disable(hadc1);
        ADC_Disable(hadc2);
        HAL_ADCEx_InjectedStop_IT(hadc1); //开启ADC采样
        enabled = false;
    }

    void update() {
        static constexpr float V_REF = 3.3f;              // ADC基准电压,单位:V
        static constexpr float ADC_REVOLUTION = 4096 - 1; // ADC分辨率
        static constexpr float OP_AMP_GAIN = 20.0f;       // 差分运放电压增益,单位:V/V
        static constexpr float R_SENSE = 0.05f;           // 采样电阻阻值,单位:Ω

        const float iu = static_cast<float>(hadc2->Instance->JDR1) - 2046.0f;
        this->iu = iu / ADC_REVOLUTION * V_REF / OP_AMP_GAIN / R_SENSE;
        const float iv = (static_cast<float>(hadc1->Instance->JDR1) - 2044.5f) * 1.03f;
        this->iv = iv / ADC_REVOLUTION * V_REF / OP_AMP_GAIN / R_SENSE;
        this->iw = -(this->iu + this->iv);
    };

private:
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
};
