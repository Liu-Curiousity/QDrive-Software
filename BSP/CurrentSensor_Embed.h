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

    void update(const float iu, const float iv) {
        this->iu = iu;
        this->iv = iv;
        this->iw = -(iu + iv);
    };

private:
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
};
