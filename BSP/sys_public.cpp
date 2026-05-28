/**
 * @name        sys_public.h
 * @brief 		用于支持简单平台移植的中间层
 * @detail
 * @author 	    Haoqi Liu
 * @date        26-5-29
 * @version 	V3.3.0
 * @note 		用户需要根据提示定义相关宏函数,其中,MAX_DELAY应与delay()匹配
 * @warning
 * @par 		历史版本
                V1.0.0创建于24-11-24
                V2.0.0创建于24-11-27, 修改打印日志宏函数命名,添加更多日志打印方式
                V3.0.0创建于25-4-17, 重写new和delete函数
                V3.1.0创建于25-9-11, 添加另外两个重载delete函数
                V3.2.0创建于26-3-29, 添加delay_us()函数
                V3.3.0创建于26-5-29, 添加硬件版本识别
 * */

#include <cmath>
#include "FOC_config.h"
#include "adc.h"
#include "main.h"

void version_detect() {
    if (HAL_GPIO_ReadPin(VersionDetect_D_GPIO_Port, VersionDetect_D_Pin) == GPIO_PIN_SET) {
        FOC_HARDWARE_VERSION = FOC_HARDWARE_VERSIONS[1];
    } else {
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); //校准ADC

        ADC_Enable(&hadc2);
        LL_ADC_REG_StartConversion(ADC1);
        while (LL_ADC_REG_IsConversionOngoing(ADC2)) {}
        if (std::abs(LL_ADC_REG_ReadConversionData12(ADC2) - 54) <= 50) {
            FOC_HARDWARE_VERSION = FOC_HARDWARE_VERSIONS[2];
        } else {
            FOC_HARDWARE_VERSION = FOC_HARDWARE_VERSIONS[0];
        }
    }
}
