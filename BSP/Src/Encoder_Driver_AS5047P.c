/**
 * @brief   编码器驱动库_AS5047P版
 * @details 编码器驱动库封装并提供以下接口:
 *          Encoder_Init()     初始化Encoder句柄,具体接口因方案而异
 *          Encoder_Start()    启动编码器
 *          Encoder_Stop()     关闭编码器
 * @author  LiuHaoqi
 * @date    2024-7-3
 * @version V1.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-7-3
 * */


#include <math.h>
#include "Encoder_Driver_AS5047P.h"


int Encoder_Init(Encoder_Driver_HandleTypeDef *Encoder, Encoder_Driver_InitTypeDef *Driver_InitStructure) {
    /**1.检查输入参数**/
    assert_param(Encoder != NULL);
    assert_param(Driver_InitStructure != NULL);
    assert_param(Driver_InitStructure->hspi != NULL);
    assert_param(Driver_InitStructure->CS_GPIO_Port_ != NULL);
    assert_param(Driver_InitStructure->CS_GPIO_Pin_ != NULL);

    /**2.初始化编码器句柄**/
    AS5047P_Init(&Encoder->AS5047P, Driver_InitStructure->hspi, Driver_InitStructure->CS_GPIO_Port_,
                 Driver_InitStructure->CS_GPIO_Pin_);
    //设置零位校准值
    Encoder->ZeroPosition_Calibration = Driver_InitStructure->ZeroPosition_Calibration;

    return 0;
}

int Encoder_Start(Encoder_Driver_HandleTypeDef *Encoder) {
    return 0;
}

int Encoder_Stop(Encoder_Driver_HandleTypeDef *Driver) {
    return 0;
}

__attribute__((section(".ccmram_func"))) inline
float Encoder_ReadAngle(Encoder_Driver_HandleTypeDef *Encoder) {
    static uint16_t rxData;
    AS5047P_ReadAngleContinuously(&Encoder->AS5047P, &rxData);
    rxData = (rxData + Encoder->ZeroPosition_Calibration) % 0x3fff;             //矫正零点
    return 2 * M_PI - (float) (rxData) / 0x3fff * 2 * M_PI;                       //转化为弧度制
}