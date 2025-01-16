/**
 * @brief   编码器驱动库_AS5047P版
 * @details 编码器驱动库封装并提供以下接口:
 *          Encoder_Init()     初始化Encoder句柄,具体接口因方案而异
 *          Encoder_Start()    启动编码器
 *          Encoder_Stop()     关闭编码器
 *          Encoder_ReadAngle()读取编码器角度,弧度制
 * @author  LiuHaoqi
 * @date    2024-7-3
 * @version V1.0.0
 * @note
 * @warning
 * @par     历史版本:
		    V1.0.0创建于2024-7-3
 * */


#ifndef ENCODER_DRIVER_AS5047P_H
#define ENCODER_DRIVER_AS5047P_H

#include "AS5047P.h"

typedef struct {
    GPIO_TypeDef *CS_GPIO_Port_;    //CS引脚端口号
    uint16_t CS_GPIO_Pin_;          //CS引脚号
    SPI_HandleTypeDef *hspi;        //SPI句柄
    int16_t ZeroPosition_Calibration;   //零位校准值
} Encoder_Driver_InitTypeDef;

typedef struct {
    AS5047P_TypeDef AS5047P;
    int16_t ZeroPosition_Calibration;   //零位校准值
} Encoder_Driver_HandleTypeDef;

extern int Encoder_Init(Encoder_Driver_HandleTypeDef *Encoder, Encoder_Driver_InitTypeDef *Driver_InitStructure);

extern int Encoder_Start(Encoder_Driver_HandleTypeDef *Encoder);

extern int Encoder_Stop(Encoder_Driver_HandleTypeDef *Encoder);

extern float Encoder_ReadAngle(Encoder_Driver_HandleTypeDef *Encoder);

#endif //ENCODER_DRIVER_AS5047P_H

