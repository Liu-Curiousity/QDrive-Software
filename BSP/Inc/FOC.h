/**
 * @brief   FOC驱动库
 * @details 该库需要提供以下定义即对应接口:
 *          Encoder_Driver_HandleTypeDef 编码器句柄
 *          BLDC_Driver_HandleTypeDef    BLDC句柄
 *          Encoder_Init()     初始化Encoder句柄,具体接口因方案而异
 *          Encoder_Start()    启动编码器
 *          Encoder_Stop()     关闭编码器
 *          Encoder_ReadAngle()读取编码器角度,弧度制
 *          BLDC_Init()     初始化BLDC句柄,具体接口因方案而异
 *          BLDC_Start()    启动BLDC驱动
 *          BLDC_Stop()     关闭BLDC驱动
 *          BLDC_SetDuty()  设置BLDC三相占空比,归一化
 * @author  LiuHaoqi
 * @date    2024-7-10
 * @version V2.0.0
 * @note    此库为中间层库,与硬件完全解耦
 * @warning 无
 * @par     历史版本:
 * 		    V1.0.0创建于2024-7-3
 *		    v2.0.0修改于2024-7-10,添加d轴电流PID控制
 * */


#ifndef FOC_H
#define FOC_H

#include "BLDC_Driver_FD6288.h"
#include "ENCODER_DRIVER_AS5047P.h"
#include "PID.h"

typedef enum {
    FOC_CurrentCtrl = 0,
    FOC_SpeedCtrl = 1,
    FOC_PositionCtrl = 2,
} FOC_CtrlType_EnumTypeDef;

typedef struct {
    BLDC_Driver_InitTypeDef Driver_InitStructure;       //驱动器句柄
    Encoder_Driver_InitTypeDef Encoder_InitStructure;   //编码器句柄
    uint8_t PolePairs;                                  //极对数
    uint16_t CtrlFrequency;                             //控制频率(速度环、位置环),单位Hz

    PID_InitTypeDef PID_CurrentQ_InitStructure;         //Q轴电流PID初始化结构体
    PID_InitTypeDef PID_CurrentD_InitStructure;         //D轴电流PID初始化结构体
    PID_InitTypeDef PID_Speed_InitStructure;            //速度PID初始化结构体
    PID_InitTypeDef PID_Position_InitStructure;         //位置PID初始化结构体

    float CurrentFilter;                                //电流低通滤波器系数
    float SpeedFilter;                                  //速度低通滤波器系数
} FOC_InitTypeDef;

/**
 * @brief FOC句柄结构体
 * */
typedef struct {
    //初始化配置参数
    FOC_InitTypeDef Init;                   //初始化参数
    BLDC_Driver_HandleTypeDef Driver;       //驱动器句柄
    Encoder_Driver_HandleTypeDef Encoder;   //编码器句柄

    PID_TypeDef PID_CurrentQ;               //Q轴电流PID
    PID_TypeDef PID_CurrentD;               //D轴电流PID
    PID_TypeDef PID_Speed;                  //速度PID
    PID_TypeDef PID_Position;               //位置PID

    //运行时参数
    FOC_CtrlType_EnumTypeDef CtrlType;      //当前控制类型
    float Angle;                            //当前电机角度,单位rad
    float PreviousAngle;                    //上一次电机角度(速度环、位置环更新中),单位rad
    float ElectricalAngle;                  //当前电机电角度,单位rad
    float Speed;                            //电机转速,单位rpm
    float Iq;                               //切向电流
    float Id;                               //法向电流
} FOC_HandleTypeDef;

/**
 * @brief FOC初始化函数
 * @param FOC FOC句柄指针
 * @param InitStructure 初始化结构体指针
 * @retval 0:初始化成功
 * */
extern int FOC_Init(FOC_HandleTypeDef *FOC, FOC_InitTypeDef *InitStructure);

/**
 * @brief FOC启动函数
 * @param FOC FOC句柄指针
 * */
extern void FOC_Start(FOC_HandleTypeDef *FOC);

/**
 * @brief FOC停止函数
 * @param FOC FOC句柄指针
 * */
extern void FOC_Stop(FOC_HandleTypeDef *FOC);

/**
 * @brief FOC控制设置函数
 * @param FOC FOC句柄指针
 * @param CtrlType 控制类型
 * @param value 控制值
 * */
extern void FOC_Ctrl(FOC_HandleTypeDef *FOC, FOC_CtrlType_EnumTypeDef CtrlType, float value);

/**
 * @brief FOC控制(速度环、位置环)中断服务函数
 * @param FOC FOC句柄指针
 * */
extern void FOC_Ctrl_ISR(FOC_HandleTypeDef *FOC);

/**
 * @brief FOC电流闭环控制中断服务函数
 * @param FOC FOC句柄指针
 * @param Iu U相电流
 * @param Iv V相电流
 * */
extern void FOC_CurrentLoopCtrl_ISR(FOC_HandleTypeDef *FOC, float Iu, float Iv);

#endif //FOC_H
