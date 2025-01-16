/**
 * @brief       PID基础控制库
 * @details
 * @author      Haoqi Liu
 * @date        2024-7-10
 * @version     V3.1.1
 * @note
 * @warning
 * @par         历史版本
 *              V1.0.0创建于2024-1-25
 *              V2.0.0创建于2024-2-12,删除树结构,改为由用户自行配置PID串并级关系
 *              V2.1.0创建于2024-2-17,PID内嵌积分限幅
 *              V3.0.0创建于2024-4-12,新增增量式PID算法,增加PID输出限幅,优化PID结构体初始化流程
 *              V3.1.0创建于2024-7-3,增加PID_Init_()函数,提供多一种初始化方式
 *              V3.1.1创建于2024-7-10,修复位置式PID算法微分项错误
 * */

#ifndef PID_H
#define PID_H

#include "main.h"

typedef enum {
    PID_TYPE_POSITION,       //位置式PID
    PID_TYPE_DELTA,          //增量式PID
} PIDType_EnumTypeDef;

typedef struct {
    PIDType_EnumTypeDef PID_Type;   //PID种类,位置式或增量式

    /*设置参数*/
    float Target;                   //目标值
    float Kp, Ki, Kd;               //比例、积分、微分系数

    /*限幅参数*/
    float Limit_SumError;           //积分限幅,仅位置式有效
    float Limit_Output;             //积分限幅
} PID_InitTypeDef;

/**
 * @brief PID结构体
 * */
typedef struct {
    PIDType_EnumTypeDef PID_Type;   //PID种类,位置式或增量式

    /*设置参数*/
    float Target;                   //目标值
    float Kp, Ki, Kd;               //比例、积分、微分系数

    /*限幅参数*/
    float Limit_SumError;           //积分限幅,仅位置式有效
    float Limit_Output;             //积分限幅

    /*中间(运行时)变量*/
    float Error;                    //上一次的偏差值
    float PreError;                 //上上一次的偏差值,仅增量式PID使用
    float SumError;                 //累计偏差值,仅位置式PID使用
    float Output;                   //PID输出值
} PID_TypeDef;

/**
 * @brief PID结构体初始化函数
 * @param PID PID结构体指针
 * @param Target PID目标值
 * @param Kp,Ki,Kd PID三个参数
 * */
extern void PID_Init(PID_TypeDef *PID, PIDType_EnumTypeDef PID_Type,
                     float Target, float Kp, float Ki, float Kd);

/**
 * @brief PID结构体初始化函数
 * @param PID PID结构体指针
 * @param InitStructure PID初始化结构体指针
 */
extern void PID_Init_(PID_TypeDef *PID, PID_InitTypeDef *InitStructure);

/**
 * @brief 设置PID限幅
 * @param Limit_SumError PID积分限幅
 * @param Limit_Output 输出限幅
 * @note ${限幅值}无论正负,均将目标值限制在+-${限幅值}中,若${限幅值}为0,则不进行限幅
 * */
extern void PID_SetLimit(PID_TypeDef *PID, float Limit_SumError, float Limit_Output);

/**
 * @brief 设置PID目标值
 * @param PID PID结构体指针
 * @param Target 新的PID目标值
 * */
extern void PID_SetTarget(PID_TypeDef *PID, float Target);

/**
 * @brief PID计算
 * @param PID PID结构体指针
 * @param Input PID观测值
 * @return PID计算结果
 * */
extern float PID_IRQ(PID_TypeDef *PID, float Input);

#endif //PID_H
