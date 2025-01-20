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

#include "PID.h"

void PID_Init(PID_TypeDef *PID, PIDType_EnumTypeDef PID_Type,
              float Target, float Kp, float Ki, float Kd) {
    PID->PID_Type = PID_Type;

    PID->Target = Target;
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;

    PID->Error = 0;
    PID->PreError = 0;
    PID->SumError = 0;
    PID->Limit_SumError = 0;
    PID->Limit_Output = 0;
    PID->Output = 0;
}

void PID_Init_(PID_TypeDef *PID, PID_InitTypeDef *InitStructure) {
    PID->PID_Type = InitStructure->PID_Type;

    PID->Target = InitStructure->Target;
    PID->Kp = InitStructure->Kp;
    PID->Ki = InitStructure->Ki;
    PID->Kd = InitStructure->Kd;
    PID->Limit_SumError = InitStructure->Limit_SumError;
    PID->Limit_Output = InitStructure->Limit_Output;

    PID->Error = 0;
    PID->PreError = 0;
    PID->SumError = 0;
    PID->Output = 0;
}

void PID_SetLimit(PID_TypeDef *PID, float Limit_SumError, float Limit_Output) {
    PID->Limit_SumError = (Limit_SumError > 0 ? Limit_SumError : -Limit_SumError);  //转换成正数,方便后续限幅
    PID->Limit_Output = (Limit_Output > 0 ? Limit_Output : -Limit_Output);          //转换成正数,方便后续限幅
}

void PID_SetTarget(PID_TypeDef *PID, float Target) {
    PID->Target = Target;
}

float PID_IRQ(PID_TypeDef *PID, float Input) {
    float error = PID->Target - Input;
    if (PID->PID_Type == PID_TYPE_POSITION) {
        /***位置式PID公式:u=Kpe(t)+Ki*e(t)的积分+Kd[e(t)-e(t-1)]***/
        PID->SumError += error;
        /*积分限幅*/
        if (PID->Limit_SumError) {
            if (PID->SumError >= PID->Limit_SumError) PID->SumError = PID->Limit_SumError;
            if (PID->SumError <= -PID->Limit_SumError) PID->SumError = -PID->Limit_SumError;
        }
        PID->Output = PID->Kp * error +
                      PID->Ki * PID->SumError +
                      PID->Kd * (error - PID->Error);
        PID->Error = error;
    } else if (PID->PID_Type == PID_TYPE_DELTA) {
        /***增量式PID公式:du=Kp[e(t)-e(t-1)]+Kie(t)+Kd[e(t)-2e(t-1)+e(t-2)]***/
        PID->Output += PID->Kp * (error - PID->Error) +
                       PID->Ki * (error) +
                       PID->Kd * (error - 2 * PID->Error + PID->PreError);
        PID->PreError = PID->Error;
        PID->Error = error;
    }
    /*输出限幅*/
    if (PID->Limit_Output) {
        if (PID->Output >= PID->Limit_Output) PID->Output = PID->Limit_Output;
        if (PID->Output <= -PID->Limit_Output) PID->Output = -PID->Limit_Output;
    }
    return PID->Output;
}

