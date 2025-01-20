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
		    V1.0.0创建于2024-7-3
		    v2.0.0修改于2024-7-10,添加d轴电流PID控制
 * */


#include "FOC.h"
#include "math.h"
#include "main.h"

#ifndef M_SQRT3_F
#define M_SQRT3_F 1.73205080756887719000f
#endif

#ifndef M_1_SQRT3_F
#define M_1_SQRT3_F 0.57735026918962576451f
#endif

/**
 * @brief FOC电流变换
 * @param FOC FOC句柄指针
 * @param Iu U相电流
 * @param Iv V相电流
 * */
__attribute__((section(".ccmram_func"))) inline
static void FOC_UpdateCurrent(FOC_HandleTypeDef *FOC, float Iu, float Iv) {
    /**1.检查输入参数**/
    assert_param(Iq != NULL);
    assert_param(Id != NULL);

    /**2.克拉克变换**/
    float Ia = Iu;
    float Ib = (Iu + 2 * Iv) * M_1_SQRT3_F;

    /**3.帕克变换**/
    float cos_angle = cosf(FOC->ElectricalAngle);
    float sin_angle = sinf(FOC->ElectricalAngle);
    FOC->Iq = (Ib * cos_angle - Ia * sin_angle) * (1 - FOC->Init.CurrentFilter) + FOC->Iq * FOC->Init.CurrentFilter;
    FOC->Id = (Ib * sin_angle + Ia * cos_angle) * (1 - FOC->Init.CurrentFilter) + FOC->Id * FOC->Init.CurrentFilter;
}

/**
 * @brief FOC控制函数
 * @param FOC FOC句柄指针
 * @param Uq 切向力矩,必须在0~1之间!
 * @param Ud 法向力矩,必须在0~1之间!
 * */
__attribute__((section(".ccmram_func"))) inline
static void FOC_SetPhaseVoltage(FOC_HandleTypeDef *FOC, float Uq, float Ud) {
    /**1.检查输入参数**/
    assert_param(Driver != NULL);
    assert_param(Uq <= 1 && Uq >= -1);
    assert_param(Ud <= 1 && Ud >= -1);

    // TODO：临时方案有待改进
    // Uu,Uv,Uw不能设置到最大值1,为了防止电流采样时候MOS对电机有驱动,影响采样
    // 表现为某一相Ux=0时候(堵转测试),电流采样值偶尔出现尖峰,电机异常抽搐
    // *0.99f为临时解决方案,缺点是牺牲功率密度
    Uq*=0.99f;
    Ud*=0.99f;

    /**2.帕克逆变换**/
    float cos_angle = cosf(FOC->ElectricalAngle);
    float sin_angle = sinf(FOC->ElectricalAngle);
    float Ualpha = (-Uq * sin_angle + Ud * cos_angle) / 2; // 除以2将Ualpha范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]
    float Ubeta = (Uq * cos_angle + Ud * sin_angle) / 2;   // 除以2将Ubeta范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]

    /**3.克拉克逆变换**/
    volatile float Uu = Ualpha + 0.5f; //加0.5使得Uu均值为0.5,在[0,1]之间变化
    volatile float Uv = -Ualpha / 2 + Ubeta * M_SQRT3_F / 2 + 0.5f;

    // 原公式:
    // float Uw = -Ualpha / 2 - Ubeta * M_SQRT3_F / 2 + 0.5f;
    // 使用Uu,Uv计算得来,减少运算量(其实运算量大头在sin和cos):
    volatile float Uw = 1.5f - Uu - Uv;

    /**4.设置驱动器占空比**/
    BLDC_SetDuty(&FOC->Driver, Uu, Uv, Uw);
}

int FOC_Init(FOC_HandleTypeDef *FOC, FOC_InitTypeDef *InitStructure) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);
    assert_param(InitStructure != NULL);
    assert_param(InitStructure->Driver != NULL);

    /**2.初始化FOC句柄**/
    FOC->Init = *InitStructure;

    /**3.初始化PID结构体**/
    PID_Init_(&FOC->PID_CurrentQ, &FOC->Init.PID_CurrentQ_InitStructure);
    PID_Init_(&FOC->PID_CurrentD, &FOC->Init.PID_CurrentD_InitStructure);
    PID_Init_(&FOC->PID_Speed, &FOC->Init.PID_Speed_InitStructure);
    PID_Init_(&FOC->PID_Position, &FOC->Init.PID_Position_InitStructure);
    // PID_SetLimit(&FOC->PID_CurrentQ, 0, 1); //电流环输出限幅[-1,1],对应FOC_SetPhaseVoltage()的归一化输入

    /**4.初始化BLDC驱动**/
    BLDC_Init(&FOC->Driver, &FOC->Init.Driver_InitStructure);

    /**5.初始化编码器**/
    Encoder_Init(&FOC->Encoder, &FOC->Init.Encoder_InitStructure);

    /**6.初始化FOC参数**/
    FOC->CtrlType = FOC_CurrentCtrl;
    FOC->Angle = 0;
    FOC->PreviousAngle = 0;
    FOC->ElectricalAngle = 0;
    FOC->Speed = 0;
    FOC->Iq = 0;
    FOC->Id = 0;
    return 0;
}

void FOC_Start(FOC_HandleTypeDef *FOC) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);

    /**2.启动BLDC驱动**/
    BLDC_Start(&FOC->Driver);

    /**3.启动编码器**/
    Encoder_Start(&FOC->Encoder);
}

void FOC_Stop(FOC_HandleTypeDef *FOC) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);

    /**2.停止BLDC驱动**/
    BLDC_Stop(&FOC->Driver);

    /**3.停止编码器**/
    Encoder_Stop(&FOC->Encoder);
}

void FOC_Ctrl(FOC_HandleTypeDef *FOC, FOC_CtrlType_EnumTypeDef CtrlType, float value) {
    FOC->CtrlType = CtrlType;
    switch (CtrlType) {
        case FOC_PositionCtrl:
            PID_SetTarget(&FOC->PID_Position, value);
            break;
        case FOC_SpeedCtrl:
            PID_SetTarget(&FOC->PID_Speed, value);
            break;
        case FOC_CurrentCtrl:
            PID_SetTarget(&FOC->PID_CurrentQ, value);
            break;
    }
}

//__attribute__((section(".ccmram_func")))
void FOC_Ctrl_ISR(FOC_HandleTypeDef *FOC) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);

    /**2.计算转速**/
    static float temp = 0;
    temp = FOC->PreviousAngle - FOC->Angle;
    if (FOC->Angle - FOC->PreviousAngle > M_PI) temp += M_PI * 2;
    else if (FOC->Angle - FOC->PreviousAngle < -M_PI) temp -= M_PI * 2;
    temp *= 60 * FOC->Init.CtrlFrequency / (M_PI * 2);
    FOC->Speed = temp * (1 - FOC->Init.SpeedFilter) + FOC->Speed * FOC->Init.SpeedFilter;
    FOC->PreviousAngle = FOC->Angle;

    /**3.速度闭环控制**/
    switch (FOC->CtrlType) {
        case FOC_PositionCtrl:
            //使电机始终沿差值小于pi的方向转动
            if (FOC->Angle - FOC->PID_Position.Target > M_PI)
                PID_SetTarget(&FOC->PID_Speed, PID_IRQ(&FOC->PID_Position, FOC->Angle - 2 * M_PI));
            else if (FOC->Angle - FOC->PID_Position.Target < -M_PI)
                PID_SetTarget(&FOC->PID_Speed, PID_IRQ(&FOC->PID_Position, FOC->Angle + 2 * M_PI));
            else
                PID_SetTarget(&FOC->PID_Speed, PID_IRQ(&FOC->PID_Position, FOC->Angle));
        case FOC_SpeedCtrl:
            PID_SetTarget(&FOC->PID_CurrentQ, PID_IRQ(&FOC->PID_Speed, FOC->Speed));
            break;
        case FOC_CurrentCtrl:
            break;
    }
}

/*CCMRAM加速运行*/
__attribute__((section(".ccmram_func"))) inline
void FOC_CurrentLoopCtrl_ISR(FOC_HandleTypeDef *FOC, float Iu, float Iv) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);

    /**2.电流变换**/
    FOC_UpdateCurrent(FOC, Iu, Iv);

    /**3.读取编码器角度**/
    FOC->Angle = Encoder_ReadAngle(&FOC->Encoder);
    FOC->ElectricalAngle = FOC->Angle * FOC->Init.PolePairs;

    /**4.电流闭环控制**/
    FOC_SetPhaseVoltage(FOC, PID_IRQ(&FOC->PID_CurrentQ, FOC->Iq), PID_IRQ(&FOC->PID_CurrentD, FOC->Id));
}
