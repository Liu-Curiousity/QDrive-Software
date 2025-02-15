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

#ifndef M_SQRT3_F
#define M_SQRT3_F 1.73205080756887719000f
#endif

#ifndef M_1_SQRT3_F
#define M_1_SQRT3_F 0.57735026918962576451f
#endif

/**
 * @brief FOC电流变换
 * @param Iu U相电流
 * @param Iv V相电流
 * */
__attribute__((section(".ccmram_func")))
void FOC::UpdateCurrent(const float Iu, const float Iv) {
    /**1.检查输入参数**/
    assert_param(Iq != NULL);
    assert_param(Id != NULL);

    /**2.克拉克变换**/
    const float Ia = Iu;
    const float Ib = (Iu + 2 * Iv) * M_1_SQRT3_F;

    /**3.帕克变换**/
    const float cos_angle = cosf(ElectricalAngle);
    const float sin_angle = sinf(ElectricalAngle);
    Iq = (Ib * cos_angle - Ia * sin_angle) * (1 - CurrentFilter) + Iq * CurrentFilter;
    Id = (Ib * sin_angle + Ia * cos_angle) * (1 - CurrentFilter) + Id * CurrentFilter;
}

/**
 * @brief FOC控制函数
 * @param Uq 切向力矩,必须在0~1之间!
 * @param Ud 法向力矩,必须在0~1之间!
 * */
__attribute__((section(".ccmram_func")))
void FOC::SetPhaseVoltage(float Uq, float Ud) const {
    /**1.检查输入参数**/
    assert_param(Driver != NULL);
    assert_param(Uq <= 1 && Uq >= -1);
    assert_param(Ud <= 1 && Ud >= -1);

    // TODO：临时方案有待改进
    // Uu,Uv,Uw不能设置到最大值1,为了防止电流采样时候MOS对电机有驱动,影响采样
    // 表现为某一相Ux=0时候(堵转测试),电流采样值偶尔出现尖峰,电机异常抽搐
    // *0.99f为临时解决方案,缺点是牺牲功率密度
    Uq *= 0.99f;
    Ud *= 0.99f;

    /**2.帕克逆变换**/
    const float cos_angle = cosf(ElectricalAngle);
    const float sin_angle = sinf(ElectricalAngle);
    const float Ualpha = (-Uq * sin_angle + Ud * cos_angle) / 2; // 除以2将Ualpha范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]
    const float Ubeta = (Uq * cos_angle + Ud * sin_angle) / 2;   // 除以2将Ubeta范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]

    /**3.克拉克逆变换**/
    const float Uu = Ualpha + 0.5f; //加0.5使得Uu均值为0.5,在[0,1]之间变化
    const float Uv = -Ualpha / 2 + Ubeta * M_SQRT3_F / 2 + 0.5f;

    // 原公式:
    // float Uw = -Ualpha / 2 - Ubeta * M_SQRT3_F / 2 + 0.5f;
    // 使用Uu,Uv计算得来,减少运算量(其实运算量大头在sin和cos):
    const float Uw = 1.5f - Uu - Uv;

    /**4.设置驱动器占空比**/
    bldc_driver.set_duty(Uu, Uv, Uw);
}

void FOC::Ctrl(const CtrlType ctrl_type, const float value) {
    this->ctrl_type = ctrl_type;
    switch (ctrl_type) {
        case CtrlType::PositionCtrl:
            PID_Position.SetTarget(value);
            break;
        case CtrlType::SpeedCtrl:
            PID_Speed.SetTarget(value);
            break;
        case CtrlType::CurrentCtrl:
            PID_CurrentQ.SetTarget(value);
            break;
    }
}

__attribute__((section(".ccmram_func")))
void FOC::Ctrl_ISR() {
    /**1.计算转速**/
    static float temp = 0;
    temp = PreviousAngle - Angle;
    if (Angle - PreviousAngle > M_PI) temp += M_PI * 2;
    else if (Angle - PreviousAngle < -M_PI) temp -= M_PI * 2;
    temp *= 60 * CtrlFrequency / (M_PI * 2);
    Speed = temp * (1 - SpeedFilter) + Speed * SpeedFilter;
    PreviousAngle = Angle;

    /**2.速度闭环控制**/
    switch (ctrl_type) {
        case CtrlType::PositionCtrl:
            //使电机始终沿差值小于pi的方向转动
            if (Angle - PID_Position.target > M_PI)
                PID_Speed.SetTarget(PID_Position.clac(Angle - 2 * M_PI));
            else if (Angle - PID_Position.target < -M_PI)
                PID_Speed.SetTarget(PID_Position.clac(Angle + 2 * M_PI));
            else
                PID_Speed.SetTarget(PID_Position.clac(Angle));
        case CtrlType::SpeedCtrl:
            PID_CurrentQ.SetTarget(PID_Speed.clac(Speed));
            break;
        case CtrlType::CurrentCtrl:
            break;
    }
}

/*CCMRAM加速运行*/
__attribute__((section(".ccmram_func")))
void FOC::CurrentLoopCtrl_ISR(float Iu, float Iv) {
    /**1.检查输入参数**/
    assert_param(FOC != NULL);

    /**2.电流变换**/
    UpdateCurrent(Iu, Iv);

    /**3.读取编码器角度**/
    Angle = bldc_encoder.read_angle();
    ElectricalAngle = Angle * PolePairs;

    /**4.电流闭环控制**/
    SetPhaseVoltage(PID_CurrentQ.clac(Iq), PID_CurrentD.clac(Id));
}
