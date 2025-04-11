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


#include <numbers>
#include "numbers"
#include "FOC.h"
#include "main.h"

using namespace std;

/**
 * @brief FOC电流变换
 * @param iu U相电流
 * @param iv V相电流
 * */
__attribute__((section(".ccmram_func")))
void FOC::UpdateCurrent(const float iu, const float iv) {
    /**1.保存电流值**/
    Iu = iu;
    Iv = iv;
    Iw = -Iu - Iv;

    /**2.克拉克变换**/
    Ia = Iu;
    Ib = (Iu + 2 * Iv) * std::numbers::inv_sqrt3_v<float>;

    /**3.帕克变换**/
    const float cos_angle = cosf(ElectricalAngle);
    const float sin_angle = sinf(ElectricalAngle);
    Iq = CurrentQFilter(Ib * cos_angle - Ia * sin_angle);
    Id = CurrentDFilter(Ib * sin_angle + Ia * cos_angle);
}

/**
 * @brief FOC控制函数
 * @param uq 切向力矩,必须在0~1之间!
 * @param ud 法向力矩,必须在0~1之间!
 * */
__attribute__((section(".ccmram_func")))
void FOC::SetPhaseVoltage(float uq, float ud) {
    /**1.检查输入参数**/
    assert_param(Uq <= 1 && Uq >= -1);
    assert_param(Ud <= 1 && Ud >= -1);

    // TODO：临时方案有待改进
    // Uu,Uv,Uw不能设置到最大值1,为了防止电流采样时候MOS对电机有驱动,影响采样
    // 表现为某一相Ux=0时候(堵转测试),电流采样值偶尔出现尖峰,电机异常抽搐
    // *0.99f为临时解决方案,缺点是牺牲功率密度
    uq *= 0.99f;
    ud *= 0.99f;

    /*2.保存电压值*/
    Uq = uq;
    Ud = ud;

    /**2.帕克逆变换**/
    const float cos_angle = cosf(ElectricalAngle);
    const float sin_angle = sinf(ElectricalAngle);
    Ua = (-Uq * sin_angle + Ud * cos_angle) / 2; // 除以2将Ua范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]
    Ub = (Uq * cos_angle + Ud * sin_angle) / 2;  // 除以2将Ub范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]

    /**3.克拉克逆变换**/
    Uu = Ua + 0.5f; //加0.5使得Uu均值为0.5,在[0,1]之间变化
    Uv = -Ua / 2 + Ub * std::numbers::sqrt3_v<float> / 2 + 0.5f;

    // 原公式:
    // float Uw = -Ua / 2 - Ub * M_SQRT3_F / 2 + 0.5f;
    // 使用Uu,Uv计算得来,减少运算量(其实运算量大头在sin和cos):
    Uw = 1.5f - Uu - Uv;

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
    if (Angle - PreviousAngle > numbers::pi_v<float>) temp += numbers::pi_v<float> * 2;
    else if (Angle - PreviousAngle < -numbers::pi_v<float>) temp -= numbers::pi_v<float> * 2;
    Speed = SpeedFilter(temp * 60 * CtrlFrequency / (numbers::pi_v<float> * 2));
    PreviousAngle = Angle;

    /**2.速度闭环控制**/
    switch (ctrl_type) {
        case CtrlType::PositionCtrl:
            //使电机始终沿差值小于pi的方向转动
            if (Angle - PID_Position.target > numbers::pi_v<float>)
                PID_Speed.SetTarget(PID_Position.clac(Angle - 2 * numbers::pi_v<float>));
            else if (Angle - PID_Position.target < -numbers::pi_v<float>)
                PID_Speed.SetTarget(PID_Position.clac(Angle + 2 * numbers::pi_v<float>));
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
void FOC::CurrentLoopCtrl_ISR(float iu, float iv) {
    /**1.电流变换**/
    UpdateCurrent(iu, iv);

    /**2.读取编码器角度**/
    Angle = bldc_encoder.get_angle();
    ElectricalAngle = Angle * PolePairs;

    /**3.电流闭环控制**/
    const float uq = PID_CurrentQ.clac(Iq);
    const float ud = PID_CurrentD.clac(Id);
    SetPhaseVoltage(uq, ud);
}
