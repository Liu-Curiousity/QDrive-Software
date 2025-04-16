/**
 * @brief   FOC驱动库
 * @details
 * @author  Haoqi Liu
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

#include "BLDC_Driver.h"
#include "Encoder.h"
#include "LowPassFilter.h"
#include "PID.h"
#include "cstdint"

/**
 * @brief FOC句柄结构体
 * */
class FOC {
public:
    enum class CtrlType {
        CurrentCtrl = 0,
        SpeedCtrl = 1,
        PositionCtrl = 2,
    };

    /**
     * @brief 初始化
     * @param PolePairs 极对数
     * @param CtrlFrequency 控制频率,用于计算转速
     * @param CurrentCtrlFrequency 电流控制频率,单位Hz
     * @param CurrentQFilter Q轴电流采样滤波器系数
     * @param CurrentDFilter D轴电流采样滤波器系数
     * @param SpeedFilter 速度滤波器系数
     * @param driver BLDC驱动
     * @param encoder 编码器驱动
     * @param zero_electric_angle 电机零点电角度,单位rad
     * @param PID_CurrentQ Q轴电流PID
     * @param PID_CurrentD D轴电流PID
     * @param PID_Speed 速度PID
     * @param PID_Position 位置PID
     */
    FOC(uint8_t PolePairs, uint16_t CtrlFrequency, uint16_t CurrentCtrlFrequency,
        LowPassFilter& CurrentQFilter, LowPassFilter& CurrentDFilter, LowPassFilter& SpeedFilter,
        BLDC_Driver& driver, Encoder& encoder, float zero_electric_angle,
        const PID& PID_CurrentQ, const PID& PID_CurrentD, const PID& PID_Speed, const PID& PID_Position):
        bldc_driver(driver), bldc_encoder(encoder), zero_electric_angle(zero_electric_angle), PolePairs(PolePairs),
        CtrlFrequency(CtrlFrequency), CurrentCtrlFrequency(CurrentCtrlFrequency),
        CurrentQFilter(CurrentQFilter), CurrentDFilter(CurrentDFilter), SpeedFilter(SpeedFilter),
        PID_CurrentQ(PID_CurrentQ), PID_CurrentD(PID_CurrentD), PID_Speed(PID_Speed), PID_Position(PID_Position) {}

    [[nodiscard]] float speed() const { return Speed; }
    [[nodiscard]] float angle() const { return Angle; }

    void init() const;

    void enable() const;

    void disable() const;

    void calibration();
    /**
     * @brief FOC控制设置函数
     * @param ctrl_type 控制类型
     * @param value 控制值
     * */
    void Ctrl(CtrlType ctrl_type, float value);

    /**
     * @brief FOC控制(速度环、位置环)中断服务函数
     * */
    void Ctrl_ISR();

    /**
     * @brief FOC电流闭环控制中断服务函数
     * @param iu U相电流
     * @param iv V相电流
     * */
    void loopCtrl(float iu, float iv);


    //初始化配置项
    const uint8_t PolePairs;             //极对数
    const uint16_t CtrlFrequency;        //控制频率(速度环、位置环),单位Hz
    const uint16_t CurrentCtrlFrequency; //控制频率(电流环),单位Hz

private:
    CtrlType ctrl_type{CtrlType::CurrentCtrl}; //当前控制类型

    //PID类
    PID PID_CurrentQ; //Q轴电流PID
    PID PID_CurrentD; //D轴电流PID
    PID PID_Speed;    //速度PID
    PID PID_Position; //位置PID

    BLDC_Driver& bldc_driver;      //驱动器
    Encoder& bldc_encoder;         //编码器
    LowPassFilter& CurrentQFilter; //Q轴电流低通滤波器
    LowPassFilter& CurrentDFilter; //D轴电流低通滤波器
    LowPassFilter& SpeedFilter;    //速度低通滤波器

    //运行时参数
    float zero_electric_angle{0}; // 电机零点电角度,单位rad
    float Angle{0};               // 当前电机角度,单位rad
    float PreviousAngle{0};       // 上一次电机角度(速度环、位置环更新中),单位rad
    float ElectricalAngle{0};     // 当前电机电角度,单位rad
    float Speed{0};               // 电机转速,单位rpm

    float Uu{0}; //U相电压
    float Uv{0}; //V相电压
    float Uw{0}; //W相电压
    float Ua{0}; //A轴电压
    float Ub{0}; //B轴电压
    float Uq{0}; //切向电压
    float Ud{0}; //法向电压

    float Iu{0}; //U相电流
    float Iv{0}; //V相电流
    float Iw{0}; //W相电流
    float Ia{0}; //A轴电流
    float Ib{0}; //B轴电流
    float Iq{0}; //切向电流
    float Id{0}; //法向电流

    void alignAngle();
    void UpdateCurrent(float iu, float iv);
    void SetPhaseVoltage(float uq, float ud);
};

#endif //FOC_H
