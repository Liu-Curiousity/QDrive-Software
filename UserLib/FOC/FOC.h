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
     * @param CurrentFilter 电流采样滤波器系数
     * @param SpeedFilter 速度滤波器系数
     * @param driver BLDC驱动
     * @param encoder 编码器驱动
     * @param PID_CurrentQ Q轴电流PID
     * @param PID_CurrentD D轴电流PID
     * @param PID_Speed 速度PID
     * @param PID_Position 位置PID
     */
    FOC(uint8_t PolePairs, uint16_t CtrlFrequency, float CurrentFilter, float SpeedFilter,
        const BLDC_Driver& driver, const Encoder& encoder,
        const PID& PID_CurrentQ, const PID& PID_CurrentD, const PID& PID_Speed, const PID& PID_Position):
        bldc_driver(driver), bldc_encoder(encoder), PolePairs(PolePairs), CtrlFrequency(CtrlFrequency),
        CurrentFilter(CurrentFilter), SpeedFilter(SpeedFilter),
        PID_CurrentQ(PID_CurrentQ), PID_CurrentD(PID_CurrentD), PID_Speed(PID_Speed), PID_Position(PID_Position) {}

    [[nodiscard]] float speed() const { return Speed; }
    [[nodiscard]] float angle() const { return Angle; }

    void start() {
        bldc_driver.start();  //1.启动BLDC驱动
        bldc_encoder.start(); //2.启动编码器
    }

    void stop() {
        bldc_driver.stop();  //1.关闭BLDC驱动
        bldc_encoder.stop(); //2.关闭编码器
    }

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
     * @param Iu U相电流
     * @param Iv V相电流
     * */
    void CurrentLoopCtrl_ISR(float Iu, float Iv);

    BLDC_Driver bldc_driver; //驱动器
    Encoder bldc_encoder;    //编码器

    //初始化配置项
    const uint8_t PolePairs;      //极对数
    const uint16_t CtrlFrequency; //控制频率(速度环、位置环),单位Hz
    const float CurrentFilter{0}; //电流低通滤波器系数,0~1,0为不滤波
    const float SpeedFilter{0};   //速度低通滤波器系数,0~1,0为不滤波

private:
    void UpdateCurrent(float Iu, float Iv);
    void SetPhaseVoltage(float Uq, float Ud) const;

    CtrlType ctrl_type{CtrlType::CurrentCtrl}; //当前控制类型

    //PID类
    PID PID_CurrentQ; //Q轴电流PID
    PID PID_CurrentD; //D轴电流PID
    PID PID_Speed;    //速度PID
    PID PID_Position; //位置PID

    //运行时参数
    float Angle{0};           //当前电机角度,单位rad
    float PreviousAngle{0};   //上一次电机角度(速度环、位置环更新中),单位rad
    float ElectricalAngle{0}; //当前电机电角度,单位rad
    float Speed{0};           //电机转速,单位rpm
    float Ia{0};              //切向电流
    float Ib{0};              //法向电流
    float Iq{0};              //切向电流
    float Id{0};              //法向电流
};

#endif //FOC_H
