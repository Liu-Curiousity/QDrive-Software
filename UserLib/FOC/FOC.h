/**
* @brief   FOC驱动库
 * @details
 * @author  LiuHaoqi
 * @date    2024-7-10
 * @version V3.0.0
 * @note    此库为中间层库,与硬件完全解耦
 * @warning 无
 * @par     历史版本:
 *		    V1.0.0创建于2024-7-3
 *		    v2.0.0修改于2024-7-10,添加d轴电流PID控制
 *		    V3.0.0修改于2025-4-12,中间漏了好多版本
 *		    V4.0.0修改于2025-5-4,添加CurrentSensor类,后将续从current_sensor中获取电流
 *		    V4.1.0修改于2025-5-5,重命名PolePairs为pole_pairs,添加电流偏置校准和相电阻测量,并在电角度校准时自动调整硬拖电压
 *		    V4.1.1修改于2025-5-6,校准相电流偏置前等待30ms,修复测量相电阻时忘记应用电流偏置校准导致相电阻测量误差的问题
 *		    V4.1.2修改于2025-5-6,优化操作逻辑,开始校准前清除已校准标志
 *		    V4.1.3修改于2025-5-6,更改校准函数名
 *		    V5.0.0修改于2025-6-26,调整initialize,enable,start三层实现逻辑细节
 *		    V5.0.0调整SetPhaseVoltage()参数顺序
 *		    V5.1.0修改于2025-7-3,修复calibrate()函数致命问题,重新调整init,enable,start三层实现逻辑细节,为后续无感算法铺路,调整更新电压函数接口名称
 */


#ifndef FOC_H
#define FOC_H

#include "cstdint"
#include "BLDC_Driver.h"
#include "CurrentSensor.h"
#include "Encoder.h"
#include "LowPassFilter.h"
#include "Storage.h"
#include "PID.h"

/**
 * @brief FOC句柄结构体
 */
class FOC {
public:
    enum class CtrlType {
        CurrentCtrl = 0,
        SpeedCtrl = 1,
        AngleCtrl = 2,
        LowSpeedCtrl = 3,
    };

    /**
     * @brief 初始化
     * @param pole_pairs 极对数
     * @param CtrlFrequency 控制频率,用于计算转速
     * @param CurrentCtrlFrequency 电流控制频率,单位Hz
     * @param CurrentQFilter Q轴电流采样滤波器系数
     * @param CurrentDFilter D轴电流采样滤波器系数
     * @param SpeedFilter 速度滤波器系数
     * @param driver BLDC驱动
     * @param encoder 编码器驱动
     * @param storage 存储器
     * @param current_sensor 电流传感器
     * @param PID_CurrentQ Q轴电流PID
     * @param PID_CurrentD D轴电流PID
     * @param PID_Speed 速度PID
     * @param PID_Angle 角度PID
     */
    FOC(const uint8_t pole_pairs, const uint16_t CtrlFrequency, const uint16_t CurrentCtrlFrequency,
        LowPassFilter& CurrentQFilter, LowPassFilter& CurrentDFilter, LowPassFilter& SpeedFilter,
        BLDC_Driver& driver, Encoder& encoder, Storage& storage, CurrentSensor& current_sensor,
        const PID& PID_CurrentQ, const PID& PID_CurrentD, const PID& PID_Speed, const PID& PID_Angle) :
        pole_pairs(pole_pairs), CtrlFrequency(CtrlFrequency), CurrentCtrlFrequency(CurrentCtrlFrequency),
        PID_CurrentQ(PID_CurrentQ), PID_CurrentD(PID_CurrentD), PID_Speed(PID_Speed), PID_Angle(PID_Angle),
        storage(storage), bldc_driver(driver), bldc_encoder(encoder), current_sensor(current_sensor),
        CurrentQFilter(CurrentQFilter), CurrentDFilter(CurrentDFilter), SpeedFilter(SpeedFilter) {}

    [[nodiscard]] CtrlType getCtrlType() const { return ctrl_type; } // 获取控制模式
    [[nodiscard]] float getSpeed() const { return Speed; }           // 获取电机转速,单位rpm
    [[nodiscard]] float getAngle() const { return Angle; }           // 获取电机角度,单位rad
    [[nodiscard]] float getCurrent() const { return Iq; }            // 获取Q轴电流,单位A
    [[nodiscard]] float getVoltage() const { return Voltage; }       // 获取母线电压,单位V

    void init();
    void enable();
    void disable();
    void start();
    void stop();
    void calibrate();
    void anticogging_calibrate();

    /**
     * @brief FOC控制设置函数
     * @param ctrl_type 控制类型
     * @param value 控制值
     */
    void Ctrl(CtrlType ctrl_type, float value);

    /**
     * @brief FOC控制(速度环、角度环)中断服务函数
     */
    void Ctrl_ISR();

    /**
     * @brief FOC电流闭环控制中断服务函数
     */
    void loopCtrl();

    /**
     * @brief 更新母线电压,用于调整控制回路增益
     * @param voltage 母线电压,单位V
     */
    void updateVoltage(float voltage);

    /**
     * @brief 设置PID参数
     * @param pid_speed_kp 速度环比例系数,若为NAN则不更新
     * @param pid_speed_ki 速度环积分系数,若为NAN则不更新
     * @param pid_speed_kd 速度环微分系数,若为NAN则不更新
     * @param pid_angle_kp 角度环比例系数,若为NAN则不更新
     * @param pid_angle_ki 角度环积分系数,若为NAN则不更新
     * @param pid_angle_kd 角度环微分系数,若为NAN则不更新
     */
    void setPID(float pid_speed_kp, float pid_speed_ki, float pid_speed_kd,
                float pid_angle_kp, float pid_angle_ki, float pid_angle_kd);

    /**
     * @brief 设置速度和电流限制
     * @param speed_limit 速度限制,单位rpm
     * @param current_limit 电流限制,单位A
     */
    void setLimit(float speed_limit, float current_limit);

    /**
     * @brief 储存PID参数
     */
    void storagePID();

    // 初始化配置项
    const uint8_t pole_pairs;            // 极对数
    const uint16_t CtrlFrequency;        // 控制频率(速度环、角度环),单位Hz
    const uint16_t CurrentCtrlFrequency; // 控制频率(电流环),单位Hz

    uint8_t ID{0};                      // 电机ID
    bool initialized{false};            // 是否初始化
    bool enabled{false};                // 是否使能
    bool started{false};                // 是否启动
    bool calibrated{false};             // 是否校准过
    bool anticogging_enabled{false};    // 是否开启齿槽转矩补偿
    bool anticogging_calibrated{false}; // 是否校准过齿槽转矩

private:
    friend void foc_config_list();
    friend void foc_store();
    friend void foc_restore();

    enum StorageStatus:uint8_t {
        STORAGE_BASE_CALIBRATE_OK = 0x80,
        STORAGE_ANTICOGGING_CALIBRATE_OK = 0x20,
        STORAGE_PID_PARAMETER_OK = 0x08,
        STORAGE_LIMIT_OK = 0x02,
        STORAGE_ALL_OK = STORAGE_BASE_CALIBRATE_OK |
                         STORAGE_ANTICOGGING_CALIBRATE_OK |
                         STORAGE_PID_PARAMETER_OK |
                         STORAGE_LIMIT_OK,
        STORAGE_ERROR = 0x55,
    };

    CtrlType ctrl_type{CtrlType::CurrentCtrl}; //当前控制类型

    //PID类
    PID PID_CurrentQ;      //Q轴电流PID
    PID PID_CurrentD;      //D轴电流PID
    PID PID_Speed;         //速度PID
    PID PID_Angle;         //角度PID
    float target_iq{0.0f}; //目标Q轴电流

    Storage& storage;              //存储器
    BLDC_Driver& bldc_driver;      //驱动器
    Encoder& bldc_encoder;         //编码器
    CurrentSensor& current_sensor; //电流传感器
    LowPassFilter& CurrentQFilter; //Q轴电流低通滤波器
    LowPassFilter& CurrentDFilter; //D轴电流低通滤波器
    LowPassFilter& SpeedFilter;    //速度低通滤波器

    // 校准参数
    bool encoder_direction{true};            // true if the encoder is in the same direction as the motor(Uq)
    float phase_resistance{NAN};             // 相电阻,单位Ω
    float phase_inductance{NAN};             // 相电感,单位H
    float iu_offset{0};                      // U相电流偏置,单位A
    float iv_offset{0};                      // V相电流偏置,单位A
    float zero_electric_angle{0};            // 电机零点电角度,单位rad
    static constexpr uint16_t map_len{2000}; // 齿槽转矩校准点数
    float anticogging_map[map_len]{};        // 齿槽转矩补偿表
    bool anticogging_calibrating{false};     // 齿槽转矩是否正在校准

    // 运行时参数
    float Angle{0};           // 当前电机角度,单位rad
    float PreviousAngle{0};   // 上一次电机角度(速度环、角度环更新中),单位rad
    float ElectricalAngle{0}; // 当前电机电角度,单位rad
    float Speed{0};           // 电机转速,单位rpm
    // 极低速控制
    float low_speed_angle{0}; // 极低速控制角度,单位rad
    float low_speed{0};       // 单位rpm

    float Uu{0}; //U相电压
    float Uv{0}; //V相电压
    float Uw{0}; //W相电压
    float Ua{0}; //A轴电压
    float Ub{0}; //B轴电压
    float Uq{0}; //切向电压
    float Ud{0}; //法向电压

    float Iu{0}; //U相电流,单位A
    float Iv{0}; //V相电流,单位A
    float Iw{0}; //W相电流,单位A
    float Ia{0}; //A轴电流,单位A
    float Ib{0}; //B轴电流,单位A
    float Iq{0}; //Q轴电流,单位A
    float Id{0}; //D轴电流,单位A

    float Voltage{1}; //母线电压

    static float wrap(float value, float min, float max);
    void load_storage_calibration();
    void freeze_storage_calibration(StorageStatus storage_type);
    void UpdateCurrent(float iu, float iv, float iw);
    void SetPhaseVoltage(float ud, float uq, float electrical_angle);
};

#endif //FOC_H
