/**
 * @file        QD4310.h
 * @brief       QD4310电机控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-7-2
 * @version     V1.4.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-12-28, 将FOC非核心功能剥离,使用QD4310类集FOC实现解耦
 *		        V1.0.1创建于2026-1-9, 添加更多错误判断
 *		        V1.0.2创建于2026-3-8, 优化储存函数接口、优化qd4310设置api
 *		        V1.1.0创建于2026-3-9, 添加UART波特率设置功能
 *		        V1.2.0创建于2026-3-29, restore移至QD4310类内
 *		        V1.2.1创建于2026-5-5, 调整PID参数存储位置
 *		        V1.3.0创建于2026-5-30, 优化初始化时从储存器读取参数的流程,添加清除校准数据的功能
 *		        V1.3.1修改于2026-6-14,适配PID重构,修复若干问题
 *		        V1.4.0修改于2026-7-2,添加错误检测
 * @copyright   (c) 2026 QDrive
 */

#ifndef FOC_QD4310_QD4310_H
#define FOC_QD4310_QD4310_H

#include "QDrive.h"
#include "Storage.h"
#include "main.h"

class QD4310 : public QDrive{
public:
    enum ErrorCode : uint8_t {
        NoError = 0b0000'0000,
        CalibrationError = 0b0000'0001,
        VoltageError = 0b0000'0010,
        TemperatureError = 0b0000'0100,
    } error_code = NoError;

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
    QD4310(const uint8_t pole_pairs, const uint16_t CtrlFrequency, const uint16_t CurrentCtrlFrequency,
           Filter& CurrentQFilter, Filter& CurrentDFilter, Filter& SpeedFilter,
           BLDC_Driver& driver, Encoder& encoder, Storage& storage, CurrentSensor& current_sensor,
           const PID& PID_CurrentQ, const PID& PID_CurrentD, const PID& PID_Speed, const PID& PID_Angle) :
        QDrive(pole_pairs, CtrlFrequency, CurrentCtrlFrequency,
            CurrentQFilter, CurrentDFilter, SpeedFilter,
            driver, encoder, current_sensor,
            PID_CurrentQ, PID_CurrentD, PID_Speed, PID_Angle),
        storage(storage) {}

    uint8_t ID{0};                   // 电机ID
    uint32_t uart_baud_rate{115200}; // UART波特率

    void init();
    bool start();
    bool stop();
    CalibrationStatus calibrate();
    void anticogging_calibrate();

    ErrorCode error_detect();

    // 获取电机角度,单位rad
    [[nodiscard]] float getAngle() const;

    /**
     * @brief QD4310控制设置函数
     * @param ctrl_type 控制类型
     * @return 设置成功返回true,失败返回false
     */
    bool Ctrl(CtrlType ctrl_type);

    /**
     * @brief FOC控制(速度环、角度环)中断服务函数
     */
    void Ctrl_ISR();

    /**
     * @brief 设置电机ID
     * @param id 电机ID,范围0-7
     * @return 设置成功返回true,失败返回false
    */
    bool setID(uint8_t id);

    /**
     * @brief 设置PID参数
     * @param pid_speed_kp 速度环比例系数
     * @param pid_speed_ki 速度环积分系数
     * @param pid_speed_kd 速度环微分系数
     * @param pid_angle_kp 角度环比例系数
     * @param pid_angle_ki 角度环积分系数
     * @param pid_angle_kd 角度环微分系数
     * @return 设置成功返回true,失败返回false
     */
    bool setPID(std::optional<float> pid_speed_kp,
                std::optional<float> pid_speed_ki,
                std::optional<float> pid_speed_kd,
                std::optional<float> pid_angle_kp,
                std::optional<float> pid_angle_ki,
                std::optional<float> pid_angle_kd);

    /**
     * @brief 设置速度和电流限制
     * @param speed_limit 速度限制,单位rpm
     * @param current_limit 电流限制,单位A
     * @return 设置成功返回true,失败返回false
     */
    bool setLimit(std::optional<float> speed_limit, std::optional<float> current_limit);

    /**
     * @brief 设置位置零点
     * @param position 位置零点,单位rad
     * @return 设置成功返回true,失败返回false
     */
    bool setZeroPosition(std::optional<float> position = {});

    /**
     * @brief 设置UART波特率
     * @param baud_rate 波特率,单位bps
     * @return 设置成功返回true,失败返回false
     */
    bool setUartBaudRate(uint32_t baud_rate);

protected:
    friend class ShellPlugs;

    enum StorageStatus:uint8_t {
        STORAGE_NONE = 0b0000'0000,
        STORAGE_BASE_CALIBRATE_OK = 0b0000'0001,
        STORAGE_ANTICOGGING_CALIBRATE_OK = 0b0000'0010,
        STORAGE_PID_PARAMETER_OK = 0b0000'0100,
        STORAGE_LIMIT_OK = 0b0000'1000,
        STORAGE_PLUG_OK = 0b0001'0000,
        STORAGE_ZERO_POS_OK = 0b0010'0000,
        STORAGE_ALL_OK = STORAGE_BASE_CALIBRATE_OK |
                         STORAGE_ANTICOGGING_CALIBRATE_OK |
                         STORAGE_PID_PARAMETER_OK |
                         STORAGE_LIMIT_OK |
                         STORAGE_PLUG_OK |
                         STORAGE_ZERO_POS_OK,
    };

    static constexpr uint8_t STORAGE_MAGIC = 0xAA; // 存储器魔术字,储存在0x000

    Storage& storage;     //存储器
    float zero_pos{0.0f}; //位置零点

    void restore_calibration();
    void load_storage_calibration();
    void freeze_storage_calibration(StorageStatus storage_type);
    void clear_storage_calibration(StorageStatus storage_type) const;
};

#endif //FOC_QD4310_QD4310_H
