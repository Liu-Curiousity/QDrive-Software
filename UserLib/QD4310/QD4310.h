/**
 * @brief 		QD4310.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2025/12/27
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于2025/12/27
 * */

#ifndef FOC_QD4310_QD4310_H
#define FOC_QD4310_QD4310_H

#include "FOC.h"
#include "Storage.h"
#include "main.h"

class QD4310 : public FOC {
public:
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
        FOC(pole_pairs, CtrlFrequency, CurrentCtrlFrequency,
            CurrentQFilter, CurrentDFilter, SpeedFilter,
            driver, encoder, current_sensor,
            PID_CurrentQ, PID_CurrentD, PID_Speed, PID_Angle),
        storage(storage) {}

    uint8_t ID{0}; // 电机ID

    void init();
    void start();
    void stop();
    void calibrate();
    void anticogging_calibrate();

    // 获取电机角度,单位rad
    [[nodiscard]] float getAngle() const;

    /**
     * @brief QD4310控制设置函数
     * @param ctrl_type 控制类型
     * @param value 控制值
     */
    void Ctrl(CtrlType ctrl_type, float value);

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
     * @brief 设置位置零点
     * @param position 位置零点,单位rad
     */
    void setZeroPosition(float position);

private:
    friend void foc_config_list();
    friend void foc_store();
    friend void foc_restore();

    enum StorageStatus:uint8_t {
        STORAGE_NONE = 0b0000'0000,
        STORAGE_BASE_CALIBRATE_OK = 0b0000'0001,
        STORAGE_ANTICOGGING_CALIBRATE_OK = 0b0000'0010,
        STORAGE_PID_PARAMETER_OK = 0b0000'0100,
        STORAGE_LIMIT_OK = 0b0000'1000,
        STORAGE_ID_OK = 0b0001'0000,
        STORAGE_ZERO_POS_OK = 0b0010'0000,
        STORAGE_ALL_OK = STORAGE_BASE_CALIBRATE_OK |
                         STORAGE_ANTICOGGING_CALIBRATE_OK |
                         STORAGE_PID_PARAMETER_OK |
                         STORAGE_LIMIT_OK |
                         STORAGE_ID_OK |
                         STORAGE_ZERO_POS_OK,
    };

    static constexpr uint8_t STORAGE_MAGIC = 0xAA; // 存储器魔术字,储存在0x000

    Storage& storage;     //存储器
    float zero_pos{0.0f}; //位置零点

    void load_storage_calibration();
    void freeze_storage_calibration(StorageStatus storage_type);
};

#endif //FOC_QD4310_QD4310_H
