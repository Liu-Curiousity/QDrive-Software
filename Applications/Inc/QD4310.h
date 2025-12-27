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

    void init() {
        // 1.初始化FOC
        FOC::init();
        // 2.初始化flash
        if (!storage.initialized)
            storage.init();
        // 3.从flash中读取校准数据
        load_storage_calibration();
    }

    void start() {
        FOC::start();
        if (started) HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }

    void stop() {
        FOC::stop();
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    }

    void calibrate() {
        if (!enabled) return; // 如果没有使能,则不能校准
        if (started) return;  // 如果已经启动,则不能校准
        FOC::calibrate();
        freeze_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 保存基础校准数据
    }

    void anticogging_calibrate() {
        if (!enabled) return;    // 如果没有使能,则不能校准
        if (!calibrated) return; // 如果没有基础校准,则不能校准
        if (started) return;     // 如果已经启动,则不能校准
        FOC::anticogging_calibrate();
        freeze_storage_calibration(STORAGE_ANTICOGGING_CALIBRATE_OK); // 储存齿槽转矩补偿表
    }

    /**
     * @brief 设置PID参数
     * @param pid_speed_kp 速度环比例系数,若为NAN则不更新
     * @param pid_speed_ki 速度环积分系数,若为NAN则不更新
     * @param pid_speed_kd 速度环微分系数,若为NAN则不更新
     * @param pid_angle_kp 角度环比例系数,若为NAN则不更新
     * @param pid_angle_ki 角度环积分系数,若为NAN则不更新
     * @param pid_angle_kd 角度环微分系数,若为NAN则不更新
     */
    void setPID(const float pid_speed_kp, const float pid_speed_ki, const float pid_speed_kd,
                const float pid_angle_kp, const float pid_angle_ki, const float pid_angle_kd) {
        if (!std::isnan(pid_speed_kp)) PID_Speed.kp = pid_speed_kp;
        if (!std::isnan(pid_speed_ki)) PID_Speed.ki = pid_speed_ki;
        if (!std::isnan(pid_speed_kd)) PID_Speed.kd = pid_speed_kd;
        if (!std::isnan(pid_angle_kp)) PID_Angle.kp = pid_angle_kp;
        if (!std::isnan(pid_angle_ki)) PID_Angle.ki = pid_angle_ki;
        if (!std::isnan(pid_angle_kd)) PID_Angle.kd = pid_angle_kd;
    }

    /**
     * @brief 设置速度和电流限制
     * @param speed_limit 速度限制,单位rpm
     * @param current_limit 电流限制,单位A
     */
    void setLimit(float speed_limit, float current_limit);

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

    Storage& storage; //存储器

    void load_storage_calibration();
    void freeze_storage_calibration(StorageStatus storage_type);
};

inline void QD4310::setLimit(const float speed_limit, const float current_limit) {
    if (!std::isnan(speed_limit)) {
        PID_Angle.output_limit_p = speed_limit;
        PID_Angle.output_limit_n = -speed_limit;
    }
    if (!std::isnan(current_limit)) {
        PID_Speed.output_limit_p = current_limit;
        PID_Speed.output_limit_n = -current_limit;
    }
}

inline void QD4310::load_storage_calibration() {
    StorageStatus storage_status;
    storage.read(0x000, reinterpret_cast<uint8_t *>(&storage_status), sizeof(storage_status));
    if ((storage_status & 0xC0) == STORAGE_BASE_CALIBRATE_OK) {
        // 如果基础校准数据正常,则读取
        storage.read(0x100, reinterpret_cast<uint8_t *>(&encoder_direction), sizeof(encoder_direction));
        storage.read(0x110, reinterpret_cast<uint8_t *>(&zero_electric_angle), sizeof(zero_electric_angle));
        storage.read(0x120, reinterpret_cast<uint8_t *>(&iu_offset), sizeof(iu_offset));
        storage.read(0x130, reinterpret_cast<uint8_t *>(&iv_offset), sizeof(iv_offset));
        storage.read(0x140, reinterpret_cast<uint8_t *>(&phase_resistance), sizeof(phase_resistance));
        storage.read(0x150, reinterpret_cast<uint8_t *>(&phase_inductance), sizeof(phase_inductance));
        calibrated = true;
    }
    if ((storage_status & 0x30) == STORAGE_ANTICOGGING_CALIBRATE_OK) {
        storage.read(0x800, reinterpret_cast<uint8_t *>(anticogging_map), sizeof(anticogging_map));
        anticogging_calibrated = true;
    }
    if ((storage_status & 0x0C) == STORAGE_PID_PARAMETER_OK) {
        storage.read(0x160, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
        storage.read(0x170, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
        storage.read(0x180, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
        storage.read(0x190, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
        storage.read(0x1A0, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
        storage.read(0x1B0, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));
    }
    if ((storage_status & 0x03) == STORAGE_LIMIT_OK) {
        storage.read(0x1C0, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p), sizeof(PID_Angle.output_limit_p));
        PID_Angle.output_limit_n = -PID_Angle.output_limit_p;
        storage.read(0x1D0, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p), sizeof(PID_Speed.output_limit_p));
        PID_Speed.output_limit_n = -PID_Speed.output_limit_p;
    }

    storage.read(0x1E0, &ID, sizeof(ID));
}

/**
 * @brief 储存校准数据
 * @param storage_type 储存数据类型
 */
inline void QD4310::freeze_storage_calibration(const StorageStatus storage_type) {
    StorageStatus storage_status;
    storage.read(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
    switch (storage_type) {
        case STORAGE_BASE_CALIBRATE_OK:
            // 储存基础校准数据
            storage.write(0x100, reinterpret_cast<uint8_t *>(&encoder_direction), sizeof(encoder_direction));
            storage.write(0x110, reinterpret_cast<uint8_t *>(&zero_electric_angle), sizeof(zero_electric_angle));
            storage.write(0x120, reinterpret_cast<uint8_t *>(&iu_offset), sizeof(iu_offset));
            storage.write(0x130, reinterpret_cast<uint8_t *>(&iv_offset), sizeof(iv_offset));
            storage.write(0x140, reinterpret_cast<uint8_t *>(&phase_resistance), sizeof(phase_resistance));
            storage.write(0x150, reinterpret_cast<uint8_t *>(&phase_inductance), sizeof(phase_inductance));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0x3F) | STORAGE_BASE_CALIBRATE_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_ANTICOGGING_CALIBRATE_OK:
            // 储存齿槽转矩补偿表
            storage.write(0x800, reinterpret_cast<uint8_t *>(anticogging_map), sizeof(anticogging_map));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xCF) | STORAGE_ANTICOGGING_CALIBRATE_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_PID_PARAMETER_OK:
            // 储存PID参数
            storage.write(0x160, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
            storage.write(0x170, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
            storage.write(0x180, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
            storage.write(0x190, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
            storage.write(0x1A0, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
            storage.write(0x1B0, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xF3) | STORAGE_PID_PARAMETER_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_LIMIT_OK:
            storage.write(0x1C0, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p),
                          sizeof(PID_Angle.output_limit_p));
            storage.write(0x1D0, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p),
                          sizeof(PID_Speed.output_limit_p));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xFC) | STORAGE_LIMIT_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        default: ;
    }
    storage.write(0x1E0, &ID, sizeof(ID));
}


#endif //FOC_QD4310_QD4310_H
