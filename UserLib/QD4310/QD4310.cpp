//
// Created by 26757 on 2025/12/28.
//
#include "QD4310.h"
#include <numbers>

using namespace std;

void QD4310::init() {
    // 1.初始化FOC
    FOC::init();
    // 2.初始化flash
    if (!storage.initialized)
        storage.init();
    // 3.从flash中读取校准数据
    load_storage_calibration();
}

void QD4310::start() {
    FOC::start();
    if (started) HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void QD4310::stop() {
    FOC::stop();
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void QD4310::calibrate() {
    if (!enabled) return; // 如果没有使能,则不能校准
    if (started) return;  // 如果已经启动,则不能校准
    FOC::calibrate();
    freeze_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 保存基础校准数据
}

void QD4310::anticogging_calibrate() {
    if (!enabled) return;    // 如果没有使能,则不能校准
    if (!calibrated) return; // 如果没有基础校准,则不能校准
    if (started) return;     // 如果已经启动,则不能校准
    FOC::anticogging_calibrate();
    freeze_storage_calibration(STORAGE_ANTICOGGING_CALIBRATE_OK); // 储存齿槽转矩补偿表
}

[[nodiscard]] float QD4310::getAngle() const {
    return wrap(FOC::getAngle() - zero_pos, 0, 2 * numbers::pi_v<float>);
}

void QD4310::Ctrl(const CtrlType ctrl_type, float value) {
    if (ctrl_type == CtrlType::AngleCtrl) {
        value = wrap(value + zero_pos, 0, 2 * numbers::pi_v<float>);
    }
    FOC::Ctrl(ctrl_type, value);
}

void QD4310::setPID(const float pid_speed_kp, const float pid_speed_ki, const float pid_speed_kd,
                    const float pid_angle_kp, const float pid_angle_ki, const float pid_angle_kd) {
    if (!isnan(pid_speed_kp)) PID_Speed.kp = pid_speed_kp;
    if (!isnan(pid_speed_ki)) PID_Speed.ki = pid_speed_ki;
    if (!isnan(pid_speed_kd)) PID_Speed.kd = pid_speed_kd;
    if (!isnan(pid_angle_kp)) PID_Angle.kp = pid_angle_kp;
    if (!isnan(pid_angle_ki)) PID_Angle.ki = pid_angle_ki;
    if (!isnan(pid_angle_kd)) PID_Angle.kd = pid_angle_kd;
}

void QD4310::setLimit(const float speed_limit, const float current_limit) {
    if (!isnan(speed_limit)) {
        PID_Angle.output_limit_p = speed_limit;
        PID_Angle.output_limit_n = -speed_limit;
    }
    if (!isnan(current_limit)) {
        PID_Speed.output_limit_p = current_limit;
        PID_Speed.output_limit_n = -current_limit;
    }
}

void QD4310::setZeroPosition(const float position) {
    zero_pos = wrap(zero_pos + position, 0, 2 * numbers::pi_v<float>);
}

void QD4310::load_storage_calibration() {
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
        storage.read(0x210, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
        storage.read(0x220, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
        storage.read(0x230, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
        storage.read(0x240, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
        storage.read(0x250, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
        storage.read(0x260, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));
        storage.read(0x270, reinterpret_cast<uint8_t *>(&zero_pos), sizeof(zero_pos));
    }
    if ((storage_status & 0x03) == STORAGE_LIMIT_OK) {
        storage.read(0x300, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p), sizeof(PID_Angle.output_limit_p));
        PID_Angle.output_limit_n = -PID_Angle.output_limit_p;
        storage.read(0x310, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p), sizeof(PID_Speed.output_limit_p));
        PID_Speed.output_limit_n = -PID_Speed.output_limit_p;
    }

    storage.read(0x1E0, &ID, sizeof(ID));
}

/**
 * @brief 储存校准数据
 * @param storage_type 储存数据类型
 */
void QD4310::freeze_storage_calibration(const StorageStatus storage_type) {
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
            storage.write(0x210, reinterpret_cast<uint8_t *>(&PID_Speed.kp), sizeof(PID_Speed.kp));
            storage.write(0x220, reinterpret_cast<uint8_t *>(&PID_Speed.ki), sizeof(PID_Speed.ki));
            storage.write(0x230, reinterpret_cast<uint8_t *>(&PID_Speed.kd), sizeof(PID_Speed.kd));
            storage.write(0x240, reinterpret_cast<uint8_t *>(&PID_Angle.kp), sizeof(PID_Angle.kp));
            storage.write(0x250, reinterpret_cast<uint8_t *>(&PID_Angle.ki), sizeof(PID_Angle.ki));
            storage.write(0x260, reinterpret_cast<uint8_t *>(&PID_Angle.kd), sizeof(PID_Angle.kd));
            storage.write(0x270, reinterpret_cast<uint8_t *>(&zero_pos), sizeof(zero_pos));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xF3) | STORAGE_PID_PARAMETER_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        case STORAGE_LIMIT_OK:
            storage.write(0x300, reinterpret_cast<uint8_t *>(&PID_Angle.output_limit_p),
                          sizeof(PID_Angle.output_limit_p));
            storage.write(0x310, reinterpret_cast<uint8_t *>(&PID_Speed.output_limit_p),
                          sizeof(PID_Speed.output_limit_p));

            // 更新储存状态
            storage_status = static_cast<StorageStatus>((storage_status & 0xFC) | STORAGE_LIMIT_OK);
            storage.write(0x000, reinterpret_cast<uint8_t *>(&storage_status), 1);
            break;
        default: ;
    }
    storage.write(0x1E0, &ID, sizeof(ID));
}
