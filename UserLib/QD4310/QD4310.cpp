/**
 * @file        QD4310.cpp
 * @brief       QD4310电机控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-6-14
 * @version     V1.3.1
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
 * @copyright   (c) 2026 QDrive
 */

#include "QD4310.h"
#include "FOC_config.h"
#include <algorithm>
#include <numbers>
#include "usart.h"

using namespace std;

void QD4310::init() {
    // 1.初始化flash
    if (!storage.initialized)
        storage.init();
    // 2.从flash中读取校准数据
    load_storage_calibration();
    // 3.初始化FOC
    FOC::init();
    // 4.完成初始化
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

void QD4310::start() {
    FOC::start();
    if (started) {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    }
}

void QD4310::stop() {
    FOC::stop();
    if (!started) {
        FOC::Ctrl(CtrlType::CurrentCtrl, 0);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
}

auto QD4310::calibrate() -> CalibrationStatus {
    if (!enabled) return CalibrationStatus::EnvironmentError; // 如果没有使能,则不能校准
    if (started) return CalibrationStatus::Busy;              // 如果已经启动,则不能校准
    const auto status = FOC::calibrate();
    if (status == CalibrationStatus::Success)                  // 如果基础校准成功
        freeze_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 保存基础校准数据
    // else if (status == CalibrationStatus::CurrentSensorError ||
    //          status == CalibrationStatus::DriverError ||
    //          status == CalibrationStatus::EncoderError) {
    //     clear_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 清除基础校准数据
    // }
    return status;
}

void QD4310::anticogging_calibrate() {
    if (!enabled) return;         // 如果没有使能,则不能校准
    if (!calibrated) return;      // 如果没有基础校准,则不能校准
    if (started) return;          // 如果已经启动,则不能校准
    if (!anticogging_map) return; // 如果补偿表指针为空,则不能校准
    FOC::anticogging_calibrate();
    if (anticogging_calibrated)                                       // 如果齿槽转矩补偿校准成功
        freeze_storage_calibration(STORAGE_ANTICOGGING_CALIBRATE_OK); // 储存齿槽转矩补偿表
}

[[nodiscard]] float QD4310::getAngle() const {
    return wrap(FOC::getAngle() - zero_pos, 0, 2 * numbers::pi_v<float>);
}

bool QD4310::Ctrl(const CtrlType ctrl_type, float value) {
    if (!started) return false;
    if (ctrl_type == CtrlType::AngleCtrl) {
        value = wrap(value + zero_pos, 0, 2 * numbers::pi_v<float>);
    }
    FOC::Ctrl(ctrl_type, value);
    return true;
}

bool QD4310::setID(const uint8_t id) {
    if (id > 7) return false; // ID必须在0-7之间
    ID = id;
    return true;
}

bool QD4310::setPID(const std::optional<float> pid_speed_kp,
                    const std::optional<float> pid_speed_ki,
                    const std::optional<float> pid_speed_kd,
                    const std::optional<float> pid_angle_kp,
                    const std::optional<float> pid_angle_ki,
                    const std::optional<float> pid_angle_kd) {
    if (pid_speed_kp) PID_Speed.kp = pid_speed_kp.value();
    if (pid_speed_ki) PID_Speed.ki = pid_speed_ki.value();
    if (pid_speed_kd) PID_Speed.kd = pid_speed_kd.value();
    if (pid_angle_kp) PID_Angle.kp = pid_angle_kp.value();
    if (pid_angle_ki) PID_Angle.ki = pid_angle_ki.value();
    if (pid_angle_kd) PID_Angle.kd = pid_angle_kd.value();
    return true;
}

bool QD4310::setLimit(const std::optional<float> speed_limit, const std::optional<float> current_limit) {
    if (speed_limit) {
        PID_Angle.output_limit_p = speed_limit.value();
        PID_Angle.output_limit_n = -speed_limit.value();
    }
    if (current_limit) {
        PID_Speed.output_limit_p = current_limit.value();
        PID_Speed.output_limit_n = -current_limit.value();
    }
    return true;
}

bool QD4310::setZeroPosition(const std::optional<float> position) {
    zero_pos = wrap(zero_pos + position.value_or(getAngle()), 0, 2 * numbers::pi_v<float>);
    freeze_storage_calibration(STORAGE_ZERO_POS_OK);
    return true;
}

bool QD4310::setUartBaudRate(const uint32_t baud_rate) {
    if (baud_rate < 50'000 || baud_rate > 10'000'000) return false; // 波特率必须在50'000-10'000'000之间
    uart_baud_rate = baud_rate;
    // TODO: 重写配置UART
    HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    return true;
}

void QD4310::restore_calibration() {
    setPID(FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_KD,
           FOC_ANGLE_KP, FOC_ANGLE_KI, FOC_ANGLE_KD);
    setLimit(FOC_MAX_SPEED, FOC_MAX_CURRENT);
    setID(0);
    setUartBaudRate(115200);

    freeze_storage_calibration(
        static_cast<StorageStatus>(STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                   STORAGE_LIMIT_OK |         // 储存限制参数
                                   STORAGE_PLUG_OK)           // 储存ID
    );
}

void QD4310::load_storage_calibration() {
    uint8_t storage_magic;
    storage.read(0x000, &storage_magic, sizeof(storage_magic));
    if (storage_magic != STORAGE_MAGIC) { return; } // 如果魔术字不对,则说明没有校准数据

    StorageStatus storage_status;
    storage.read(0x010, &storage_status, sizeof(storage_status));
    if ((storage_status & STORAGE_BASE_CALIBRATE_OK) == STORAGE_BASE_CALIBRATE_OK) {
        // 如果基础校准数据正常,则读取
        storage.read(0x100, &encoder_direction, sizeof(encoder_direction));
        storage.read(0x110, &zero_electric_angle, sizeof(zero_electric_angle));
        storage.read(0x120, &iu_offset, sizeof(iu_offset));
        storage.read(0x130, &iv_offset, sizeof(iv_offset));
        storage.read(0x140, &phase_resistance, sizeof(phase_resistance));
        storage.read(0x150, &phase_inductance, sizeof(phase_inductance));
        calibrated = true;
    }
    if ((storage_status & STORAGE_ANTICOGGING_CALIBRATE_OK) == STORAGE_ANTICOGGING_CALIBRATE_OK) {
        storage.read(0x800, anticogging_map, sizeof(anticogging_map));
        anticogging_calibrated = true;
    }
    if ((storage_status & STORAGE_PID_PARAMETER_OK) == STORAGE_PID_PARAMETER_OK) {
        storage.read(0x200, &PID_Speed.kp, sizeof(PID_Speed.kp));
        storage.read(0x210, &PID_Speed.ki, sizeof(PID_Speed.ki));
        storage.read(0x220, &PID_Speed.kd, sizeof(PID_Speed.kd));
        storage.read(0x230, &PID_Angle.kp, sizeof(PID_Angle.kp));
        storage.read(0x240, &PID_Angle.ki, sizeof(PID_Angle.ki));
        storage.read(0x250, &PID_Angle.kd, sizeof(PID_Angle.kd));
    }
    if ((storage_status & STORAGE_LIMIT_OK) == STORAGE_LIMIT_OK) {
        storage.read(0x300, &PID_Angle.output_limit_p, sizeof(PID_Angle.output_limit_p));
        PID_Angle.output_limit_n = -PID_Angle.output_limit_p.value();
        storage.read(0x310, &PID_Speed.output_limit_p, sizeof(PID_Speed.output_limit_p));
        PID_Speed.output_limit_n = -PID_Speed.output_limit_p.value();
    }
    if ((storage_status & STORAGE_PLUG_OK) == STORAGE_PLUG_OK) {
        storage.read(0x400, &ID, sizeof(ID));
        storage.read(0x410, &uart_baud_rate, sizeof(uart_baud_rate));
        setUartBaudRate(uart_baud_rate); // 配置UART波特率
    }
    if ((storage_status & STORAGE_ZERO_POS_OK) == STORAGE_ZERO_POS_OK) {
        storage.read(0x500, &zero_pos, sizeof(zero_pos));
    }
}

/**
 * @brief 储存校准数据
 * @param storage_type 储存数据类型
 */
void QD4310::freeze_storage_calibration(const StorageStatus storage_type) {
    static uint8_t storage_buffer[0x100];
    uint8_t storage_magic;
    StorageStatus storage_status;
    storage.read(0x000, &storage_magic, sizeof(storage_magic));
    // 如果魔术字不对,则清零所有储存标志
    if (storage_magic != STORAGE_MAGIC) {
        storage_magic = STORAGE_MAGIC;
        storage_status = STORAGE_NONE;
        std::fill_n(storage_buffer, sizeof(storage_buffer), 0);
        *reinterpret_cast<decltype(storage_magic) *>(&storage_buffer[0x000]) = storage_magic;
        *reinterpret_cast<decltype(storage_status) *>(&storage_buffer[0x010]) = storage_status;
        storage.write(0x000, storage_buffer, 0x020);
    }

    storage.read(0x010, &storage_status, 1);
    if ((storage_type & STORAGE_BASE_CALIBRATE_OK) == STORAGE_BASE_CALIBRATE_OK) {
        // 储存基础校准数据
        std::fill_n(storage_buffer, sizeof(storage_buffer), 0);
        *reinterpret_cast<decltype(encoder_direction) *>(&storage_buffer[0x000]) = encoder_direction;
        *reinterpret_cast<decltype(zero_electric_angle) *>(&storage_buffer[0x010]) = zero_electric_angle;
        *reinterpret_cast<decltype(iu_offset) *>(&storage_buffer[0x020]) = iu_offset;
        *reinterpret_cast<decltype(iv_offset) *>(&storage_buffer[0x030]) = iv_offset;
        *reinterpret_cast<decltype(phase_resistance) *>(&storage_buffer[0x040]) = phase_resistance;
        *reinterpret_cast<decltype(phase_inductance) *>(&storage_buffer[0x050]) = phase_inductance;
        storage.write(0x100, storage_buffer, 0x060);
    }
    if ((storage_type & STORAGE_ANTICOGGING_CALIBRATE_OK) == STORAGE_ANTICOGGING_CALIBRATE_OK) {
        // 储存齿槽转矩补偿表
        storage.write(0x800, anticogging_map, sizeof(anticogging_map));
    }
    if ((storage_type & STORAGE_PID_PARAMETER_OK) == STORAGE_PID_PARAMETER_OK) {
        // 储存PID参数
        std::fill_n(storage_buffer, sizeof(storage_buffer), 0);
        *reinterpret_cast<decltype(PID_Speed.kp) *>(&storage_buffer[0x000]) = PID_Speed.kp;
        *reinterpret_cast<decltype(PID_Speed.ki) *>(&storage_buffer[0x010]) = PID_Speed.ki;
        *reinterpret_cast<decltype(PID_Speed.kd) *>(&storage_buffer[0x020]) = PID_Speed.kd;
        *reinterpret_cast<decltype(PID_Angle.kp) *>(&storage_buffer[0x030]) = PID_Angle.kp;
        *reinterpret_cast<decltype(PID_Angle.ki) *>(&storage_buffer[0x040]) = PID_Angle.ki;
        *reinterpret_cast<decltype(PID_Angle.kd) *>(&storage_buffer[0x050]) = PID_Angle.kd;
        storage.write(0x200, storage_buffer, 0x060);
    }
    if ((storage_type & STORAGE_LIMIT_OK) == STORAGE_LIMIT_OK) {
        // 储存限幅参数
        std::fill_n(storage_buffer, sizeof(storage_buffer), 0);
        *reinterpret_cast<decltype(PID_Angle.output_limit_p) *>(&storage_buffer[0x000]) = PID_Angle.output_limit_p;
        *reinterpret_cast<decltype(PID_Speed.output_limit_p) *>(&storage_buffer[0x010]) = PID_Speed.output_limit_p;
        storage.write(0x300, storage_buffer, 0x020);
    }
    if ((storage_type & STORAGE_PLUG_OK) == STORAGE_PLUG_OK) {
        std::fill_n(storage_buffer, sizeof(storage_buffer), 0);
        *reinterpret_cast<decltype(ID) *>(&storage_buffer[0x000]) = ID;                         // 储存ID
        *reinterpret_cast<decltype(uart_baud_rate) *>(&storage_buffer[0x010]) = uart_baud_rate; // 储存波特率
        storage.write(0x400, storage_buffer, 0x020);
    }
    if ((storage_type & STORAGE_ZERO_POS_OK) == STORAGE_ZERO_POS_OK) {
        // 储存位置零点
        storage.write(0x500, &zero_pos, sizeof(zero_pos));
    }

    // 更新储存状态
    storage_status = static_cast<StorageStatus>(storage_status | storage_type);
    storage.write(0x010, &storage_status, sizeof(storage_status));
}

/**
 * @brief 清除校准数据
 * @param storage_type 储存数据类型
 */
void QD4310::clear_storage_calibration(const StorageStatus storage_type) const {
    uint8_t storage_magic;
    StorageStatus storage_status;
    storage.read(0x000, &storage_magic, sizeof(storage_magic));
    // 如果魔术字不对,则清零所有储存标志
    if (storage_magic != STORAGE_MAGIC) {
        storage_magic = STORAGE_MAGIC;
        storage_status = STORAGE_NONE;
        storage.write(0x000, &storage_magic, sizeof(storage_magic));
        storage.write(0x010, &storage_status, sizeof(storage_status));
    }
    // 更新储存状态
    storage.read(0x010, &storage_status, 1);
    storage_status = static_cast<StorageStatus>(storage_status & !storage_type);
    storage.write(0x010, &storage_status, sizeof(storage_status));
}
