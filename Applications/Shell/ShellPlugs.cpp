/**
 * @file        ShellPlugs.cpp
 * @brief       shell 接口函数
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-7-2
 * @version     V1.5.3
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-6-22
 *		        V1.1.0创建于2025-7-8, 添加校准接口
 *		        V1.2.0创建于2025-7-11, 添加限幅设置接口
 *		        V1.3.0创建于2025-7-24, 添加静默模式
 *		        V1.4.0创建于2025-8-15, 添加ID设置功能
 *		        V1.4.0创建于2025-12-18, 添加角度步进控制功能
 *		        V1.4.0创建于2025-12-28, 适配QD4310类
 *		        V1.4.1创建于2026-1-9, 重新实现轻量化atof函数，避免引入庞大的标准库
 *		        V1.5.0创建于2026-3-9, 添加UART波特率设置功能
 *		        V1.5.1创建于2026-5-5, 修复角度步进模式和速度模式均错误显示角度模式的问题
 *		        V1.5.2创建于2026-5-30, 补充打印校准信息
 *		        V1.5.3创建于2026-7-2, 补充打印错误信息
 * @copyright   (c) 2026 QDrive
 */

#include <algorithm>

#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "retarget/retarget.h"
#include "QD4310.h"
#include "FOC_config.h"
#include "main.h"

extern QD4310 qd4310;
extern Shell shell;

static float atof_lite(const char *s) {
    if (!s) return 0.0f;

    // 可选符号
    int sign = 1;
    if (*s == '+') {
        ++s;
    } else if (*s == '-') {
        sign = -1;
        ++s;
    }

    // 解析整数部分
    float int_part = 0.0f;
    bool has_digit = false;
    while (*s >= '0' && *s <= '9') {
        has_digit = true;
        int_part = int_part * 10.0f + static_cast<float>(*s - '0');
        ++s;
    }

    // 解析小数部分
    float frac_part = 0.0f;
    float scale = 1.0f;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            has_digit = true;
            frac_part = frac_part * 10.0f + static_cast<float>(*s - '0');
            scale *= 10.0f;
            ++s;
        }
    }

    if (!has_digit) return 0.0f;

    const float result = int_part + (frac_part / scale);
    return (sign < 0) ? -result : result;
}

signed short silent(char *data, unsigned short len) {
    return 0;
}

// 打印单行
#define PRINT(...)                          \
    do {                                    \
        if (shell.write != silent) {        \
        printf(__VA_ARGS__);                \
        printf("\r\n");}                    \
    } while (0)

void print_version() {
    PRINT("Hardware version %s", FOC_HARDWARE_VERSION);
    PRINT("Software version %s", FOC_SOFTWARE_VERSION);
}

void foc_info() {
    PRINT("Hardware Info:");
    PRINT("  Pole pairs       : %d ", FOC_POLE_PAIRS);
    PRINT("  KV rating        : %.1f rpm/V", FOC_KV);
    PRINT("  Nominal voltage  : %d V", FOC_NOMINAL_VOLTAGE);
    PRINT("  Phase inductance : %.2f mH", FOC_PHASE_INDUCTANCE);
    PRINT("  Phase resistance : %.2f Ω", FOC_PHASE_RESISTANCE);
    PRINT("  Torque constant  : %.2f Nm/A", FOC_TORQUE_CONSTANT);
    PRINT("  Max current      : %.2f A", FOC_MAX_CURRENT);
}

void foc_status() {
    PRINT("Motor Status:");
    PRINT("  CAN ID       : %03d", qd4310.ID);
    PRINT("  Status       : %s", qd4310.started ? "enabled" : "disabled");
    PRINT("  CtrlMode     : %s",
          qd4310.getCtrlType() == QD4310::CtrlType::CurrentCtrl ? "CurrentCtrl" :
          qd4310.getCtrlType() == QD4310::CtrlType::SpeedCtrl ? "SpeedCtrl" :
          qd4310.getCtrlType() == QD4310::CtrlType::AngleCtrl ? "AngleCtrl" :
          qd4310.getCtrlType() == QD4310::CtrlType::StepAngleCtrl ? "StepAngleCtrl" :
          qd4310.getCtrlType() == QD4310::CtrlType::LowSpeedCtrl ? "LowSpeedCtrl" : "Unknown");
    PRINT("  Current      : %.2f A", qd4310.getCurrent());
    PRINT("  Speed        : %.2f rpm", qd4310.getSpeed());
    PRINT("  Angle        : %.2f rad", qd4310.getAngle());
    PRINT("  Voltage      : %.2f V", qd4310.getVoltage());
}

void foc_config_help() {
    PRINT("Usage: config [--list | PARAM_PATH VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  config pid.speed.kp 0.1");
    PRINT("  config pid.speed.ki=0.1");
    PRINT("  config --help");
    PRINT("  config --list");
    PRINT("");
    PRINT("Configuration Parameters:");
    PRINT("  pid.speed.kp       : Speed PID proportional gain");
    PRINT("  pid.speed.ki       : Speed PID integral gain");
    PRINT("  pid.speed.kd       : Speed PID derivative gain");
    PRINT("  pid.angle.kp       : Angle PID proportional gain");
    PRINT("  pid.angle.ki       : Angle PID integral gain");
    PRINT("  pid.angle.kd       : Angle PID derivative gain");
    PRINT("  limit.speed        : Speed limit in rpm");
    PRINT("  limit.current      : Current limit in A");
    PRINT("  can.id             : CAN ID of the motor (0-7)");
    PRINT("  uart.baud_rate     : UART BaudRate of the motor (50K-10M)");
    PRINT("  zero_pos           : Position zero offset in rad");

    // TODO:部分不可调
    // PRINT("  can.baud_rate      : CAN bus baud rate");
}

void foc_config_list() {
    PRINT("Current Configuration:");
    if (qd4310.PID_Speed.kp == 0)
        PRINT("pid.speed.kp = 0.000");
    else
        PRINT("pid.speed.kp = %.3g", qd4310.PID_Speed.kp);
    if (qd4310.PID_Speed.ki == 0)
        PRINT("pid.speed.ki = 0.000");
    else
        PRINT("pid.speed.ki = %.3g", qd4310.PID_Speed.ki);
    if (qd4310.PID_Speed.kd == 0)
        PRINT("pid.speed.kd = 0.000");
    else
        PRINT("pid.speed.kd = %.3g", qd4310.PID_Speed.kd);
    if (qd4310.PID_Angle.kp == 0)
        PRINT("pid.angle.kp = 0.000");
    else
        PRINT("pid.angle.kp = %.3g", qd4310.PID_Angle.kp);
    if (qd4310.PID_Angle.ki == 0)
        PRINT("pid.angle.ki = 0.000");
    else
        PRINT("pid.angle.ki = %.3g", qd4310.PID_Angle.ki);
    if (qd4310.PID_Angle.kd == 0)
        PRINT("pid.angle.kd = 0.000");
    else
        PRINT("pid.angle.kd = %.3g", qd4310.PID_Angle.kd);
    if (!qd4310.PID_Angle.output_limit_p)
        PRINT("limit.speed = no limit");
    else
        PRINT("limit.speed = %.3g rpm", qd4310.PID_Angle.output_limit_p.value());
    if (!qd4310.PID_Speed.output_limit_p)
        PRINT("limit.current = no limit");
    else
        PRINT("limit.current = %.3g A", qd4310.PID_Speed.output_limit_p.value());
    PRINT("can.id = %03d", qd4310.ID);
    // TODO: 波特率不可更改
    PRINT("can.baud_rate = 1'000'000");
    PRINT("uart.baud_rate = %u", qd4310.uart_baud_rate);
}

void foc_config(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        foc_config_help();
        return;
    }

    if (strcmp(argv[1], "--list") == 0) {
        foc_config_list();
        return;
    }

    const char *key = argv[1];
    const char *value = nullptr;

    if (strchr(key, '=') != nullptr) {
        // 解析 key=value 格式
        static char keybuf[128];
        strncpy(keybuf, key, sizeof(keybuf) - 1);
        keybuf[sizeof(keybuf) - 1] = '\0';

        char *eq = strchr(keybuf, '=');
        *eq = '\0';
        key = keybuf;
        value = eq + 1;
    } else if (argc >= 3) {
        value = argv[2];
    }

    if (strcmp(key, "zero_pos") == 0) {
        if (qd4310.setZeroPosition(value ? atof_lite(value) : qd4310.getAngle()))
            PRINT("Setting config [zero_pos]");
        else
            PRINT("QDrive is running, please disable it first");
    } else if (value) {
        float valf = atof_lite(value);
        if (strcmp(key, "pid.speed.kp") == 0) {
            qd4310.setPID(valf, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
        } else if (strcmp(key, "pid.speed.ki") == 0) {
            qd4310.setPID(std::nullopt, valf, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
        } else if (strcmp(key, "pid.speed.kd") == 0) {
            qd4310.setPID(std::nullopt, std::nullopt, valf, std::nullopt, std::nullopt, std::nullopt);
        } else if (strcmp(key, "pid.angle.kp") == 0) {
            qd4310.setPID(std::nullopt, std::nullopt, std::nullopt, valf, std::nullopt, std::nullopt);
        } else if (strcmp(key, "pid.angle.ki") == 0) {
            qd4310.setPID(std::nullopt, std::nullopt, std::nullopt, std::nullopt, valf, std::nullopt);
        } else if (strcmp(key, "pid.angle.kd") == 0) {
            qd4310.setPID(std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, valf);
        } else if (strcmp(key, "limit.speed") == 0) {
            qd4310.setLimit(valf, std::nullopt);
        } else if (strcmp(key, "limit.current") == 0) {
            qd4310.setLimit(std::nullopt, valf);
        } else if (strcmp(key, "can.id") == 0) {
            if (!qd4310.setID(valf)) {
                PRINT("Invalid CAN ID: %d, must be between 0 and 7", static_cast<int>(valf));
                return;
            }
        } else if (strcmp(key, "uart.baud_rate") == 0) {
            if (!qd4310.setUartBaudRate(valf)) {
                PRINT("Invalid UART baud rate: %d, must be between 10'000 and 10'000'000", static_cast<int>(valf));
                return;
            }
            PRINT("UART baud rate will be set after storing and rebooting");
        } else {
            PRINT("Unknown config target: %s", key);
            return;
        }
        if (valf == 0) {
            PRINT("Setting config [%s] = 0.000", key);
        } else {
            PRINT("Setting config [%s] = %.3g", key, valf);
        }
    } else {
        PRINT("Missing value for config [%s]", key);
    }
}

void foc_ctrl_help() {
    PRINT("Usage: ctrl [current VALUE | low_speed VALUE  | speed VALUE  | step_angle VALUE | angle VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  ctrl speed 100");
    PRINT("  ctrl speed=100");
    PRINT("  ctrl --help");
    PRINT("");
    PRINT("Control Parameters:");
    PRINT("  current           : Set current in Q axis (A)");
    PRINT("  low_speed         : Set speed by increasing angle (rpm)");
    PRINT("  speed             : Set speed (rpm)");
    PRINT("  angle             : Set angle (rad)");
    PRINT("  step_angle        : Step an specific angle (rad)");
}

void foc_ctrl(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        foc_ctrl_help();
        return;
    }

    const char *key = argv[1];
    const char *value = nullptr;

    if (strchr(key, '=') != nullptr) {
        // 解析 key=value 格式
        static char keybuf[128];
        strncpy(keybuf, key, sizeof(keybuf) - 1);
        keybuf[sizeof(keybuf) - 1] = '\0';

        char *eq = strchr(keybuf, '=');
        *eq = '\0';
        key = keybuf;
        value = eq + 1;
    } else if (argc >= 3) {
        value = argv[2];
    }

    if (!value) {
        PRINT("Missing value for ctrl [%s]", key);
        return;
    }
    if (!qd4310.started) {
        PRINT("QDrive is not running, please enable it first");
        return;
    }
    float valf = atof_lite(value);
    if (strcmp(key, "current") == 0) {
        PRINT("Setting current = %.2f A", valf);
        qd4310.Ctrl(QD4310::CtrlType::CurrentCtrl, valf);
    } else if (strcmp(key, "speed") == 0) {
        PRINT("Setting speed = %.2f rpm", valf);
        qd4310.Ctrl(QD4310::CtrlType::SpeedCtrl, valf);
    } else if (strcmp(key, "angle") == 0) {
        PRINT("Setting angle = %.2f rad", valf);
        qd4310.Ctrl(QD4310::CtrlType::AngleCtrl, valf);
    } else if (strcmp(key, "step_angle") == 0) {
        PRINT("Stepping %.2f rad angle", valf);
        qd4310.Ctrl(QD4310::CtrlType::StepAngleCtrl, valf);
    } else if (strcmp(key, "low_speed") == 0) {
        PRINT("Setting low_speed = %.2f rpm", valf);
        qd4310.Ctrl(QD4310::CtrlType::LowSpeedCtrl, valf);
    } else {
        PRINT("Unknown ctrl target: %s", key);
        foc_ctrl_help();
    }
}

void foc_enable() {
    if (qd4310.start()) {
        PRINT("QDrive enabled");
    } else if (qd4310.error_code & QD4310::CalibrationError) {
        PRINT("enable failed, please calibrate first");
    } else if (qd4310.error_code & QD4310::VoltageError) {
        PRINT("enable failed, voltage error");
    } else {
        PRINT("enable failed, unknown error");
    }
}

void foc_disable() {
    qd4310.stop();
    PRINT("QDrive disabled");
}

void foc_calibrate() {
    if (qd4310.started) {
        PRINT("QDrive is running, please disable it first");
        return;
    }
    if (qd4310.calibrated) {
        PRINT("QDrive already calibrated,do you want to re-calibrate? (y/n)");
        char response;
        while (!shellRead(&response, 1)) {
            delay(1);
        }
        if (response != 'y' && response != 'Y') {
            PRINT("Calibration aborted");
            return;
        }
    }
    PRINT("QDrive calibration started, please wait...");
    if (const auto status = qd4310.calibrate(); status == QD4310::CalibrationStatus::Success)
        PRINT("QDrive calibration completed");
    else if (status == QD4310::CalibrationStatus::EnvironmentError)
        PRINT("QDrive calibration failed: environment error");
    else if (status == QD4310::CalibrationStatus::VoltageError)
        PRINT("QDrive calibration failed: voltage error");
    else if (status == QD4310::CalibrationStatus::Busy)
        PRINT("QDrive calibration failed: busy");
    else if (status == QD4310::CalibrationStatus::CurrentSensorError)
        PRINT("QDrive calibration failed: current sensor error");
    else if (status == QD4310::CalibrationStatus::DriverError)
        PRINT("QDrive calibration failed: driver error");
    else if (status == QD4310::CalibrationStatus::EncoderError)
        PRINT("QDrive calibration failed: encoder error");
    else
        PRINT("QDrive calibration failed");
}

void foc_restore() {
    PRINT("Are you sure you want to restore factory settings? (y/n)");
    char response;
    while (!shellRead(&response, 1)) {
        delay(1);
    }
    if (response != 'y' && response != 'Y') {
        PRINT("Factory restore cancelled");
        return;
    }
    qd4310.restore_calibration(); // 恢复出厂设置
    PRINT("QDrive factory restore completed");
    foc_config_list();
}

void foc_store() {
    if (qd4310.started) {
        PRINT("QDrive is running, please disable it first");
        return;
    }
    foc_config_list();
    PRINT("Are you sure you want to store configurations? (y/n)");
    char response;
    while (!shellRead(&response, 1)) {
        delay(1);
    }
    if (response != 'y' && response != 'Y') {
        PRINT("Store operation cancelled");
        return;
    }
    qd4310.freeze_storage_calibration(
        static_cast<QD4310::StorageStatus>(QD4310::STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                           QD4310::STORAGE_LIMIT_OK |         // 储存限制参数
                                           QD4310::STORAGE_PLUG_OK)           // 储存ID
    );
    PRINT("Store configuration completed");
}

void shell_reboot() {
    NVIC_SystemReset();
}

void shell_silent() {
    shell.write = silent; // 禁止输出
}

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    silent, shell_silent, "Disable shell output, reboot to enable again"
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    version, print_version, Show version info
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    reboot, shell_reboot, reboot system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    store, foc_store, Store configurations
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    restore, foc_restore, Factory restore
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    ctrl, foc_ctrl, Set control targets
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    config, foc_config, Configure system parameters
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    calibrate, foc_calibrate, Calibrate FOC system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable, foc_disable, Disable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable, foc_enable, Enable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    status, foc_status, Show current motor status
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    info, foc_info, Show hardware information
);
