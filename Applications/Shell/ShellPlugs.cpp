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

extern QD4310 qd4310;
extern Shell shell;

class ShellPlugs : public QD4310 {
public:
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

    static void print_len(const char *format, ...) {
        if (shell.write != silent) {
            va_list args;
            va_start(args, format);
            vprintf(format, args);
            va_end(args);
            printf("\r\n");
        }
    }

    static void print(const char *format, ...) {
        if (shell.write != silent) {
            va_list args;
            va_start(args, format);
            vprintf(format, args);
            va_end(args);
        }
    }

    struct ConfigItem {
        const char *name;
        const char *description;
        const char *unit;
        const char *format;
        void (*print_value)(const ConfigItem&);
        bool (*set_value)(float);
    };

    inline static const ConfigItem ConfigItems[12] = {
        {
            "pid.speed.kp", "Speed PID proportional gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Speed.kp);
            },
            [](float value) {
                qd4310.setPID(value, std::nullopt, std::nullopt,
                              std::nullopt, std::nullopt, std::nullopt);
                return true;
            }
        },
        {
            "pid.speed.ki", "Speed PID integral gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Speed.ki);
            },
            [](float value) {
                qd4310.setPID(std::nullopt, value, std::nullopt,
                              std::nullopt, std::nullopt, std::nullopt);
                return true;
            }
        },
        {
            "pid.speed.kd", "Speed PID derivative gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Speed.kd);
            },
            [](float value) {
                qd4310.setPID(std::nullopt, std::nullopt, value,
                              std::nullopt, std::nullopt, std::nullopt);
                return true;
            }
        },
        {
            "pid.angle.kp", "Angle PID proportional gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Angle.kp);
            },
            [](float value) {
                qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
                              value, std::nullopt, std::nullopt);
                return true;
            }
        },
        {
            "pid.angle.ki", "Angle PID integral gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Angle.ki);
            },
            [](float value) {
                qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
                              std::nullopt, value, std::nullopt);
                return true;
            }
        },
        {
            "pid.angle.kd", "Angle PID derivative gain", nullptr, "%.3g",
            [](const ConfigItem& self) {
                print(self.format, qd4310.PID_Angle.kd);
            },
            [](float value) {
                qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
                              std::nullopt, std::nullopt, value);
                return true;
            }
        },
        {
            "limit.speed", "Speed limit in rpm", "rpm", "%.3g",
            [](const ConfigItem& self) {
                if (!qd4310.PID_Angle.output_limit_p)
                    print("no limit");
                else
                    print(self.format, qd4310.PID_Angle.output_limit_p.value());
            },
            [](float value) {
                return qd4310.setLimit(value, std::nullopt);
            }
        },
        {
            "limit.current", "Current limit in A", "A", "%.3g",
            [](const ConfigItem& self) {
                if (!qd4310.PID_Speed.output_limit_p)
                    print("no limit");
                else
                    print(self.format, qd4310.PID_Speed.output_limit_p.value());
            },
            [](const float value) {
                return qd4310.setLimit(std::nullopt, value);
            }
        },
        {
            "can.id", "CAN ID of the motor (0-7)", nullptr, "%03u",
            [](const ConfigItem& self) {
                print(self.format, qd4310.ID);
            },
            [](const float value) {
                if (!qd4310.setID(value)) {
                    print_len("Invalid CAN ID: %d, must be between 0 and 7", static_cast<int>(value));
                    return false;
                }
                return true;
            }
        },
        {
            "uart.baud_rate", "UART BaudRate of the motor (50K-10M)", "bps", "%u",
            [](const ConfigItem& self) {
                print(self.format, qd4310.uart_baud_rate);
            },
            [](const float value) {
                if (!qd4310.setUartBaudRate(value)) {
                    print_len("Invalid UART baud rate: %d, must be between 10'000 and 10'000'000",
                              static_cast<int>(value));
                    return false;
                }
                print_len("UART baud rate will be set after storing and rebooting");
                return true;
            }
        },
        {
            "zero_pos", "Position zero offset in rad", nullptr, nullptr,
            nullptr,
            [](const float value) {
                if (qd4310.started) {
                    print_len("QDrive is running, please disable it first");
                    return false;
                }
                return qd4310.setZeroPosition(std::isnan(value) ? qd4310.getAngle() : value);
            }
        },
        {
            "can.baud_rate", "CAN bus baud rate (fixed)", "bps", "%u",
            [](const ConfigItem& self) {
                (void)self;
                print("1'000'000");
            },
            nullptr
        },
    };

    static const ConfigItem* find_config_item(const char *key) {
        for (const auto& config_item : ConfigItems) {
            if (strcmp(config_item.name, key) == 0) return &config_item;
        }
        return nullptr;
    }

    static signed short silent(char *data, unsigned short len) { // NOLINT(readability-non-const-parameter)
        (void)data;
        (void)len;
        return 0;
    }

    static void print_version() {
        print_len("Hardware version %s", FOC_HARDWARE_VERSION);
        print_len("Software version %s", FOC_SOFTWARE_VERSION);
    }


    static void foc_info() {
        print_len("Hardware Info:");
        print_len("  Pole pairs       : %d ", FOC_POLE_PAIRS);
        print_len("  KV rating        : %.1f rpm/V", FOC_KV);
        print_len("  Nominal voltage  : %d V", FOC_NOMINAL_VOLTAGE);
        print_len("  Phase inductance : %.2f mH", FOC_PHASE_INDUCTANCE);
        print_len("  Phase resistance : %.2f Ω", FOC_PHASE_RESISTANCE);
        print_len("  Torque constant  : %.2f Nm/A", FOC_TORQUE_CONSTANT);
        print_len("  Max current      : %.2f A", FOC_MAX_CURRENT);
    }

    static void foc_status() {
        print_len("Motor Status:");
        print_len("  CAN ID       : %03d", qd4310.ID);
        print_len("  Status       : %s", qd4310.started ? "enabled" : "disabled");
        print_len("  CtrlMode     : %s",
                  qd4310.getCtrlType() == CtrlType::CurrentCtrl ? "CurrentCtrl" :
                  qd4310.getCtrlType() == CtrlType::SpeedCtrl ? "SpeedCtrl" :
                  qd4310.getCtrlType() == CtrlType::AngleCtrl ? "AngleCtrl" :
                  qd4310.getCtrlType() == CtrlType::StepAngleCtrl ? "StepAngleCtrl" :
                  qd4310.getCtrlType() == CtrlType::LowSpeedCtrl ? "LowSpeedCtrl" : "Unknown");
        print_len("  Current      : %.2f A", qd4310.getCurrent());
        print_len("  Speed        : %.2f rpm", qd4310.getSpeed());
        print_len("  Angle        : %.2f rad", qd4310.getAngle());
        print_len("  Voltage      : %.2f V", qd4310.getVoltage());
    }

    static void foc_config_help() {
        print_len("Usage: config [--list | PARAM_PATH VALUE | key=value]");
        print_len("");
        print_len("Examples:");
        print_len("  config %s 0.1", ConfigItems[0].name);
        print_len("  config %s=0.1", ConfigItems[1].name);
        print_len("  config --help");
        print_len("  config --list");
        print_len("");
        print_len("Configuration Parameters:");
        for (const auto& config_item : ConfigItems) {
            print_len("  %-18s : %s", config_item.name, config_item.description);
        }
    }

    static void foc_config_list() {
        print_len("QDrive Configuration:");
        for (const auto& config_item : ConfigItems) {
            if (config_item.print_value) {
                print("%s = ", config_item.name);
                config_item.print_value(config_item);
                print_len(" %s", config_item.unit ? config_item.unit : "");
            }
        }
    }

    static void foc_config(int argc, char *argv[]) {
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

        const auto *config_item = find_config_item(key);
        if (!config_item || !config_item->set_value) {
            print_len("Unknown config target: %s", key);
            return;
        }
        if (!value && find_config_item("zero_pos") != config_item) {
            print_len("Missing value for config [%s]", key);
            return;
        }
        const float valf = value ? atof_lite(value) : NAN;
        if (!config_item->set_value(valf)) {
            print_len("Config [%s] failed", key);
            return;
        }
        if (std::isnan(valf)) {
            print_len("Config [%s]", key);
        } else {
            print_len("Config [%s] = %.3g", key, valf);
        }
    }

    static void foc_ctrl_help() {
        print_len(
            "Usage: ctrl [current VALUE | low_speed VALUE  | speed VALUE  | step_angle VALUE | angle VALUE | key=value]");
        print_len("");
        print_len("Examples:");
        print_len("  ctrl speed 100");
        print_len("  ctrl speed=100");
        print_len("  ctrl --help");
        print_len("");
        print_len("Control Parameters:");
        print_len("  current           : Set current in Q axis (A)");
        print_len("  low_speed         : Set speed by increasing angle (rpm)");
        print_len("  speed             : Set speed (rpm)");
        print_len("  angle             : Set angle (rad)");
        print_len("  step_angle        : Step an specific angle (rad)");
    }

    static void foc_ctrl(int argc, char *argv[]) {
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
            print_len("Missing value for ctrl [%s]", key);
            return;
        }
        if (!qd4310.started) {
            print_len("QDrive is not running, please enable it first");
            return;
        }
        float valf = atof_lite(value);
        if (strcmp(key, "current") == 0) {
            print_len("Setting current = %.2f A", valf);
            qd4310.Ctrl(CtrlType::CurrentCtrl, valf);
        } else if (strcmp(key, "speed") == 0) {
            print_len("Setting speed = %.2f rpm", valf);
            qd4310.Ctrl(CtrlType::SpeedCtrl, valf);
        } else if (strcmp(key, "angle") == 0) {
            print_len("Setting angle = %.2f rad", valf);
            qd4310.Ctrl(CtrlType::AngleCtrl, valf);
        } else if (strcmp(key, "step_angle") == 0) {
            print_len("Stepping %.2f rad angle", valf);
            qd4310.Ctrl(CtrlType::StepAngleCtrl, valf);
        } else if (strcmp(key, "low_speed") == 0) {
            print_len("Setting low_speed = %.2f rpm", valf);
            qd4310.Ctrl(CtrlType::LowSpeedCtrl, valf);
        } else {
            print_len("Unknown ctrl target: %s", key);
            foc_ctrl_help();
        }
    }

    static void foc_enable() {
        if (qd4310.start()) {
            print_len("QDrive enabled");
        } else if (qd4310.error_code & CalibrationError) {
            print_len("Enable failed, please calibrate first");
        } else if (qd4310.error_code & VoltageError) {
            print_len("Enable failed, voltage error");
        } else {
            print_len("Enable failed, unknown error");
        }
    }

    static void foc_disable() {
        qd4310.stop();
        print_len("QDrive disabled");
    }

    static void foc_calibrate() {
        if (qd4310.started) {
            print_len("QDrive is running, please disable it first");
            return;
        }
        if (qd4310.calibrated) {
            print_len("QDrive already calibrated,do you want to re-calibrate? (y/n)");
            char response;
            while (!shellRead(&response, 1)) {
                delay(1);
            }
            if (response != 'y' && response != 'Y') {
                print_len("Calibration aborted");
                return;
            }
        }
        print_len("Calibration started, please wait...");
        if (const auto status = qd4310.calibrate(); status == CalibrationStatus::Success)
            print_len("Calibration completed");
        else {
            print("Calibration failed: ");
            if (status == CalibrationStatus::EnvironmentError)
                print_len("environment error");
            else if (status == CalibrationStatus::VoltageError)
                print_len("voltage error");
            else if (status == CalibrationStatus::Busy)
                print_len("busy");
            else if (status == CalibrationStatus::CurrentSensorError)
                print_len("current sensor error");
            else if (status == CalibrationStatus::DriverError)
                print_len("driver error");
            else if (status == CalibrationStatus::EncoderError)
                print_len("encoder error");
            else
                print_len("unknown error");
        }
    }

    static void foc_restore() {
        print_len("Are you sure you want to restore factory settings? (y/n)");
        char response;
        while (!shellRead(&response, 1)) {
            delay(1);
        }
        if (response != 'y' && response != 'Y') {
            print_len("Factory restore cancelled");
            return;
        }
        qd4310.restore_calibration(); // 恢复出厂设置
        print_len("QDrive factory restore completed");
        foc_config_list();
    }

    static void foc_store() {
        if (qd4310.started) {
            print_len("QDrive is running, please disable it first");
            return;
        }
        foc_config_list();
        print_len("Are you sure you want to store configurations? (y/n)");
        char response;
        while (!shellRead(&response, 1)) {
            delay(1);
        }
        if (response != 'y' && response != 'Y') {
            print_len("Store operation cancelled");
            return;
        }
        qd4310.freeze_storage_calibration(
            static_cast<StorageStatus>(STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                       STORAGE_LIMIT_OK |         // 储存限制参数
                                       STORAGE_PLUG_OK)           // 储存ID
        );
        print_len("Store configuration completed");
    }

    static void shell_reboot() {
        NVIC_SystemReset();
    }

    static void shell_silent() {
        shell.write = silent; // 禁止输出
    }
};

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    silent, ShellPlugs::shell_silent, "Disable shell output, reboot to enable again"
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    version, ShellPlugs::print_version, Show version info
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    reboot, ShellPlugs::shell_reboot, reboot system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    store, ShellPlugs::foc_store, Store configurations
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    restore, ShellPlugs::foc_restore, Factory restore
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    ctrl, ShellPlugs::foc_ctrl, Set control targets
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    config, ShellPlugs::foc_config, Configure system parameters
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    calibrate, ShellPlugs::foc_calibrate, Calibrate FOC system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable, ShellPlugs::foc_disable, Disable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable, ShellPlugs::foc_enable, Enable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    status, ShellPlugs::foc_status, Show current motor status
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    info, ShellPlugs::foc_info, Show hardware information
);
