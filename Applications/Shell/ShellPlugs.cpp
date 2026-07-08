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

#define PROMPT_DISABLE_FIRST "QDrive is running, please disable it first"
#define PROMPT_ENABLE_FIRST "QDrive is not running, please enable it first"
#define PROMPT_UNKNOW_TARGET(cmd, key) "Unknown "#cmd" target: %s", key
#define PROMPT_MISSING_VALUE_FOR(cmd, key) ("Missing value for "#cmd" [%s]", key)

class ShellPlugs : public QD4310 {
public:
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
        print_len("  CtrlMode     : %s ctrl",
                  qd4310.getCtrlType() == CtrlType::CurrentCtrl ? CtrlItems[0].name :
                  qd4310.getCtrlType() == CtrlType::SpeedCtrl ? CtrlItems[1].name :
                  qd4310.getCtrlType() == CtrlType::AngleCtrl ? CtrlItems[2].name :
                  qd4310.getCtrlType() == CtrlType::StepAngleCtrl ? CtrlItems[3].name :
                  qd4310.getCtrlType() == CtrlType::LowSpeedCtrl ? CtrlItems[4].name : "Unknown");
        print_len("  Current      : %.2f A", qd4310.getCurrent());
        print_len("  Speed        : %.2f rpm", qd4310.getSpeed());
        print_len("  Angle        : %.2f rad", qd4310.getAngle());
        print_len("  Voltage      : %.2f V", qd4310.getVoltage());
    }

    static void foc_config_help() {
        print_len("Usage: config [--list | CONFIG_PARAM VALUE | key=value]");
        print_len("");
        print_len("Examples:");
        print_len("  config %s 0.1", ConfigItems[0].name);
        print_len("  config %s=0.1", ConfigItems[1].name);
        print_len("  config --help");
        print_len("  config --list");
        print_len("");
        print_len("Configuration Parameters:");
        for (const auto& item : ConfigItems) {
            print_len("  %-18s : %s", item.name, item.description);
        }
    }

    static void foc_config_list() {
        print_len("QDrive Configuration:");
        for (const auto& item : ConfigItems) {
            if (item.print_value) {
                print("%s = ", item.name);
                item.print_value(item);
                print_len(" %s", item.unit ? item.unit : "");
            }
        }
    }

    static void foc_config(const int argc, char *argv[]) {
        if (argc < 2 || strcmp(argv[1], "--help") == 0) {
            foc_config_help();
            return;
        }

        if (strcmp(argv[1], "--list") == 0) {
            foc_config_list();
            return;
        }

        char *key = argv[1];
        const char *value = parse_key_value_arg(key);
        if (!value && argc >= 3) {
            value = argv[2];
        }

        const auto *config_item = Item::find_item(ConfigItems, key);
        if (!config_item || !config_item->set_value) {
            print_len(PROMPT_UNKNOW_TARGET(config, key));
            return;
        }
        if (!value && Item::find_item(ConfigItems, "zero_pos") != config_item) {
            print_len(PROMPT_MISSING_VALUE_FOR(config, key));
            return;
        }
        const float valf = value ? atof_lite(value) : NAN;
        if (!config_item->set_value(valf)) {
            print_len("Config [%s] failed", key);
            return;
        }
        print("Config [%s] = ", key);
        if (!std::isnan(valf)) print(config_item->format, valf);
        print_len("");
    }

    static void foc_ctrl_help() {
        print_len(
            "Usage: ctrl [CONFIG_PARAM VALUE | key=value]");
        print_len("");
        print_len("Examples:");
        print_len("  ctrl %s 100", CtrlItems[1].name);
        print_len("  ctrl %s=100", CtrlItems[1].name);
        print_len("  ctrl --help");
        print_len("");
        print_len("Control Parameters:");
        for (const auto& item : CtrlItems) {
            print_len("  %-18s : %s (%s)", item.name, item.description, item.unit);
        }
    }

    static void foc_ctrl(const int argc, char *argv[]) {
        if (argc < 2 || strcmp(argv[1], "--help") == 0) {
            foc_ctrl_help();
            return;
        }

        char *key = argv[1];
        const char *value = parse_key_value_arg(key);
        if (!value && argc >= 3)
            value = argv[2];

        if (!qd4310.started) {
            print_len(PROMPT_ENABLE_FIRST);
            return;
        }
        const auto *ctrl_item = Item::find_item(CtrlItems, key);
        if (!ctrl_item || !ctrl_item->set_value) {
            print_len(PROMPT_UNKNOW_TARGET(ctrl, key));
            return;
        }
        if (!value) {
            print_len(PROMPT_MISSING_VALUE_FOR(ctrl, key));
            return;
        }
        const float valf = atof_lite(value);
        if (!ctrl_item->set_value(valf)) {
            print_len("Ctrl [%s] failed", key);
            return;
        }
        print("Setting [%s] = ", key);
        print_len(ctrl_item->format, valf);
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
            print_len(PROMPT_DISABLE_FIRST);
            return;
        }
        if (qd4310.calibrated) {
            print_len("QDrive already calibrated,do you want to re-calibrate? (y/n)");
            char response;
            while (!shellRead(&response, 1)) {
                delay(1);
            }
            if (response != 'y' && response != 'Y') {
                print_len("Calibration cancelled");
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
        if (qd4310.started) {
            print_len(PROMPT_DISABLE_FIRST);
            return;
        }
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
        print_len("Factory restore completed");
        foc_config_list();
    }

    static void foc_store() {
        if (qd4310.started) {
            print_len(PROMPT_DISABLE_FIRST);
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
        print_len("Store operation completed");
    }

    static void shell_reboot() {
        NVIC_SystemReset();
    }

    static void shell_silent() {
        shell.write = silent; // 禁止输出
    }

private:
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

    static signed short silent(char *data, unsigned short len) {
        (void)data;
        (void)len;
        return 0;
    }

    // 解析 key=value 格式
    static char* parse_key_value_arg(char *arg) {
        if (!arg) return nullptr;
        char *eq = strchr(arg, '=');
        if (!eq) return nullptr;
        *eq = '\0';
        return eq + 1;
    }

    class Item {
    public:
        const char *name;
        const char *description;
        const char *unit;
        const char *format;
        void (*print_value)(const Item&);
        bool (*set_value)(float);

        template <size_t N>
        static const Item* find_item(const Item (&items)[N], const char *key) {
            return find_item(items, N, key);
        }

        static const Item* find_item(const Item items[], const size_t N, const char *key) {
            for (size_t i = 0; i < N; ++i) {
                if (strcmp(items[i].name, key) == 0) return &items[i];
            }
            return nullptr;
        }
    };

    inline static const Item ConfigItems[] = {
        {
            "pid.speed.kp", "Speed PID proportional gain", nullptr, "%.3g",
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
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
            [](const Item& self) {
                print(self.format, qd4310.ID);
            },
            [](const float value) {
                if (!qd4310.setID(static_cast<uint8_t>(value))) {
                    print_len("Invalid CAN ID: %d, must be between 0 and 7", static_cast<int>(value));
                    return false;
                }
                return true;
            }
        },
        {
            "uart.baud_rate", "UART BaudRate of the motor (50K-10M)", "bps", "%u",
            [](const Item& self) {
                print(self.format, qd4310.uart_baud_rate);
            },
            [](const float value) {
                if (!qd4310.setUartBaudRate(static_cast<uint32_t>(value))) {
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
                    print_len(PROMPT_DISABLE_FIRST);
                    return false;
                }
                return qd4310.setZeroPosition(std::isnan(value) ? qd4310.getAngle() : value);
            }
        },
        {
            "can.baud_rate", "CAN bus baud rate (fixed)", "bps", "%u",
            [](const Item& self) {
                (void)self;
                print("1'000'000");
            },
            nullptr
        },
    };

    inline static const Item CtrlItems[] = {
        {
            "current", "Set current in Q axis", "A", "%.3g",
            nullptr,
            [](const float value) {
                qd4310.Ctrl(CtrlType::CurrentCtrl, value);
                return true;
            }
        },
        {
            "speed", "Set speed", "rpm", "%.3g",
            nullptr,
            [](const float value) {
                qd4310.Ctrl(CtrlType::SpeedCtrl, value);
                return true;
            }
        },
        {
            "angle", "Set angle", "rad", "%.3g",
            nullptr,
            [](const float value) {
                qd4310.Ctrl(CtrlType::AngleCtrl, value);
                return true;
            }
        },
        {
            "step_angle", "Step an specific angle", "rad", "%.3g",
            nullptr,
            [](const float value) {
                qd4310.Ctrl(CtrlType::StepAngleCtrl, value);
                return true;
            }
        },
        {
            "low_speed", "Set speed by increasing angle", "rpm", "%.3g",
            nullptr,
            [](const float value) {
                qd4310.Ctrl(CtrlType::LowSpeedCtrl, value);
                return true;
            }
        },
    };
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
