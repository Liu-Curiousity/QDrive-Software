#include <algorithm>

#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "retarget/retarget.h"
#include "QD4310.h"
#include "FOC_config.h"
#include "main.h"

extern QD4310 qd4310;
extern Shell shell;


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
          qd4310.getCtrlType() == QD4310::CtrlType::SpeedCtrl ? "SpeedCtrl" : "AngleCtrl");
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
    if (std::isnan(qd4310.PID_Angle.output_limit_p))
        PRINT("limit.speed = no limit");
    else
        PRINT("limit.speed = %.3g rpm", qd4310.PID_Angle.output_limit_p);
    if (std::isnan(qd4310.PID_Speed.output_limit_p))
        PRINT("limit.current = no limit");
    else
        PRINT("limit.current = %.3g A", qd4310.PID_Speed.output_limit_p);
    PRINT("can.id = %03d", qd4310.ID);
    // TODO: 波特率不可更改
    PRINT("can.baud_rate = 1'000'000");
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
        qd4310.setZeroPosition(value ? atoff(value) : qd4310.getAngle());
        PRINT("Setting config [zero_pos]");
    } else if (value) {
        float valf = atoff(value);
        if (strcmp(key, "pid.speed.kp") == 0) {
            qd4310.setPID(valf,NAN,NAN,NAN,NAN,NAN);
        } else if (strcmp(key, "pid.speed.ki") == 0) {
            qd4310.setPID(NAN, valf,NAN,NAN,NAN,NAN);
        } else if (strcmp(key, "pid.speed.kd") == 0) {
            qd4310.setPID(NAN,NAN, valf,NAN,NAN,NAN);
        } else if (strcmp(key, "pid.angle.kp") == 0) {
            qd4310.setPID(NAN,NAN,NAN, valf,NAN,NAN);
        } else if (strcmp(key, "pid.angle.ki") == 0) {
            qd4310.setPID(NAN,NAN,NAN, NAN, valf,NAN);
        } else if (strcmp(key, "pid.angle.kd") == 0) {
            qd4310.setPID(NAN,NAN,NAN, NAN,NAN, valf);
        } else if (strcmp(key, "limit.speed") == 0) {
            qd4310.setLimit(valf, NAN);
        } else if (strcmp(key, "limit.current") == 0) {
            qd4310.setLimit(NAN, valf);
        } else if (strcmp(key, "can.id") == 0) {
            qd4310.ID = static_cast<uint8_t>(std::clamp(static_cast<int>(valf), 0, 7));
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

    if (value) {
        float valf = atoff(value);
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
    } else {
        PRINT("Missing value for ctrl [%s]", key);
    }
}

void foc_enable() {
    qd4310.start();
    if (qd4310.started) {
        PRINT("QDrive enabled");
    } else
        PRINT("enable failed, please calibrate first");
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
    qd4310.calibrate();
    if (qd4310.calibrated)
        PRINT("QDrive calibration completed");
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
    qd4310.setPID(FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_KD,
                  FOC_ANGLE_KP, FOC_ANGLE_KI, FOC_ANGLE_KD);
    qd4310.setLimit(FOC_MAX_SPEED,FOC_MAX_CURRENT);
    qd4310.ID = 0;

    qd4310.freeze_storage_calibration(
        static_cast<QD4310::StorageStatus>(QD4310::STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                           QD4310::STORAGE_LIMIT_OK |         // 储存限制参数
                                           QD4310::STORAGE_ID_OK)             // 储存ID
    );
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
                                           QD4310::STORAGE_ID_OK)             // 储存ID
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
