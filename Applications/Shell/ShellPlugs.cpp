#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "retarget/retarget.h"
#include "FOC.h"
#include "FOC_config.h"
#include "main.h"

extern FOC foc;

// 打印单行
#define PRINT(...)         \
    do {                    \
        printf(__VA_ARGS__); \
        printf("\r\n");      \
    } while (0)

void print_help() {
    PRINT("Usage: QDrive [COMMAND] [ARGS...]");
    PRINT("");
    PRINT("Main commands:");
    PRINT("  info               Show hardware information");
    PRINT("  status             Show current motor status");
    PRINT("  enable             Enable FOC control");
    PRINT("  disable            Disable FOC control");
    PRINT("  calibrate          Calibrate FOC system");
    PRINT("  config             Configure system parameters");
    PRINT("  ctrl               Set control targets");
    PRINT("  restore            Factory restore");
    PRINT("  store              Store configurations");
    PRINT("  --help             Show this help message");
    PRINT("  -v, --version      Show version info");
}

void print_version() {
    PRINT("QDrive version %s", FOC_VERSION);
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
    PRINT("  Status       : %s", foc.started ? "enabled" : "disabled");
    PRINT("  CtrlMode     : %s",
          foc.getCtrlType() == FOC::CtrlType::CurrentCtrl ? "CurrentCtrl" :
          foc.getCtrlType() == FOC::CtrlType::SpeedCtrl ? "SpeedCtrl" : "AngleCtrl");
    PRINT("  Current      : %.2f A", foc.getCurrent());
    PRINT("  Speed        : %.2f rpm", foc.getSpeed());
    PRINT("  Angle        : %.2f rad", foc.getAngle());
    PRINT("  Voltage      : %.2f V", foc.getVoltage());
}

void foc_config_help() {
    PRINT("Usage: QDrive config [--list | PARAM_PATH VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  QDrive config pid.speed.kp 0.1");
    PRINT("  QDrive config pid.speed.ki=0.1");
    PRINT("  QDrive config --help");
    PRINT("  QDrive config --list");
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

    // TODO:部分不可调
    // PRINT("  can.baud_rate      : CAN bus baud rate");
}

void foc_config_list() {
    PRINT("Current Configuration:");
    if (foc.PID_Speed.kp == 0)
        PRINT("pid.speed.kp = 0.000");
    else
        PRINT("pid.speed.kp = %.3g", foc.PID_Speed.kp);
    if (foc.PID_Speed.ki == 0)
        PRINT("pid.speed.ki = 0.000");
    else
        PRINT("pid.speed.ki = %.3g", foc.PID_Speed.ki);
    if (foc.PID_Speed.kd == 0)
        PRINT("pid.speed.kd = 0.000");
    else
        PRINT("pid.speed.kd = %.3g", foc.PID_Speed.kd);
    if (foc.PID_Angle.kp == 0)
        PRINT("pid.angle.kp = 0.000");
    else
        PRINT("pid.angle.kp = %.3g", foc.PID_Angle.kp);
    if (foc.PID_Angle.ki == 0)
        PRINT("pid.angle.ki = 0.000");
    else
        PRINT("pid.angle.ki = %.3g", foc.PID_Angle.ki);
    if (foc.PID_Angle.kd == 0)
        PRINT("pid.angle.kd = 0.000");
    else
        PRINT("pid.angle.kd = %.3g", foc.PID_Angle.kd);
    if (std::isnan(foc.PID_Angle.output_limit_p))
        PRINT("limit.speed = no limit");
    else
        PRINT("limit.speed = %.3g rpm", foc.PID_Angle.output_limit_p);
    if (std::isnan(foc.PID_Speed.output_limit_p))
        PRINT("limit.current = no limit");
    else
        PRINT("limit.current = %.3g A", foc.PID_Speed.output_limit_p);
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
    const char *value = NULL;

    if (strchr(key, '=') != NULL) {
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
        float valf = atof(value);
        do {
            if (strcmp(key, "pid.speed.kp") == 0) {
                foc.setPID(valf,NAN,NAN,NAN,NAN,NAN);
            } else if (strcmp(key, "pid.speed.ki") == 0) {
                foc.setPID(NAN, valf,NAN,NAN,NAN,NAN);
            } else if (strcmp(key, "pid.speed.kd") == 0) {
                foc.setPID(NAN,NAN, valf,NAN,NAN,NAN);
            } else if (strcmp(key, "pid.angle.kp") == 0) {
                foc.setPID(NAN,NAN,NAN, valf,NAN,NAN);
            } else if (strcmp(key, "pid.angle.ki") == 0) {
                foc.setPID(NAN,NAN,NAN, NAN, valf,NAN);
            } else if (strcmp(key, "pid.angle.kd") == 0) {
                foc.setPID(NAN,NAN,NAN, NAN,NAN, valf);
            } else if (strcmp(key, "limit.speed") == 0) {
                foc.setLimit(valf, NAN);
            } else if (strcmp(key, "limit.current") == 0) {
                foc.setLimit(NAN, valf);
            } else {
                PRINT("Unknown config target: %s", key);
                break;
            }
            PRINT("Setting config [%s] = %.3g", key, valf);
        } while (false);
    } else {
        PRINT("Missing value for config [%s]", key);
    }
}

void foc_ctrl_help() {
    PRINT("Usage: QDrive ctrl [currentQ VALUE | speed VALUE | angle VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  QDrive ctrl speed 100");
    PRINT("  QDrive ctrl speed=100");
    PRINT("  QDrive ctrl --help");
    PRINT("");
    PRINT("Control Parameters:");
    PRINT("  currentQ          : Set current in Q axis (A)");
    PRINT("  speed             : Set speed (rpm)");
    PRINT("  angle             : Set angle (rad)");
}

void foc_ctrl(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        foc_ctrl_help();
        return;
    }

    const char *key = argv[1];
    const char *value = NULL;

    if (strchr(key, '=') != NULL) {
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
        float valf = atof(value);
        if (strcmp(key, "current") == 0) {
            PRINT("Setting current = %.2f A", valf);
            foc.Ctrl(FOC::CtrlType::CurrentCtrl, valf);
        } else if (strcmp(key, "speed") == 0) {
            PRINT("Setting speed = %.2f rpm", valf);
            foc.Ctrl(FOC::CtrlType::SpeedCtrl, valf);
        } else if (strcmp(key, "angle") == 0) {
            PRINT("Setting angle = %.2f rad", valf);
            foc.Ctrl(FOC::CtrlType::AngleCtrl, valf);
        } else {
            PRINT("Unknown ctrl target: %s", key);
            foc_ctrl_help();
        }
    } else {
        PRINT("Missing value for ctrl [%s]", key);
    }
}

void foc_enable() {
    foc.start();
    if (foc.started) {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        PRINT("QDrive enabled");
    } else
        PRINT("enable failed, please calibrate first");
}

void foc_disable() {
    foc.stop();
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    PRINT("QDrive disabled");
}

void foc_calibrate() {
    if (foc.started) {
        PRINT("QDrive is running, please disable it first");
        return;
    }
    if (foc.calibrated) {
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
    foc.calibrate();
    if (foc.calibrated)
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
    foc.setPID(FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_KD,
               FOC_ANGLE_KP, FOC_ANGLE_KI, FOC_ANGLE_KD);
    foc.setLimit(FOC_MAX_SPEED,FOC_MAX_CURRENT);

    foc.freeze_storage_calibration(FOC::STORAGE_PID_PARAMETER_OK); //储存PID参数
    foc.freeze_storage_calibration(FOC::STORAGE_LIMIT_OK);         //储存限制参数
    PRINT("QDrive factory restore completed");
    foc_config_list();
}

void foc_store() {
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
    foc.freeze_storage_calibration(FOC::STORAGE_PID_PARAMETER_OK); //储存PID参数
    foc.freeze_storage_calibration(FOC::STORAGE_LIMIT_OK);         //储存限制参数
    PRINT("Store configuration completed");
}

int shell_foc(int argc, char *argv[]) {
    if (argc < 2) {
        print_help();
        return 0;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "--help") == 0 || strcmp(cmd, "help") == 0) {
        print_help();
    } else if (strcmp(cmd, "-v") == 0 || strcmp(cmd, "--version") == 0 || strcmp(cmd, "version") == 0) {
        print_version();
    } else if (strcmp(cmd, "enable") == 0) {
        foc_enable();
    } else if (strcmp(cmd, "disable") == 0) {
        foc_disable();
    } else if (strcmp(cmd, "info") == 0) {
        foc_info();
    } else if (strcmp(cmd, "status") == 0) {
        foc_status();
    } else if (strcmp(cmd, "config") == 0) {
        foc_config(argc - 1, argv + 1);
    } else if (strcmp(cmd, "ctrl") == 0) {
        foc_ctrl(argc - 1, argv + 1);
    } else if (strcmp(cmd, "calibrate") == 0) {
        foc_calibrate();
    } else if (strcmp(cmd, "restore") == 0) {
        foc_restore();
    } else if (strcmp(cmd, "store") == 0) {
        foc_store();
    } else {
        PRINT("Unknown command: %s", cmd);
        print_help();
        return -1;
    }

    return 0;
}

int shell_reboot(int argc, char *argv[]) {
    UNUSED(argc);
    UNUSED(argv);
    NVIC_SystemReset();
}

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    reboot, shell_reboot, reboot system
);

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    QDrive, shell_foc, QDrive command interface motor ctrl
);
