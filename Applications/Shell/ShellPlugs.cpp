#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "retarget/retarget.h"
#include "FOC.h"
#include "FOC_config.h"

extern FOC foc;

// 打印单行
#define PRINT(...)         \
    do {                    \
        printf(__VA_ARGS__); \
        printf("\r\n");      \
    } while (0)

void print_help() {
    PRINT("Usage: foc [COMMAND] [ARGS...]");
    PRINT("");
    PRINT("Main commands:");
    PRINT("  enable             Enable FOC control");
    PRINT("  disable            Disable FOC control");
    PRINT("  info               Show hardware information");
    PRINT("  status             Show current motor status");
    PRINT("  config             Configure system parameters");
    PRINT("  ctrl               Set control targets");
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
    PRINT("  Current      : %.2f A", foc.current());
    PRINT("  Speed        : %.2f rpm", foc.speed());
    PRINT("  Angle        : %.2f rad", foc.angle());
    PRINT("  Voltage      : %.2f V", foc.voltage());
}

void foc_config_help() {
    PRINT("Usage: QDrive config [--list | PARAM_PATH VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  QDrive config --list");
    PRINT("  QDrive config pid.currentQ.kp 0.1");
    PRINT("  QDrive config limit.speed 100");
    PRINT("  QDrive config can.baud_rate 100000");
    PRINT("  QDrive config pid.currentQ.kp=0.1");
}

void foc_config(int argc, char *argv[]) {
    if (argc < 3 || strcmp(argv[2], "--help") == 0) {
        foc_config_help();
        return;
    }

    if (strcmp(argv[2], "--list") == 0) {
        PRINT("Current Configuration:");
        // TODO: 打印所有配置项
        return;
    }

    const char *key = argv[2];
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
    } else if (argc >= 4) {
        value = argv[3];
    }

    if (value) {
        float valf = atof(value);
        PRINT("Setting config [%s] = %.3f", key, valf);
        // TODO: 设置参数
    } else {
        PRINT("Missing value for config [%s]", key);
    }
}

void foc_ctrl_help() {
    PRINT("Usage: QDrive ctrl [currentQ VALUE | speed VALUE | angle VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  QDrive ctrl currentQ 5.0");
    PRINT("  QDrive ctrl speed 1000");
    PRINT("  QDrive ctrl angle 90");
    PRINT("  QDrive ctrl currentQ=5.0");
}

void foc_ctrl(int argc, char *argv[]) {
    if (argc < 3 || strcmp(argv[2], "--help") == 0) {
        foc_ctrl_help();
        return;
    }

    const char *key = argv[2];
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
    } else if (argc >= 4) {
        value = argv[3];
    }

    if (value) {
        float valf = atof(value);
        if (strcmp(key, "currentQ") == 0) {
            PRINT("Setting currentQ = %.2f A", valf);
            foc.Ctrl(FOC::CtrlType::CurrentCtrl, valf);
            // TODO
        } else if (strcmp(key, "speed") == 0) {
            PRINT("Setting speed = %.2f rpm", valf);
            foc.Ctrl(FOC::CtrlType::SpeedCtrl, valf);
            // TODO
        } else if (strcmp(key, "angle") == 0) {
            PRINT("Setting angle = %.2f deg", valf);
            foc.Ctrl(FOC::CtrlType::AngleCtrl, valf);
            // TODO
        } else {
            PRINT("Unknown ctrl target: %s", key);
            foc_ctrl_help();
        }
    } else {
        PRINT("Missing value for ctrl [%s]", key);
    }
}

int shell_foc(int argc, char *argv[]) {
    if (argc < 2) {
        print_help();
        return 0;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "--help") == 0 || strcmp(cmd, "help") == 0 || strcmp(cmd, "QDrive") == 0) {
        print_help();
    } else if (strcmp(cmd, "-v") == 0 || strcmp(cmd, "--version") == 0 || strcmp(cmd, "version") == 0) {
        print_version();
    } else if (strcmp(cmd, "info") == 0) {
        foc_info();
    } else if (strcmp(cmd, "status") == 0) {
        foc_status();
    } else if (strcmp(cmd, "config") == 0) {
        foc_config(argc, argv);
    } else if (strcmp(cmd, "ctrl") == 0) {
        foc_ctrl(argc, argv);
    } else if (strcmp(cmd, "enable") == 0) {
        foc.start();
    } else if (strcmp(cmd, "disable") == 0) {
        foc.stop();
    } else {
        PRINT("Unknown command: %s", cmd);
        print_help();
        return -1;
    }

    return 0;
}

int shell_reboot(int argc, char *argv[]) {
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
