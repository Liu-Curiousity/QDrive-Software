#pragma once

#include "FOC_config.h"

#define     SHELL_TITLE                 \
            "  ____    ____          _           \r\n"\
            " / __ \\  |  _ \\  _ __ (_)__   ___   \r\n"\
            "| |  | | | | | || '__|| \\ \\ / / _ \\ \r\n"\
            "| |__| | | |_| || |   | |\\ V /  __/ \r\n"\
            " \\___\\_\\ |____/ |_|   |_| \\_/ \\___| \r\n"
#define     SHELL_VERSION               FOC_SOFTWARE_VERSION
#define     SHELL_GET_TICK()            HAL_GetTick()
#define     SHELL_DEFAULT_USER          "QDrive"