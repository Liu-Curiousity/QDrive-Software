#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "FOC.h"
#include "retarget/retarget.h"

extern FOC foc;

int shell_foc(int argc, char *argv[]) {
    char tx_buffer[50];
    printf("%dparameter(s)\r\n", argc);
    for (char i = 1; i < argc; i++) {
        printf("%s\r\n", argv[i]);
    }
    return 0;
}

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    foc, shell_foc, foc info
);
