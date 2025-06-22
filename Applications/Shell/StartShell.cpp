#include "task_public.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "retarget/retarget.h"
#include "task.h"

Shell shell;
char shellBuffer[256];

void StartStartShell(void *argument) {
    MX_USB_Device_Init();
    delay(100); // 错开RAM使用高峰期
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 256);
    xTaskCreate(shellTask, "LetterShellTask", 128, &shell, osPriorityNormal, nullptr);
    vTaskDelete(nullptr);
}
