#include "cmsis_os.h"
#include "task_public.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "usbd_cdc_if.h"

/**
 * @brief shell写数据函数原型
 *
 * @param data 需写的字符数据
 * @param len 需要写入的字符数
 *
 * @return unsigned short 实际写入的字符数量
 */
signed short shellWrite(char *data, unsigned short len) {
    CDC_Transmit_FS(reinterpret_cast<uint8_t *>(data), len);
    delay(1);
    return 0;
}

Shell shell;
char shellBuffer[512];

void App_DebugTask(void *argument) {
    MX_USB_Device_Init();

    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 512);
    xTaskCreate(shellTask, "LetterShellTask", 128, &shell, osPriorityNormal, nullptr);
    for (;;) {
        delay(1000);
    }
}
