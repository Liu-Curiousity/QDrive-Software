#include "cmsis_os.h"
#include "task_public.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "CharCircularQueue.h"

Shell shell;
char shellBuffer[512];
CharCircularQueue char_queue{128};

extern "C" void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len) {
    auto length = *Len;
    while (length--) char_queue.enqueue(*(Buf++));
}

/**
 * @brief shell读取数据函数原型
 *
 * @param data shell读取的字符
 * @param len 请求读取的字符数量
 *
 * @return unsigned short 实际读取到的字符数量
 */
signed short shellRead(char *data, unsigned short len) {
    signed short i = 0;
    for (i = 0; i < len && !char_queue.isEmpty(); ++i, ++data) {
        char_queue.dequeue(*data);
    }
    return i;
}

/**
 * @brief shell写数据函数原型
 *
 * @param data 需写的字符数据
 * @param len 需要写入的字符数
 *
 * @return unsigned short 实际写入的字符数量
 */
signed short shellWrite(char *data, unsigned short len) {
    const auto start_tick = xTaskGetTickCount();
    while (HAL_OK != CDC_Transmit_FS(reinterpret_cast<uint8_t *>(data), len))
        if (xTaskGetTickCount() - start_tick > 100) break;
    return 0;
}

void App_DebugTask(void *argument) {
    MX_USB_Device_Init();

    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 512);
    xTaskCreate(shellTask, "LetterShellTask", 128, &shell, osPriorityNormal, nullptr);
    for (;;) {
        delay(1000);
    }
}
