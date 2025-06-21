#include "task_public.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "CharCircularQueue.h"
#include "task.h"

Shell shell;
char shellBuffer[128];
CharCircularQueue char_queue{128};

extern "C" void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len) {
    auto length = *Len;
    while (length--) char_queue.enqueue(*(Buf++));
}

signed short shellRead(char *data, unsigned short len) {
    signed short i = 0;
    for (i = 0; i < len && !char_queue.isEmpty(); ++i, ++data) {
        char_queue.dequeue(*data);
    }
    delay(1); // 延时1ms,因为shellTask是死循环一点delay都没有,为了让IDLE线程能够运行以释放内存等
    return i;
}

signed short shellWrite(char *data, unsigned short len) {
    const auto start_tick = xTaskGetTickCount();
    while (HAL_OK != CDC_Transmit_FS(reinterpret_cast<uint8_t *>(data), len))
        if (xTaskGetTickCount() - start_tick > 100) break;
    return 0;
}

void StartStartShell(void *argument) {
    MX_USB_Device_Init();
    delay(100); // 错开RAM使用高峰期
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 128);
    xTaskCreate(shellTask, "LetterShellTask", 128, &shell, osPriorityNormal, nullptr);
    vTaskDelete(nullptr);
}
