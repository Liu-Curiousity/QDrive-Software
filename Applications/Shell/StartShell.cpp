#include "task_public.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "retarget/retarget.h"
#include "task.h"

Shell shell;
char shellBuffer[256];

void USB_Disconnected() {
    __HAL_RCC_USB_FORCE_RESET();
    HAL_Delay(200);
    __HAL_RCC_USB_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(300);
}

void StartStartShell(void *argument) {
    USB_Disconnected(); //USB重枚举
    MX_USB_Device_Init();
    delay(100); // 错开RAM使用高峰期
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 256);
    xTaskCreate(shellTask, "LetterShellTask", 128, &shell, osPriorityNormal, nullptr);
    vTaskDelete(nullptr);
}
