/**
 * @file        StartShell.cpp
 * @brief       启动shell
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-5-14
 * @version     V1.1.2
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-6-21
 *		        V1.1.0创建于2025-7-8
 *		        V1.1.1创建于2026-5-6, 使用sizeof()替换定值shell缓冲区大小, 减少误设置风险
 *		        V1.1.2创建于2026-5-14, 增大LetterShell任务的栈空间分配
 * @copyright   (c) 2026 QDrive
 */

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
    delay_ms(200);
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
    delay_ms(300);
}

void StartStartShell(void *argument) {
    USB_Disconnected(); //USB重枚举
    MX_USB_Device_Init();
    delay_ms(100); // 错开RAM使用高峰期
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, sizeof(shellBuffer));
    xTaskCreate(shellTask, "LetterShellTask", 512, &shell, osPriorityNormal, nullptr);
    vTaskDelete(nullptr);
}
