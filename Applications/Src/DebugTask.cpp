/**
 * @file        DebugTask.cpp
 * @brief       Debug任务
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2025-6-22
 * @version     V1.0.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-6-22
 * @copyright   (c) 2025 QDrive
 */

#include "task_public.h"
#include "task.h"

void StartDebugTask(void *argument) {
    vTaskDelete(nullptr);
    for (;;) {
        delay(1000);
    }
}
