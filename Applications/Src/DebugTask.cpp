#include "task_public.h"
#include "task.h"

void StartDebugTask(void *argument) {
    vTaskDelete(nullptr);
    for (;;) {
        delay(1000);
    }
}
