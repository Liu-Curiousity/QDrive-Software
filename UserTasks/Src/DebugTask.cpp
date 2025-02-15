#include "sys_public.h"
#include "task_public.h"

#include "usart.h"

void App_DebugTask(void *argument) {
    ///1.串口重定向
    RetargetInit(&huart3);

    for (;;) {

        delay(1000);
    }
}
