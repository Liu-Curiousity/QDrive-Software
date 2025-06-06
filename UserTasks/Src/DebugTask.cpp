#include "task_public.h"
#include "usb_device.h"

void App_DebugTask(void *argument) {
    MX_USB_Device_Init();
    for (;;) {
        delay(10);
    }
}
