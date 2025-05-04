#include "task_public.h"
#include "FOC.h"

extern FOC foc;

void App_DebugTask(void *argument) {
    while (!foc.initialized)
        delay(100);

    // 1.使能电机
    foc.enable(); // 启动FOC
    // 2.基础校准
    if (!foc.calibrated) {
        foc.calibration(); // 校准FOC
    }
    // 3.齿槽转矩校准
    if (!foc.anticogging_calibrated) {
        foc.anticogging_calibration(); // 齿槽转矩校准
    }
    foc.anticogging_enabled = true;

    delay(2000);
    foc.start(); // 启动FOC
    // foc.Ctrl(FOC::CtrlType::PositionCtrl, M_PI_2); //设置目标位置
    // foc.Ctrl(FOC::CtrlType::SpeedCtrl, 30);
    foc.Ctrl(FOC::CtrlType::CurrentCtrl, 0.026);
    for (;;) {
        delay(1000);
    }
}
