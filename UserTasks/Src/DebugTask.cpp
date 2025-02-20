#include "task_public.h"
#include "FOC.h"

extern FOC foc;

#include "fdcan.h"

void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan);

int16_t Angle;
int16_t Speed;

void App_DebugTask(void *argument) {
    FDCAN_Filter_INIT(&hfdcan1);
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
        delay(10);
    }
}

/**
 * @brief CAN外设初始化函数
 * */
void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType = FDCAN_STANDARD_ID;
    Filter.FilterIndex = 0;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1 = 0x00000000;
    Filter.FilterID2 = 0x00000000;

    // Filter.FilterMode = CAN_FILTERMODE_IDMASK;      //掩码模式过滤器
    // Filter.FilterActivation = CAN_FILTER_ENABLE;    //使能过滤器
    // Filter.FilterScale = CAN_FILTERSCALE_32BIT;     //32位过滤器
    // Filter.FilterBank = 0;                          //设置过滤器编号
    // Filter.SlaveStartFilterBank = 0;                //设置从机过滤器编号
    // Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; //使用FIFO0
    // /*一点都不过滤*/
    // Filter.FilterIdHigh = 0x0000;
    // Filter.FilterIdLow = 0x0000;
    // Filter.FilterMaskIdHigh = 0x0000;
    // Filter.FilterMaskIdLow = 0x0000;

    HAL_FDCAN_ConfigFilter(hfdcan, &Filter);
    // HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
}

/**
 * @brief CAN接收回调函数
 * */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    if (hfdcan==&hfdcan1){
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t FDCAN_RxData[8];
        /*如果FIFO中有数据*/
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0))
            /*读取数据*/
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN_RxData);
            Angle = (int16_t) (FDCAN_RxData[0] << 8 | FDCAN_RxData[1]);
            Speed = (int16_t) (FDCAN_RxData[2] << 8 | FDCAN_RxData[3]);
    }
}
