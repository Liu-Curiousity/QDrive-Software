//
// Created by 26757 on 25-5-6.
//
#include <numbers>
#include "task_public.h"
#include "fdcan.h"
#include "FOC.h"
#include "queue.h"
#include "task.h"

using namespace std;

extern FOC foc;

void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan);
void CAN_Transmit(uint8_t length, uint8_t *pdata);
void FeedBackSend();

xQueueHandle xQueue1;

void StartCommunicateTask(void *argument) {
    xQueue1 = xQueueCreate(5, 3);
    // 1.等待foc初始化
    while (!foc.initialized)
        delay(100);
    foc.enable(); // 使能FOC
    foc.anticogging_enabled = false;
    // 2.初始化CAN并开启CAN接收
    FDCAN_Filter_INIT(&hfdcan1);

    uint8_t RxBuffer[3];
    while (true) {
        xQueueReceive(xQueue1, &RxBuffer, portMAX_DELAY);

        switch (RxBuffer[0]) {
            case 0x00: // NOP指令,只发送反馈报文
                break;
            case 0x01: // 使能指令
                foc.start();
                if (foc.started)
                    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
                break;
            case 0x02: // 失能指令
                foc.stop();
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
                break;
            case 0x03: // 电流控制
                foc.Ctrl(FOC::CtrlType::CurrentCtrl,
                         *(int16_t *)(RxBuffer + 1) * 10.0f / INT16_MAX);
                break;
            case 0x04: // 速度控制
                foc.Ctrl(FOC::CtrlType::SpeedCtrl,
                         *(int16_t *)(RxBuffer + 1) * 1000.0f / INT16_MAX);
                break;
            case 0x05: // 角度控制
                foc.Ctrl(FOC::CtrlType::AngleCtrl,
                         *(int16_t *)(RxBuffer + 1) * 2 * numbers::pi_v<float> / UINT16_MAX);
                break;
            case 0x06: // 角度控制
                foc.Ctrl(FOC::CtrlType::LowSpeedCtrl,
                         *(int16_t *)(RxBuffer + 1) * 1000.0f / INT16_MAX);
                break;
            default:
                break;
        }
        if (RxBuffer[0] <= 0x06) {
            // 是合法命令则发送反馈报文
            FeedBackSend();
        }
    }
}

/**
 * @brief CAN外设初始化函数
 * */
void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType = FDCAN_STANDARD_ID;
    Filter.FilterIndex = 0;
    Filter.FilterType = FDCAN_FILTER_RANGE;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1 = 0x400;
    Filter.FilterID2 = 0x40F;

    HAL_FDCAN_ConfigFilter(hfdcan, &Filter);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief CAN接收回调函数
 * */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    BaseType_t xHigherPriorityTaskWoken;
    if (hfdcan == &hfdcan1) {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t FDCAN_RxData[3];
        /*如果FIFO中有数据*/
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
            /*读取数据*/
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN_RxData);
            if (RxHeader.Identifier == 0x400 + foc.ID && RxHeader.DataLength == 3) {
                // 如果是自己ID的报文,进行处理
                xQueueSendToBackFromISR(xQueue1, FDCAN_RxData, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken) {
                    taskYIELD();
                }
            }
        }
    }
}

void CAN_Transmit(uint8_t length, uint8_t *pdata) {
    /*定义CAN数据包头*/
    static FDCAN_TxHeaderTypeDef TxHeader = {
        0x500, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_ESI_ACTIVE,
        FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0
    };
    TxHeader.Identifier = 0x500 + foc.ID;
    TxHeader.DataLength = length;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pdata);
}

void FeedBackSend() {
    static uint8_t FeedBackDataBuffer[8] = {0};
    // 电机状态
    FeedBackDataBuffer[0] = foc.started;
    // 错误码(预留)
    FeedBackDataBuffer[1] = 0x00;
    // Q轴电流
    *(int16_t *)(FeedBackDataBuffer + 2) = foc.getCurrent() / 10 * INT16_MAX;
    // 电机转速
    *(int16_t *)(FeedBackDataBuffer + 4) = foc.getSpeed() / 5000 * INT16_MAX;
    // 电机角度
    *(int16_t *)(FeedBackDataBuffer + 6) = foc.getAngle() / (2 * numbers::pi_v<float>) * UINT16_MAX;
    CAN_Transmit(8, FeedBackDataBuffer);
}
