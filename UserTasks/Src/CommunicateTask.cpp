//
// Created by 26757 on 25-5-6.
//
#include <numbers>

#include "task_public.h"
#include "fdcan.h"
#include "FOC.h"
#include "Storage_EmbeddedFlash.h"

using namespace std;

extern FOC foc;
uint8_t ID = 0;
bool calibrating = false;
bool anticogging_calibrating = false;

uint8_t storage_status;

void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan);
void CAN_Transmit(uint8_t length, uint8_t *pdata);
void FeedBackSend();

void StartCommunicateTask(void *argument) {
    // 1.等待foc初始化
    while (!foc.initialized)
        delay(100);
    foc.enable(); // 使能FOC
    // 2.从flash中读取ID
    storage.read(0x700, &storage_status, sizeof(storage_status));
    if (storage_status == 0xAA) {
        // 0xAA 表示ID已经储存
        storage.read(0x720, &ID, sizeof(ID));
    }
    // 3.初始化CAN并开启CAN接收
    FDCAN_Filter_INIT(&hfdcan1);

    while (true) {
        if (calibrating) {
            if (foc.calibrated) calibrating = false;
        }
        if (anticogging_calibrating) {
            if (foc.anticogging_calibrated) anticogging_calibrating = false;
        }
        delay(100);
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
    if (hfdcan == &hfdcan1) {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t FDCAN_RxData[8];
        /*如果FIFO中有数据*/
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
            /*读取数据*/
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN_RxData);
            if (RxHeader.Identifier == 0x400 + ID && RxHeader.DataLength == 8) {
                // 如果是自己ID的报文,进行处理
                if (calibrating || anticogging_calibrating) {
                    // 如果正在校准,只发送反馈报文
                    FeedBackSend();
                } else {
                    switch (FDCAN_RxData[0]) {
                        case 0x00: // NOP指令,只发送反馈报文
                            FeedBackSend();
                            break;
                        case 0x01: // 使能指令
                            foc.start();
                            FeedBackSend();
                            break;
                        case 0x02: // 失能指令
                            foc.stop();
                            FeedBackSend();
                            break;
                        case 0x03: // 基础校准
                            foc.calibrate();
                            calibrating = true;
                            break;
                        case 0x04: // 齿槽转矩校准
                            foc.anticogging_calibrate();
                            anticogging_calibrating = true;
                            break;
                        case 0x05: // 运动控制
                            switch (FDCAN_RxData[1]) {
                                case 0x00: // 电流控制
                                    foc.Ctrl(FOC::CtrlType::CurrentCtrl,
                                             *(int16_t *)(FDCAN_RxData + 2) * 10.0f / INT16_MAX);
                                    break;
                                case 0x01: // 速度控制
                                    foc.Ctrl(FOC::CtrlType::SpeedCtrl,
                                             *(int16_t *)(FDCAN_RxData + 2) * 5000.0f / INT16_MAX);
                                    break;
                                case 0x02: // 角度控制
                                    foc.Ctrl(FOC::CtrlType::PositionCtrl,
                                             *(int16_t *)(FDCAN_RxData + 2) * 2 * numbers::pi_v<float> / UINT16_MAX);
                                    break;
                                default:
                                    break;
                            }
                            break;
                        case 0x06: // 设置ID
                            ID = FDCAN_RxData[7];
                            storage.write(0x720, &ID, sizeof(ID));
                            storage_status = 0xAA;
                            storage.write(0x700, &storage_status, sizeof(storage_status));
                            break;
                        default:
                            break;
                    }
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
    TxHeader.Identifier = 0x500 + ID;
    TxHeader.DataLength = length;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pdata);
}

void FeedBackSend() {
    static uint8_t FeedBackDataBuffer[8];
    // 电机状态
    FeedBackDataBuffer[0] = foc.started |
                            (calibrating || anticogging_calibrating) << 1 |
                            foc.calibrated << 2 |
                            foc.anticogging_calibrated << 3 |
                            foc.anticogging_enabled << 4;
    // 错误码(预留)
    FeedBackDataBuffer[1] = 0x00;
    // Q轴电流
    *(int16_t *)(FeedBackDataBuffer + 2) = foc.current() / 10 * INT16_MAX;
    // 电机转速
    *(int16_t *)(FeedBackDataBuffer + 4) = foc.speed() / 5000 * INT16_MAX;
    // 电机角度
    *(int16_t *)(FeedBackDataBuffer + 6) = foc.angle() / (2 * numbers::pi_v<float>) * UINT16_MAX;
    CAN_Transmit(8, FeedBackDataBuffer);
}
