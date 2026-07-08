/**
 * @file        CommunicateTask.cpp
 * @brief       通信任务
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-7-2
 * @version     V1.4.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-5-6
 *		        V1.1.0创建于2025-8-6, 添加低速控制
 *		        V1.1.0创建于2025-12-18, 添加角度步进控制
 *		        V1.1.1创建于2025-12-27, 适配新的QD4310类接口
 *		        V1.2.0创建于2026-3-9, 使用union联合替代数组实现命令解析,添加UART通信接口支持
 *		        V1.2.1创建于2026-3-12, 修复CAN通信失效的问题
 *		        V1.2.2创建于2026-5-4, 修复CAN通信发送超过指令长度帧可能导致的溢出问题
 *		        V1.2.3创建于2026-5-5, 优化上电初始化后通信开启逻辑
 *		        V1.3.0创建于2026-5-14, 添加通过Uart/Can设置零点和重启设备
 *		        V1.4.0创建于2026-7-2, 收到重启命令后先发送反馈报文再执行重启
 *		                             反馈报文添加控制状态反馈和错误码反馈
 * @copyright   (c) 2026 QDrive
 */

#include <algorithm>

#include "task_public.h"
#include "fdcan.h"
#include "usart.h"
#include "QD4310.h"
#include <numbers>

#include "FreeRTOS.h"
#include "queue.h"

using namespace std;

class RxCommand {
public:
    enum class PlugType : uint8_t {
        CAN = 0x00,  // CAN总线
        UART = 0x01, // UART串口
        PWM = 0x02,  // PWM信号
    };

    enum class CmdType : uint8_t {
        NOP = 0x00,           // 无操作
        Enable = 0x01,        // 使能
        Disable = 0x02,       // 失能
        CurrentCtrl = 0x03,   // 电流控制
        SpeedCtrl = 0x04,     // 速度控制
        AngleCtrl = 0x05,     // 角度控制
        LowSpeedCtrl = 0x06,  // 低速控制
        StepAngleCtrl = 0x07, // 角度步进控制

        Reboot = 0xFF,     // 重启
        SetZeroPos = 0xFE, // 设置零点
        ClearError = 0xFB, // 清除错误
    };

    static bool is_CmdType(const CmdType cmd) {
        switch (cmd) {
            case CmdType::NOP:
            case CmdType::Enable:
            case CmdType::Disable:
            case CmdType::CurrentCtrl:
            case CmdType::SpeedCtrl:
            case CmdType::AngleCtrl:
            case CmdType::LowSpeedCtrl:
            case CmdType::StepAngleCtrl:
            case CmdType::Reboot:
            case CmdType::SetZeroPos:
            case CmdType::ClearError:
                return true;
            default:
                return false;
        }
    }

    union RxData {
        struct __attribute__((packed)) {
            CmdType cmd_type; // 命令类型
            int16_t data;     // 命令数据
        } fields;

        uint8_t raw[8]; // 原始数据
    } rx_data{};

    PlugType plug = PlugType::CAN;
};

union TxData {
    struct __attribute__((packed)) {
        uint8_t id;          // 电机ID
        uint8_t motor_state; // 电机状态
        uint8_t error_code;  // 错误码(预留)
        int16_t current;     // Q轴电流
        int16_t speed;       // 电机转速
        int16_t angle;       // 电机角度
        uint8_t crc8;        // CRC8校验
    } data;

    uint8_t raw[10]; // 原始数据
};

extern QD4310 qd4310;
uint8_t UART_RxBuffer[sizeof(RxCommand::rx_data) + 2]; // UART接收缓冲区
void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan);
void CAN_Transmit(uint8_t length, uint8_t *pdata);
uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert);

xQueueHandle xQueue1;

void StartCommunicateTask(void *argument) {
    xQueue1 = xQueueCreate(5, sizeof(RxCommand));
    // 1.等待foc启动
    while (!qd4310.enabled)
        delay(10);
    // 2.初始化CAN并开启CAN接收
    FDCAN_Filter_INIT(&hfdcan1);
    // 3.开启UART接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART_RxBuffer, sizeof(UART_RxBuffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断

    RxCommand rx_command;
    while (true) {
        xQueueReceive(xQueue1, &rx_command, portMAX_DELAY);
        if (!RxCommand::is_CmdType(rx_command.rx_data.fields.cmd_type)) continue;

        bool status = false;
        switch (rx_command.rx_data.fields.cmd_type) {
            case RxCommand::CmdType::NOP: // NOP指令,只发送反馈报文
                status = true;
                break;
            case RxCommand::CmdType::Enable: // 使能指令
                status = qd4310.start();
                break;
            case RxCommand::CmdType::Disable: // 失能指令
                status = qd4310.stop();
                break;
            case RxCommand::CmdType::CurrentCtrl: // 电流控制
                status = qd4310.Ctrl(QD4310::CtrlType::CurrentCtrl,
                                     rx_command.rx_data.fields.data * 10.0f / INT16_MAX);
                break;
            case RxCommand::CmdType::SpeedCtrl: // 速度控制
                status = qd4310.Ctrl(QD4310::CtrlType::SpeedCtrl,
                                     rx_command.rx_data.fields.data * 1000.0f / INT16_MAX);
                break;
            case RxCommand::CmdType::AngleCtrl: // 角度控制
                status = qd4310.Ctrl(QD4310::CtrlType::AngleCtrl,
                                     rx_command.rx_data.fields.data * 2 * numbers::pi_v<float> / UINT16_MAX);
                break;
            case RxCommand::CmdType::LowSpeedCtrl: // 低速控制
                status = qd4310.Ctrl(QD4310::CtrlType::LowSpeedCtrl,
                                     rx_command.rx_data.fields.data * 1000.0f / INT16_MAX);
                break;
            case RxCommand::CmdType::StepAngleCtrl: // 角度步进
                status = qd4310.Ctrl(QD4310::CtrlType::StepAngleCtrl,
                                     rx_command.rx_data.fields.data * 2 * numbers::pi_v<float> / INT16_MAX);
                break;
            case RxCommand::CmdType::Reboot: // 重启
                status = true;
                break;
            case RxCommand::CmdType::SetZeroPos: // 设置零点
                status = qd4310.setZeroPosition();
                break;
            default:
                status = false;
                break;
        }
        static TxData tx_data{};
        tx_data.data.id = qd4310.ID;
        tx_data.data.motor_state = qd4310.started | status << 1 |
                                   static_cast<uint8_t>(qd4310.getCtrlType()) << 4;       // 电机状态
        tx_data.data.error_code = qd4310.error_code;                                      // 错误码
        tx_data.data.current = qd4310.getCurrent() / 10 * INT16_MAX;                      // Q轴电流
        tx_data.data.speed = qd4310.getSpeed() / 1000 * INT16_MAX;                        // 电机转速
        tx_data.data.angle = qd4310.getAngle() / (2 * numbers::pi_v<float>) * UINT16_MAX; // 电机角度
        tx_data.data.crc8 = CRC8(tx_data.raw, sizeof(tx_data.raw) - 1, 0x07, 0x00, 0x00, false, false);
        // 根据不同的接口类型发送反馈报文
        if (rx_command.plug == RxCommand::PlugType::CAN) {
            CAN_Transmit(sizeof(tx_data.raw) - 2, tx_data.raw + 1);
        } else if (rx_command.plug == RxCommand::PlugType::UART) {
            HAL_UART_Transmit_DMA(&huart3, tx_data.raw, sizeof(tx_data.raw));
        } else if (rx_command.plug == RxCommand::PlugType::PWM) {} else {}

        // 发送完毕后如果是重启指令则重启设备
        if (rx_command.rx_data.fields.cmd_type == RxCommand::CmdType::Reboot) {
            NVIC_SystemReset();
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
        static FDCAN_RxHeaderTypeDef RxHeader;
        static RxCommand rx_command{.rx_data = {}, .plug = RxCommand::PlugType::CAN};
        /*如果FIFO中有数据*/
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
            /*读取数据*/
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_command.rx_data.raw);
            // 如果是自己ID的报文且数据长度匹配,进行处理
            if (RxHeader.Identifier == 0x400 + qd4310.ID && RxHeader.DataLength == 3) {
                xQueueSendToBackFromISR(xQueue1, &rx_command, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

/**
 * @brief UART空闲接收回调函数
 * */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken;
    if (huart->Instance == huart3.Instance) {
        static RxCommand rx_command{.rx_data = {}, .plug = RxCommand::PlugType::UART};
        // 如果是自己ID的报文、数据长度匹配且CRC8校验通过,进行处理
        // id:1 byte, cmd:1 byte, data:2 bytes, crc8:1 byte
        if (UART_RxBuffer[0] == qd4310.ID && Size == 5 &&
            CRC8(UART_RxBuffer, 4, 0x07, 0x00, 0x00, false, false) == UART_RxBuffer[4]) {
            std::copy_n(UART_RxBuffer + 1, 3, rx_command.rx_data.raw);
            xQueueSendToBackFromISR(xQueue1, &rx_command, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        std::fill_n(UART_RxBuffer, sizeof(UART_RxBuffer), 0);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART_RxBuffer, sizeof(UART_RxBuffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART_RxBuffer, sizeof(UART_RxBuffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断
    }
}

void CAN_Transmit(uint8_t length, uint8_t *pdata) {
    /*定义CAN数据包头*/
    static FDCAN_TxHeaderTypeDef TxHeader = {
        0x500, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_ESI_ACTIVE,
        FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0
    };
    TxHeader.Identifier = 0x500 + qd4310.ID;
    TxHeader.DataLength = length;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pdata);
}


/**
 * @brief 反转字节(按bit反转)
 * */
static uint8_t ReverseBits(uint8_t data) {
    data = ((data & 0x55) << 1) | ((data & 0xAA) >> 1);
    data = ((data & 0x33) << 2) | ((data & 0xCC) >> 2);
    data = ((data & 0x0F) << 4) | ((data & 0xF0) >> 4);
    return data;
}

uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert) {
    /**1.校验参数**/
    assert_param(data != nullptr);
    assert_param(len > 0);
    /**2.变量定义**/
    uint8_t crc = init;
    /**3.计算**/
    do {
        crc ^= input_invert ? ReverseBits(*(data++)) : *(data++);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    } while (--len);
    return output_invert ? ReverseBits(crc ^ xor_out) : (crc ^ xor_out);
}
