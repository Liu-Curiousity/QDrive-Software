/**
 * @brief 		CDC_Tx_DualBuffer.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2025/9/11
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于2025/9/11
 * */

#ifndef FOC_PM4310_CDC_TX_DUALBUFFER_H
#define FOC_PM4310_CDC_TX_DUALBUFFER_H
#include <algorithm>

template <typename Tp, std::size_t Nm>
class TxDualBuffer {
public:
    explicit TxDualBuffer(uint8_t (*TxFunc)(uint8_t *, uint16_t)) : TxFunc(TxFunc) {};

    [[nodiscard]] bool isTransmitting() const { return transmitting; }

    bool inBuffer(Tp *& buf, std::size_t len) {
        static bool result = false;
        HAL_NVIC_DisableIRQ(USB_LP_IRQn);
        if (Nm - buffer_index >= len) {
            std::copy(buf, buf + len, vice_buffer + buffer_index);
            buffer_index += len;
            result = true;
        } else {
            result = false;
        }
        // if (buffer_index && !transmitting) start_transmit();
        if (buffer_index) start_transmit(); // 原本是要判断transmitting的,但是该死的CDC,发送失败是不会有回调函数的,故只能这样写了
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);
        return result;
    }

    void transmitComplete() {
        if (buffer_index) start_transmit();
        else transmitting = false;
    }

private:
    Tp buffer1[Nm]{};
    Tp buffer2[Nm]{};
    Tp *main_buffer{buffer1};
    Tp *vice_buffer{buffer2};
    size_t buffer_index{0};
    bool transmitting{false};
    uint8_t (*TxFunc)(uint8_t *, uint16_t);

    void start_transmit() {
        if (TxFunc(reinterpret_cast<uint8_t *>(vice_buffer), buffer_index) == 0) {
            std::swap(main_buffer, vice_buffer);
            buffer_index = 0;
            transmitting = true;
        } else {
            transmitting = false;
        }
    }
};

#endif //FOC_PM4310_CDC_TX_DUALBUFFER_H
