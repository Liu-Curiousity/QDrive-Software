/**
 * @brief 		filters implementation
 * @detail
 * @author 	    Haoqi Liu
 * @date        25-4-17
 * @version 	V3.0.1
 * @note
 * @warning
 * @par 		history
                V1.0.0 on 24-12-7
                V2.0.0 on 25-2-21,optimize LowPassFilter(rename,delete template,add constructor,add operator())
                V3.0.0 on 25-4-11,refactor with C++ inheritance
                V3.0.1 on 25-4-17,replace malloc/free with new/delete
 * */

#ifndef FILTERS_H
#define FILTERS_H

#include "Filter.h"
#include <algorithm>
#include <cstring>
#include <initializer_list>
#include <numbers>

using namespace std;

/**
 * @brief one-order low pass filter
 */
class LowPassFilter_1_Order final : public LP_Filter {
public:
    /**
     * @brief constructor
     * @param Ts Low pass filter time constant, unit s
     * @param Fc Low pass filter cut-off frequency, unit Hz
     */
    LowPassFilter_1_Order(const float Ts, const float Fc) :
        Ts(Ts), Fc(Fc), a(2 * numbers::pi_v<float> * Fc * Ts / (2 * numbers::pi_v<float> * Fc * Ts + 1)) {}

    float getFc() override { return Fc; }
    float getTs() override { return Ts; }

    float operator()(const float value) override {
        this->value = a * value + (1 - a) * this->value;
        return this->value;
    }

private:
    const float Ts{0}; // Low pass filter time constant, unit s
    const float Fc{0}; // Low pass filter cut-off frequency, unit Hz
    float value{0};
    const float a{1}; // filter coefficient,default 1(no filter)
};

/**
 * @brief two-order low pass filter
 */
class LowPassFilter_2_Order final : public LP_Filter {
public:
    /**
     * @brief constructor
     * @param Ts Low pass filter time constant, unit s
     * @param Fc Low pass filter cut-off frequency, unit Hz
     * @param dampingRatio Damping ratio, default 0.707
     */
    LowPassFilter_2_Order(const float Ts, const float Fc, const float dampingRatio = 0.707) :
        Ts(Ts), Fc(Fc), Wc(2 / Ts * tan(numbers::pi_v<float> * Fc * Ts)),
        b0(Wc * Wc * Ts * Ts), b1(2 * b0), b2(b0),
        a0(4 + 4 * dampingRatio * Wc * Ts + b0),
        a1(-8 + 2 * b0), a2(4 - 4 * dampingRatio * Wc * Ts + b0) {}

    float getFc() override { return Fc; }
    float getTs() override { return Ts; }

    float operator()(const float x) override {
        xin[2] = x;
        yout[2] = (b0 * xin[2] + b1 * xin[1] + b2 * xin[0] - a1 * yout[1] - a2 * yout[0]) / a0;
        xin[0] = xin[1];
        xin[1] = xin[2];
        yout[0] = yout[1];
        yout[1] = yout[2];

        return yout[2];
    }

private:
    const float Ts{0}; // Low pass filter time constant, unit s
    const float Fc{0}; // Low pass filter cut-off frequency, unit Hz
    const float Wc{0};

    const float b0{0};
    const float b1{0};
    const float b2{0};
    const float a0{0};
    const float a1{0};
    const float a2{0};

    float xin[3]{};
    float yout[3]{};
};


/**
 * @brief two-order band pass filter
 */
class BandPassFilter_2_Order final : public BP_Filter {
public:
    /**
     * @brief constructor
     * @param Ts Band pass filter time constant, unit s
     * @param Fp Band pass filter pass frequency, unit Hz
     * @param Fs Band pass filter stop frequency, unit Hz
     */
    BandPassFilter_2_Order(const float Ts, const float Fp, const float Fs) :
        Ts(Ts), Fp(Fp), Fs(Fs),
        Wp(2 / Ts * tan(numbers::pi_v<float> * Fp * Ts)),
        Ws(2 / Ts * tan(numbers::pi_v<float> * Fs * Ts)),
        Wc(sqrt(Wp * Ws)),
        b0(2 / Ts * (Ws - Wp)), b1(0), b2(-b0),
        a0(4 / Ts / Ts + b0 + Ws * Wp),
        a1(-8 / Ts / Ts + 2 * Ws * Wp),
        a2(4 / Ts / Ts - b0 + Ws * Wp) {}

    float getFp() override { return Fp; }
    float getFs() override { return Fs; }
    float getTs() override { return Ts; }

    float operator()(const float x) override {
        xin[2] = x;
        yout[2] = (b0 * xin[2] + b1 * xin[1] + b2 * xin[0] - a1 * yout[1] - a2 * yout[0]) / a0;
        xin[0] = xin[1];
        xin[1] = xin[2];
        yout[0] = yout[1];
        yout[1] = yout[2];

        return yout[2];
    }

private:
    const float Ts{0}; // Band pass filter time constant, unit s
    const float Fp{0}; // Band pass filter pass frequency, unit Hz
    const float Fs{0}; // Band pass filter stop frequency, unit Hz
    const float Wp{0};
    const float Ws{0};
    const float Wc{0};

    const float b0{0};
    const float b1{0};
    const float b2{0};
    const float a0{0};
    const float a1{0};
    const float a2{0};

    float xin[3]{};
    float yout[3]{};
};

/**
 * @brief one-order Kalman filter
 */
class KalmanFilter_1_Order final : public Filter {
public:
    /**
     * @brief constructor
     * @param Q 过程噪声协方差,Q为对模型的信任度,Q越大,滤波后的曲线跟测量曲线跟的越紧密,滤波后噪声越大
     * @param R 观测噪声协方差,R决定稳态噪声,小了初始增益大,但是稳态容易引入噪声;R越大对噪声越不敏感,即滤波后的数据跳动越小,但R越大,kalman滤波输出收敛的越慢
     */
    KalmanFilter_1_Order(const float Q, const float R) : Q(Q), R(R) {}

    float getTs() override { return 0; }
    float operator()(const float value) override {
        P = P + Q;
        k = P / (P + R);
        x = x + k * (value - x);
        P = (1 - k) * P;
        return x;
    }

private:
    float x{0};
    float P{1};
    float k{0};
    float Q{0};
    float R{0};
};

class MovingAverageFilter final : public Filter {
public:
    /**
     * @brief constructor
     * @param window_size window size
     */
    explicit MovingAverageFilter(const size_t window_size) :
        window_size(window_size), values(new float[window_size]) {}

    ~MovingAverageFilter() override { delete[] values; }

    float getTs() override { return 0; }
    float operator()(const float value) override {
        sum -= values[index];
        values[index] = value;
        sum += value;
        index = (index + 1) % window_size;
        return sum / window_size;
    }

private:
    size_t index{0};
    size_t window_size{0};
    float sum{0};
    float *values{nullptr};
};

class FIRFilter final : public Filter {
public:
    FIRFilter(const initializer_list<float> coefficients) :
        order(coefficients.size()), coef(new float[order]), buffer(new float[order]) {
        ranges::copy(coefficients, coef);
        std::fill_n(buffer, order, 0.0f);
    }

    ~FIRFilter() override {
        delete[] coef;
        delete[] buffer;
    }

    float getTs() override { return 0; }

    float operator()(const float value) override {
        buffer[index] = value;
        float result = 0.0f;
        size_t buf_index = index;
        for (size_t i = 0; i < order; ++i) {
            result += coef[i] * buffer[buf_index];
            buf_index = (buf_index == 0) ? order - 1 : buf_index - 1;
        }
        index = (index + 1) % order;
        return result;
    }

private:
    size_t order{0};
    size_t index{0};
    float *coef{nullptr};
    float *buffer{nullptr};
};

#endif //FILTERS_H
