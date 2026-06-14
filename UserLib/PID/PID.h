/**
 * @brief       PID基础控制库(C++)
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-6-14
 * @version     V1.2.1
 * @note
 * @warning
 * @par         历史版本
 *              V1.0.0创建于2024-9-21,更改自C语言版本PID库
 *              V1.1.0创建于2025-1-20,添加类初始化默认限幅参数,现在可以在初始化时设置限幅
 *              V1.1.1创建于2025-7-8,修改限幅NAN判断
 *              V1.2.0创建于2026-6-13,添加Ts控制周期,添加可选的dinput输入
 *              V1.2.1创建于2026-6-14,使用optional作为可选的限幅类型
 * @copyright   (c) 2025 QDrive
 * */

#ifndef PID_H
#define PID_H

#include <cmath>
#include <optional>

class PID {
public:
    enum PID_type {
        position_type, //位置式PID
        delta_type,    //增量式PID
    };

    /**
     * @brief 不允许默认初始化,初始化必须要有PID种类,kp,ki,kd参数
     * */
    PID() = delete;

    /**
     * @brief PID结构体初始化函数
     * @param PID_type          PID类型
     * @param kp,ki,kd          PID三个参数
     * @param Ts                控制周期,单位s,默认为1s
     * @param sum_error_limit_p 积分限幅上限
     * @param sum_error_limit_n 积分限幅下限
     * @param output_limit_p    输出限幅上限
     * @param output_limit_n    输出限幅下限
     * */
    PID(const PID_type PID_type, const float kp, const float ki, const float kd, const float Ts = 1,
        const std::optional<float> sum_error_limit_p = std::nullopt,
        const std::optional<float> sum_error_limit_n = std::nullopt,
        const std::optional<float> output_limit_p = std::nullopt,
        const std::optional<float> output_limit_n = std::nullopt)
        : PID_type(PID_type), kp(kp), ki(ki), kd(kd), Ts(Ts),
          sum_error_limit_p(sum_error_limit_p), sum_error_limit_n(sum_error_limit_n),
          output_limit_p(output_limit_p), output_limit_n(output_limit_n) {}

    /**
     * @brief 设置PID目标值
     * @param Target 新的PID目标值
     * */
    void SetTarget(const float Target) { target = Target; }

    /**
     * @brief 设置积分值,用于高自由度的积分限幅
     * @param sum_error PID积分值
     */
    void set_sum_error(const float sum_error) { PID::sum_error = sum_error; }

    /**
     * @brief 复位PID运行时状态
     */
    void reset() {
        error = 0;
        pre_target = 0;
        pre_error = 0;
        sum_error = 0;
        output = 0;
    }

    /**
     * @brief PID计算
     * @param input PID观测值
     * @param dinput PID观测值的微分,若外部可以直接获取微分传入,则用外部微分替换PID内部的差分
     * @return PID计算结果
     * */
    [[nodiscard]] float calc(float input, std::optional<float> dinput = {});

    /*设置参数*/
    float target{0};   // 目标值
    PID_type PID_type; // PID种类,位置式或增量式
    float kp, ki, kd;  // 比例、积分、微分系数
    float Ts{1};       // 控制周期

    /*限幅参数*/
    std::optional<float> sum_error_limit_p{}; // 积分限幅上限,仅位置式有效
    std::optional<float> sum_error_limit_n{}; // 积分限幅下限,仅位置式有效
    std::optional<float> output_limit_p{};    // 输出限幅上限
    std::optional<float> output_limit_n{};    // 输出限幅下限
private:
    /*中间(运行时)变量*/
    float error{0};      // 上一次的偏差值
    float pre_target{0}; // 上一次的目标值
    float pre_error{0};  // 上上一次的偏差值,仅增量式PID使用
    float sum_error{0};  // 累计偏差值,仅位置式PID使用
    float output{0};     // PID输出值
};

inline float PID::calc(const float input, const std::optional<float> dinput) {
    const float error_ = target - input;
    if (PID_type == position_type) {
        /***位置式PID公式:u=Kpe(t)+Ki*e(t)的积分+Kd[e(t)-e(t-1)]***/
        sum_error += error_ * Ts;
        /*积分限幅*/
        if (sum_error_limit_p && sum_error >= sum_error_limit_p) sum_error = sum_error_limit_p.value();
        if (sum_error_limit_n && sum_error <= sum_error_limit_n) sum_error = sum_error_limit_n.value();
        output = kp * error_ +
                 ki * sum_error +
                 kd * (dinput ? (target - pre_target) / Ts - dinput.value() : (error_ - error) / Ts);
        error = error_;
        pre_target = target;
    } else if (PID_type == delta_type) {
        /***增量式PID公式:du=Kp[e(t)-e(t-1)]+Kie(t)+Kd[e(t)-2e(t-1)+e(t-2)]***/
        output += kp * (error_ - error) +
            ki * (error_) * Ts +
            kd * (error_ - 2 * error + pre_error) / Ts;
        pre_error = error;
        error = error_;
        pre_target = target;
    }
    /*输出限幅*/
    if (output_limit_p && output >= output_limit_p) output = output_limit_p.value();
    if (output_limit_n && output <= output_limit_n) output = output_limit_n.value();

    return output;
}

#endif //PID_H
