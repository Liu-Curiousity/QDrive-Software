/**
 * @brief       PID基础控制库(C++)
 * @details
 * @author      Haoqi Liu
 * @date        2024-9-21
 * @version     V1.0.0
 * @note
 * @warning
 * @par         历史版本
 *              V1.0.0创建于2024-9-21,更改自C语言版本PID库
 * */

#include "PID.h"

void PID::SetLimit(float SumError_limit_p, float SumError_limit_n, float Output_limit_p, float Output_limit_n) {
    sum_error_limit_p = SumError_limit_p;
    sum_error_limit_n = SumError_limit_n;
    output_limit_p    = Output_limit_p;
    output_limit_n    = Output_limit_n;
}
