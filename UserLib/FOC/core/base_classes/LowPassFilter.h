/**
 * @brief   LowPassFilter base class
 * @details User should override the pure virtual functions to implement the driver.
 * @author  LiuHaoqi
 * @date    2025-4-11
 * @version V1.0.0
 * @note
 * @warning
 * @par     history:
            V1.0.0 on 2025-4-11
 * */

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
public:
    virtual ~LowPassFilter() = default;
    // user should define constructor self, just to assign the member variables.

    float Fc{}; //!< Low pass filter cut-off frequency
    virtual float operator()(float x) = 0;
};

#endif //LOWPASSFILTER_H
