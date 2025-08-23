/**
 * @brief   Filter base class
 * @details User should override the pure virtual functions to implement the driver.
 * @author  LiuHaoqi
 * @date    2025-4-11
 * @version V1.0.0
 * @note
 * @warning
 * @par     history:
            V1.0.0 on 2025-4-11
 * */

#ifndef FILTER_H
#define FILTER_H

class Filter {
public:
    virtual ~Filter() = default;
    // user should define constructor self, just to assign the member variables.

    virtual float getFc() = 0; //!< Return low pass filter cut-off frequency
    virtual float operator()(float x) = 0;
};

#endif //FILTER_H
