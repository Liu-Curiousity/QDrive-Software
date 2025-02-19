/**
 * @brief   BLDC_Driver base class
 * @details User should override the pure virtual functions to implement the driver.
 * @author  LiuHaoqi
 * @date    2025-4-8
 * @version V3.0.0
 * @note
 * @warning
 * @par     history:
            V1.0.0 on 2024-5-12
            V2.0.0 on 2025-1-20,refactor by C++
            V3.0.0 on 2025-4-8,redesign refer to SimpleFOC
 * */

#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

class BLDC_Driver {
public:
    virtual ~BLDC_Driver() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    bool initialized = false; // true if the driver is initialized
    bool enabled = false;     // true if the driver is enabled

    virtual void init() = 0;    // initialize the driver
    virtual void enable() = 0;  // enable the driver
    virtual void disable() = 0; // disable the driver

    // set the duty cycle of the driver, then Uu Uv Uw is equal to Vbus * duty
    // the duty cycle is normalized to [0, 1]
    virtual void set_duty(float u, float v, float w) = 0;

    // user should declare hardware specific parameters self
};

#endif //BLDC_DRIVER_H
