/**
 * @file        Encoder.h
 * @brief       Encoder base class
 * @details     User should override the pure virtual functions to implement the driver.
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-6-14
 * @version     V3.1.0
 * @note
 * @warning
 * @par         history:
                V1.0.0 on 2024-7-3
                V2.0.0 on 2025-1-20,refactor by C++
                V3.0.0 on 2025-4-8,redesign refer to SimpleFOC
                V3.1.0 on 2026-6-14,add resolution
 * @copyright   (c) 2026 QDrive
 * */

#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
    virtual ~Encoder() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    bool initialized = false; // true if the driver is initialized
    bool enabled = false;     // true if the driver is enabled
    float resolution;         // encoder resolution, unit: rad

    virtual void init() = 0;    // initialize the driver
    virtual void enable() = 0;  // enable the driver
    virtual void disable() = 0; // disable the driver
    virtual float get_angle() =0;

    // user should declare hardware specific parameters self
};

#endif //ENCODER_H
