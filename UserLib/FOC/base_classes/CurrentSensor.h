/**
 * @file        CurrentSensor.h
 * @brief 		CurrentSensor base class
 * @detail      User should override the pure virtual functions to implement the sensor.
 * @author 	    Liu-Curiousity (2675794963@qq.com)
 * @date        26-5-29
 * @version 	V1.1.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于25-5-4
                V1.1.0创建于26-5-29, 添加电流偏置设置
 * @copyright   (c) 2026 QDrive
 * */

#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

class CurrentSensor {
public:
    virtual ~CurrentSensor() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    bool initialized = false; // true if the sensor is initialized
    bool enabled = false;     // true if the sensor is enabled

    float iu{}, iv{}, iw{}; // current in u, v and w axis, units: A

    virtual void init() = 0;                                                        // initialize the sensor
    virtual void enable() = 0;                                                      // enable the sensor
    virtual void disable() = 0;                                                     // disable the sensor
    virtual void set_offset(float iu_offset, float iv_offset, float iw_offset) = 0; // set offset of the sensor

    // user should declare hardware specific parameters self
};

#endif //CURRENTSENSOR_H
