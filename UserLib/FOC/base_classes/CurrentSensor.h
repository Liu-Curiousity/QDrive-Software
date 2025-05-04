/**
 * @brief 		CurrentSensor base class
 * @detail      User should override the pure virtual functions to implement the driver.
 * @author 	    Haoqi Liu
 * @date        25-5-4
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于25-5-4
 * */

#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

class CurrentSensor {
public:
    virtual ~CurrentSensor() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    bool initialized = false; // true if the driver is initialized
    bool enabled = false;     // true if the driver is enabled

    float iu, iv, iw; // current in u, v and w axis, units: A

    virtual void init() = 0;    // initialize the driver
    virtual void enable() = 0;  // enable the driver
    virtual void disable() = 0; // disable the driver

    // user should declare hardware specific parameters self
};

#endif //CURRENTSENSOR_H
