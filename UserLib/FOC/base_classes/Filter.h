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

    virtual float getTs() = 0; //!< Return filter time constant
    virtual float operator()(float x) = 0;
};

class LP_Filter : public Filter {
public:
    virtual float getFc() = 0; //!< Return filter cut-off frequency, unit s
};

class HP_Filter : public Filter {
public:
    virtual float getFc() = 0; //!< Return filter cut-off frequency, unit s
};

class BP_Filter : public Filter {
public:
    virtual float getFp() = 0;                  //!< Return filter cut-off frequency, unit s
    virtual float getFs() = 0;                  //!< Return filter cut-off frequency, unit s
    float getBW() { return getFs() - getFp(); } //!< Return filter bandwidth, unit s
};

class BS_Filter : public Filter {
public:
    virtual float getFp() = 0;                  //!< Return filter cut-off frequency, unit s
    virtual float getFs() = 0;                  //!< Return filter cut-off frequency, unit s
    float getBW() { return getFp() - getFs(); } //!< Return filter bandwidth, unit s
};

#endif //FILTER_H
