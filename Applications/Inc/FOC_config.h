/**
 * @brief 		用于定义FOC控制器的配置常量
 * @detail
 * @author 	    Haoqi Liu
 * @date        25-5-5
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于25-5-5
 * */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#define FOC_HARDWARE_VERSION "4310_5.1.1"
#define FOC_SOFTWARE_VERSION "5.2.1"

/*==========================电机参数==========================*/
#define FOC_KV                  33.0f   // KV值,单位rpm/V
#define FOC_POLE_PAIRS          14      // 极对数
#define FOC_NOMINAL_VOLTAGE     24      // 额定电压,单位V
#define FOC_PHASE_INDUCTANCE    4.74f   // 相电感,单位mH
#define FOC_PHASE_RESISTANCE    10.9f   // 相电阻,单位Ω
#define FOC_TORQUE_CONSTANT     0.27f   // 转矩常数,单位Nm/A

/*=========================驱动板参数==========================*/
#define FOC_MAX_CURRENT         1.65f   // 最大电流,单位A

/*==========================配置参数==========================*/
#define FOC_MAX_SPEED           1000.0f   // 最大转速,单位rpm

#define FOC_CURRENT_KP          10.0f
#define FOC_CURRENT_KI          1.0f
#define FOC_CURRENT_KD          0.0f
#define FOC_SPEED_KP            3e-3f
#define FOC_SPEED_KI            3.9e-4f
#define FOC_SPEED_KD            0.0f
#define FOC_ANGLE_KP            1200.0f
#define FOC_ANGLE_KI            0.0f
#define FOC_ANGLE_KD            0.0f

#endif //FOC_CONFIG_H
