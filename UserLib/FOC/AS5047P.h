/**
 * @brief   AS5047P驱动库
 * @details
 * @author  LiuHaoqi
 * @date    2024-6-22
 * @version V2.0.0
 * @note    AS5047P的SPI接口速率上限为10Mbs,需确保SPI速率在10Mbs以下
 *          CPOL=0,CPHA=1,即CubeMX中将SPI的时钟极性设置为Low,时钟相位设置为2Edge
 *          OTP编程函数尚未完全验证
 * @warning 根据手册上的说明,OTP编程只能进行一次,编程后不可更改,请谨慎使用!
 * @par     历史版本:
 *		    V1.0.0创建于2024-4-24
 *		    V1.1.0创建于2024-6-22,更新注释,添加ERROR寄存器查询函数
 *		    V2.0.0创建于2024-6-24,隐藏ReadREG函数接口,使用AS5047P_ReadAngleContinuously函数代替
 *		                         添加OTP编程函数,用于AS5047P的非易失寄存器编程
 * */

#ifndef AS5047P_H
#define AS5047P_H

#include <stdbool.h>
#include "main.h"
#include "retarget.h"

//是否打印编程信息
#define AS5047P_PRINT_PROGRAMMING_INFO 0

/*易失寄存器表*/
/*Volatile Register Table*/
#define NOP         0x0000  //No operation
#define ERRFL       0x0001  //Error register, bit0,SPI frame error;bit1,Command error;bit2,Parity error
#define PROG        0x0003  //Programming register, bit0,
#define DIAAGC      0x3FFC  //Diagnostic and AGC,
#define MAG         0x3FFD  //CORDIC magnitude
#define ANGLEUNC    0x3FFE  //Measured angle without dynamic angle error compensation\
                              bit0,

#define ANGLECOM    0x3FFF  //Measured angle with dynamic angle error compensation\
                              bit0,

/*非易失寄存器表*/
/*Non-Volatile Register Table*/
#define ZPOSM       0x0016  //Zero position MSB
#define ZPOSL       0x0017  //Zero position LSB/MAG diagnostic
#define SETTINGS1   0x0018  //Custom setting register 1
#define SETTINGS2   0x0019  //Custom setting register 2

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AS5047P错误枚举
 * */
typedef enum {
    AS5047P_SPI_FRAME_ERROR = 0x0001, //SPI帧错误
    AS5047P_COMMAND_ERROR = 0x0002,   //命令错误
    AS5047P_PARITY_ERROR = 0x0004,    //偶校验错误
} AS5047P_Error_EnumTypeDef;

/**
 * @brief AS5047P编程状态枚举
 * */
typedef enum {
    AS5047P_PROGRAMMING_OK = 0x00,
    AS5047P_PROGRAMMING_WRITE_ERROR = 0x01,
    AS5047P_PROGRAMMING_READ_ERROR = 0x02,
    AS5047P_PROGRAMMING_VERIFY_ERROR = 0x03,
} AS5047P_ProgrammingStatus_EnumTypeDef;

/**
 * @brief AS5047P模式枚举
 * */
typedef enum {
    AS5047P_MODE_UVW_ABI = 0x00,
    AS5047P_MODE_UVW_PWM = 0x01,
    AS5047P_MODE_ABI_PWM = 0x02,
} AS5047P_Mode_EnumTypeDef;

/**
 * @brief AS5047PABI分辨率枚举
 * */
typedef enum {
    AS5047P_ABI_RESOLUTION_1000_PULSES = 0x00,
    AS5047P_ABI_RESOLUTION_500_PULSES = 0x01,
    AS5047P_ABI_RESOLUTION_400_PULSES = 0x02,
    AS5047P_ABI_RESOLUTION_300_PULSES = 0x03,
    AS5047P_ABI_RESOLUTION_200_PULSES = 0x04,
    AS5047P_ABI_RESOLUTION_100_PULSES = 0x05,
    AS5047P_ABI_RESOLUTION_50_PULSES = 0x06,
    AS5047P_ABI_RESOLUTION_25_PULSES = 0x07,
    AS5047P_ABI_RESOLUTION_1024_PULSE = 0x08,
    AS5047P_ABI_RESOLUTION_512_PULSE = 0x09,
    AS5047P_ABI_RESOLUTION_256_PULSE = 0x0A,
} AS5047P_ABI_Resolution_EnumTypeDef;

/**
 * @brief AS5047P滞迟枚举
 * @note LSB由ABI分辨率决定
 * */
typedef enum {
    AS5047P_HYSTERESIS_3LSB = 0x00,
    AS5047P_HYSTERESIS_2LSB = 0x01,
    AS5047P_HYSTERESIS_1LSB = 0x02,
    AS5047P_HYSTERESIS_0LSB = 0x03,
} AS5047P_Hysteresis_EnumTypeDef;

/**
 * @brief AS5047P UVW极对数枚举
 * */
typedef enum {
    AS5047P_UVW_PolePairs_1 = 0x00,
    AS5047P_UVW_PolePairs_2 = 0x01,
    AS5047P_UVW_PolePairs_3 = 0x02,
    AS5047P_UVW_PolePairs_4 = 0x03,
    AS5047P_UVW_PolePairs_5 = 0x04,
    AS5047P_UVW_PolePairs_6 = 0x05,
    AS5047P_UVW_PolePairs_7 = 0x06,
} AS5047P_UVW_PolePairs_EnumTypeDef;

/**
 * @brief AS5047P编程结构体
 * */
typedef struct {
    bool Direction;                                    //旋转方向
    bool DynamicAngleErrorCompensation;                //动态角度误差补偿
    AS5047P_Mode_EnumTypeDef Mode;                     //模式
    AS5047P_ABI_Resolution_EnumTypeDef ABI_Resolution; //ABI分辨率
    AS5047P_Hysteresis_EnumTypeDef Hysteresis;         //滞迟
    AS5047P_UVW_PolePairs_EnumTypeDef UVW_PolePairs;   //UVW极对数
} AS5047P_OTP_Config_TypeDef;


/**
 * @brief AS5047P结构体
 * */
typedef struct {
    GPIO_TypeDef *CS_GPIO_Port_; //CS引脚端口号
    uint16_t CS_GPIO_Pin_;       //CS引脚号
    SPI_HandleTypeDef *hspi;     //SPI句柄
} AS5047P_TypeDef;

/**
 * @brief AS5047P初始化函数
 * @param AS5047P AS5047P结构体指针
 * @param hspi spi句柄指针
 * @param CS_GPIO_Port CS引脚端口号
 * @param CS_GPIO_Pin CS引脚号
 * */
extern void
AS5047P_Init(AS5047P_TypeDef *AS5047P, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin);

/**
 * @brief AS5047P持续读取角度
 * @note 由于高速连续读取的需要,函数中不再校验读取是否成功
 * @param AS5047P AS5047P结构体指针
 * @param pRxData 接收数据指针
 * */
extern void AS5047P_ReadAngleContinuously(AS5047P_TypeDef *AS5047P, uint16_t *pRxData);

/**
 * @brief AS5047P读取Error寄存器函数
 * @param AS5047P AS5047P结构体指针
 * @return AS5047P_Error_EnumTypeDef 错误枚举
 * */
extern AS5047P_Error_EnumTypeDef AS5047P_GetError(AS5047P_TypeDef *AS5047P);

/**
 * @brief AS5047P OTP非易失寄存器编程函数
 * @note 尚未完全验证
 * @warning 根据手册上的说明,OTP编程只能进行一次,编程后不可更改,请谨慎使用!
 * @param AS5047P AS5047P结构体指针
 * @param Config AS5047P编程配置结构体指针
 * @return AS5047P_ProgrammingStatus_EnumTypeDef 编程状态
 * */
extern AS5047P_ProgrammingStatus_EnumTypeDef
AS5047P_Programming(AS5047P_TypeDef *AS5047P, AS5047P_OTP_Config_TypeDef *Config);

#ifdef __cplusplus
}
#endif

#endif //AS5047P_H
