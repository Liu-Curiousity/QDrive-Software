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

#include "AS5047P.h"

#if AS5047P_PRINT_PROGRAMMING_INFO == 1
#define PRINT_INFO(...) printf(__VA_ARGS__)
#else
#define PRINT_INFO(...)
#endif

/**
 * @brief 偶校验函数,生成偶校验位
 * @param Data 待生成校验的数据
 * @return 偶校验位
 * */
static bool EvenParity(uint16_t Data) {
    bool ParityBit = false;
    do {
        ParityBit ^= Data & 1;
    } while (Data >>= 1);
    return ParityBit;
}

/**
 * @brief AS5047P基础收发函数(传输1次)
 * @note HAL库SPI收发函数到AS5047P收发函数的中间层,在函数中完成偶校验的计算,判断读取是否成功
 * @param AS5047P AS5047P结构体指针
 * @param txData 待发送数据,无需偶校验,本函数会自动添加偶校验
 * @param pRxData 接收的数据,本函数已剔除校验位,和错误标志位(以返回值的形式告知用户)
 * @return 0:正常执行   -1:读取异常
 * */
static int AS5047P_TransmitReceive(AS5047P_TypeDef *AS5047P, uint16_t txData, uint16_t *pRxData) {
    int result = 0;
    txData += EvenParity(txData) << 15;
    HAL_GPIO_WritePin(AS5047P->CS_GPIO_Port_, AS5047P->CS_GPIO_Pin_, GPIO_PIN_RESET);
    /*如果返回值不等于HAL_OK*/
    if (HAL_SPI_TransmitReceive(AS5047P->hspi, (uint8_t *) &txData, (uint8_t *) pRxData, 1, 100))
        result = -1;
    HAL_GPIO_WritePin(AS5047P->CS_GPIO_Port_, AS5047P->CS_GPIO_Pin_, GPIO_PIN_SET);
    /*如果接收到错误标志位*/
    if (*pRxData & (1 << 14))
        result = -1;
    *pRxData &= 0x3FFF;
    return result;
}

/**
 * @brief AS5047P写入寄存器
 * @note 在函数中完成偶校验的计算,判断读取是否成功
 * @param AS5047P AS5047P结构体指针
 * @param Reg 待写入寄存器
 * @param TxData 待写入数据
 * @return 0:正常执行   -1:读取异常
 * */
static int AS5047P_WriteReg(AS5047P_TypeDef *AS5047P, uint16_t Reg, uint16_t TxData) {
    uint16_t rxData;
    //检验数据传输是否成功
    if (AS5047P_TransmitReceive(AS5047P, Reg, &rxData)) return -1;
    if (AS5047P_TransmitReceive(AS5047P, TxData, &rxData)) return -1;
    //note:这里读到的rxdata是写入前寄存器中的值
    return 0;
}

/**
 * @brief AS5047P读取寄存器
 * @note 在函数中完成偶校验的计算,判断读取是否成功
 * @param AS5047P AS5047P结构体指针
 * @param Reg 待写入寄存器
 * @param pRxData 待写入数据
 * @return 0:正常执行   -1:读取异常
 * */
int AS5047P_ReadReg(AS5047P_TypeDef *AS5047P, uint16_t Reg, uint16_t *pRxData) {
    if (AS5047P_TransmitReceive(AS5047P, Reg | (1 << 14), pRxData))     //第14位 置1表示读取
        return -1;
    if (AS5047P_TransmitReceive(AS5047P, NOP, pRxData))                 //发送NOP同时接收返回数据
        return -1;
    return 0;
}

void AS5047P_Init(AS5047P_TypeDef *AS5047P, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin) {
    AS5047P->CS_GPIO_Port_ = CS_GPIO_Port;
    AS5047P->CS_GPIO_Pin_ = CS_GPIO_Pin;
    AS5047P->hspi = hspi;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_GPIO_Pin, GPIO_PIN_SET);
}

__attribute__((section(".ccmram_func")))
void AS5047P_ReadAngleContinuously(AS5047P_TypeDef *AS5047P, uint16_t *pRxData) {
    /**为极致效率放弃兼容性**/
    static uint16_t txData = 0xFFFF;
    HAL_GPIO_WritePin(AS5047P->CS_GPIO_Port_, AS5047P->CS_GPIO_Pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(AS5047P->hspi, (uint8_t *) &txData, (uint8_t *) pRxData, 1, 100);
    HAL_GPIO_WritePin(AS5047P->CS_GPIO_Port_, AS5047P->CS_GPIO_Pin_, GPIO_PIN_SET);
    *pRxData &= 0x3FFF;
}

AS5047P_Error_EnumTypeDef AS5047P_GetError(AS5047P_TypeDef *AS5047P) {
    uint16_t rxData;
    AS5047P_ReadReg(AS5047P, ERRFL, &rxData);
    return (AS5047P_Error_EnumTypeDef) rxData;
}

AS5047P_ProgrammingStatus_EnumTypeDef
AS5047P_Programming(AS5047P_TypeDef *AS5047P, AS5047P_OTP_Config_TypeDef *Config) {

    /**step1:Verify parameter**/
    assert_param(AS5047P);
    assert_param(Config);
    AS5047P_ProgrammingStatus_EnumTypeDef Status = AS5047P_PROGRAMMING_OK;
    uint16_t Settings1 = 0;
    uint16_t Settings2 = 0;
    uint16_t ZeroPositionM = 0;
    uint16_t ZeroPositionL = 0;
    uint16_t Settings1_Verify = 0;
    uint16_t Settings2_Verify = 0;
    uint16_t ZeroPositionM_Verify = 0;
    uint16_t ZeroPositionL_Verify = 0;

    /**step2:Config Settings1 and Settings2 by Config structure**/
    Settings1 |= (1 << 0);                                              //bit0 Pre-Programmed to 1
    Settings1 |= (0 << 1);                                              //bit1 Pre-Programmed to 0
    Settings1 |= (Config->Direction << 2);                              //bit2 Rotation direction
    Settings1 |= ((Config->Mode == AS5047P_MODE_UVW_PWM) << 3);         //bit3 0:ABI operating, 1:UVW operating
    Settings1 |= ((!Config->DynamicAngleErrorCompensation) << 4);       //bit4 AngleErrorCompensation 0:Enable 1:Disable
    //bit5 ABI decimal or binary selection 0:Decimal 1:Binary
    Settings1 |= ((Config->ABI_Resolution >= AS5047P_ABI_RESOLUTION_1024_PULSE) << 5);
    Settings1 |= (0 << 6);                                              //bit6 DAEC Angle can be read in ANGLECOM
    Settings1 |= ((Config->Mode != AS5047P_MODE_UVW_ABI) << 7);         //bit7 Enables PWM
    Settings2 |= (Config->UVW_PolePairs << 0);                          //bit0-2 UVW pole pairs
    Settings2 |= (Config->Hysteresis << 3);                             //bit3-4 Hysteresis setting
    Settings2 |= ((Config->ABI_Resolution & 0x07) << 5);                //bit5-7 ABI resolution

    /**step3:写入Settings1和Settings2**/
    PRINT_INFO("Writing Settings1 Settings2...\n");
    if (AS5047P_WriteReg(AS5047P, SETTINGS1, Settings1)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, SETTINGS2, Settings2)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Writing Error!\n");
        return Status;
    }
    PRINT_INFO("Writing Success!\n");

    /**step4:Zeroing ZeroPosition**/
    //4.1:Read out the measured angle from the ANGLE register
    PRINT_INFO("Reading Angle...\n");
    uint16_t Angle;
    if (AS5047P_ReadReg(AS5047P, ANGLECOM, &Angle)) {
        Status = AS5047P_PROGRAMMING_READ_ERROR;
        PRINT_INFO("Error:Reading Error!\n");
        return Status;
    }
    ZeroPositionM |= Angle >> 6;                    //bit0-7 8 most significant bits of the zero position
    ZeroPositionL |= Angle & 0x3f;                  //bit0-5 6 least significant bits of the zero position
    ZeroPositionL |= (1 << 6);                      //bit6 Enable MAGH check
    ZeroPositionL |= (1 << 7);                      //bit7 Enable MAGL check

    //4.2:Write the ZPOSM and ZPOSL registers
    PRINT_INFO("Writing ZPOSM ZPOSL...\n");
    if (AS5047P_WriteReg(AS5047P, ZPOSM, ZeroPositionM)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, ZPOSL, ZeroPositionL)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Writing Error!\n");
        return Status;
    }
    PRINT_INFO("Writing Success!\n");

    /**step5:Verify the written data,Verify 1**/
    PRINT_INFO("Verifying 1...\n");
    if (AS5047P_ReadReg(AS5047P, SETTINGS1, &Settings1_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, SETTINGS2, &Settings2_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, ZPOSM, &ZeroPositionM_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, ZPOSL, &ZeroPositionL_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Reading Error!\n");
        return Status;
    }
    //只保留有定义的位
    Settings1_Verify &= 0x00FF;
    Settings2_Verify &= 0x00FF;
    ZeroPositionM_Verify &= 0x00FF;
    ZeroPositionL_Verify &= 0x00FF;
    if (Settings1 != Settings1_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (Settings2 != Settings2_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (ZeroPositionM != ZeroPositionM_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (ZeroPositionL != ZeroPositionL_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Verify Error!\n");
        return Status;
    }
    PRINT_INFO("Verify Success!\n");

    /**step6:Start OTP Programming**/
    PRINT_INFO("Start OTP Programming...\n");
    if (AS5047P_WriteReg(AS5047P, PROG, 0x0001)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, PROG, 0x0008)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:OTP Programming Write Error!\n");
        return Status;
    }
    uint16_t rxData = 0x0000;
    while (rxData != 0x0001) {
        if (AS5047P_ReadReg(AS5047P, PROG, &rxData)) Status = AS5047P_PROGRAMMING_READ_ERROR;
        if (AS5047P_PROGRAMMING_OK != Status) {
            PRINT_INFO("Error:OTP Programming Read Error!\n");
            return Status;
        }
    }
    PRINT_INFO("OTP Programming Finished\n");

    /**step7:Verify the written data,Verify 2**/
    PRINT_INFO("Verifying 2...\n");
    //7.1:Clear the OTP register
    if (AS5047P_WriteReg(AS5047P, SETTINGS1, 0x0000)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, SETTINGS2, 0x0000)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, ZPOSM, 0x0000)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, ZPOSL, 0x0000)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Clear OTP Register Error!\n");
        return Status;
    }
    //7.2:Rewrite the OTP register
    if (AS5047P_WriteReg(AS5047P, PROG, 0x0040)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_WriteReg(AS5047P, PROG, 0x0004)) Status = AS5047P_PROGRAMMING_WRITE_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Rewrite OTP Register Error!\n");
        return Status;
    }
    //7.3:Verify the OTP register
    if (AS5047P_ReadReg(AS5047P, SETTINGS1, &Settings1_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, SETTINGS2, &Settings2_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, ZPOSM, &ZeroPositionM_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_ReadReg(AS5047P, ZPOSL, &ZeroPositionL_Verify)) Status = AS5047P_PROGRAMMING_READ_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Verify OTP Register Error!\n");
        return Status;
    }
    //只保留有定义的位
    Settings1_Verify &= 0x00FF;
    Settings2_Verify &= 0x00FF;
    ZeroPositionM_Verify &= 0x00FF;
    ZeroPositionL_Verify &= 0x00FF;
    if (Settings1 != Settings1_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (Settings2 != Settings2_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (ZeroPositionM != ZeroPositionM_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (ZeroPositionL != ZeroPositionL_Verify) Status = AS5047P_PROGRAMMING_VERIFY_ERROR;
    if (AS5047P_PROGRAMMING_OK != Status) {
        PRINT_INFO("Error:Verify OTP Register Error!\n");
        PRINT_INFO("Settings1:%X Settings1_Verify:%X\n", Settings1, Settings1_Verify);
        PRINT_INFO("Settings2:%X Settings2_Verify:%X\n", Settings2, Settings2_Verify);
        PRINT_INFO("ZeroPositionM:%X ZeroPositionM_Verify:%X\n", ZeroPositionM, ZeroPositionM_Verify);
        PRINT_INFO("ZeroPositionL:%X ZeroPositionL_Verify:%X\n", ZeroPositionL, ZeroPositionL_Verify);
        return Status;
    }
    PRINT_INFO("Verify OTP Register Success!\n");

    PRINT_INFO("OTP Programming Success!\n");

    return Status;
}