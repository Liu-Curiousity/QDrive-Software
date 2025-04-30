/**
 * @brief 		a library for storage datas using embedded flash
 * @detail
 * @author 	    Haoqi Liu
 * @date        25-4-22
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		history
                V1.0.0 on 25-4-22
 * */

#ifndef STORAGE_H
#define STORAGE_H

#include <cstdint>

class Storage {
public:
    virtual ~Storage() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    bool initialized = false;

    virtual void init() = 0;

    /**
     * @brief 向储存区写入count个字节
     * @param addr 储存区地址(类似于VMA)
     * @param buff 写入缓冲区
     * @param count 写入字节数
     */
    virtual void write(uint32_t addr, uint8_t *buff, uint32_t count) = 0;

    /**
     * @brief 从储存区读出count个字节
     * @param addr 储存区地址(类似于VMA)
     * @param buff 读出缓冲区
     * @param count 读取字节数
     */
    virtual void read(uint32_t addr, uint8_t *buff, uint32_t count) = 0;
};

#endif //STORAGE_H
