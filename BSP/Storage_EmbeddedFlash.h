/**
 * @brief 		Storage_EmbeddedFlash.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        25-4-30
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于25-4-30
 * */

#ifndef STORAGE_EMBEDDEDFLASH_H
#define STORAGE_EMBEDDEDFLASH_H

#include "Storage.h"
#include "main.h"

class Storage_EmbeddedFlash final : public Storage {
public:
    Storage_EmbeddedFlash(const uint32_t STORAGE_ADDRESS_BASE, const uint32_t StorageSize) :
        Storage(StorageSize), STORAGE_ADDRESS_BASE(STORAGE_ADDRESS_BASE) {}

    void init() override { initialized = true; }

    void write(uint32_t addr, uint8_t *buff, uint32_t count) override;

    void read(uint32_t addr, uint8_t *buff, uint32_t count) override;

private:
    const uint32_t STORAGE_ADDRESS_BASE; // 存储起始地址
    inline static uint8_t page_buffer[FLASH_PAGE_SIZE]{};

    /**
     * @brief 往页中写入数据
     * @param page 第几页
     * @param addr 当前页的相对地址
     * @param pdata 待写入数据
     * @param count 写入数据的长度
     */
    static void write_page_bytes(uint32_t page, uint32_t addr, const uint8_t *pdata, uint32_t count);
};

extern Storage_EmbeddedFlash storage;
extern Storage_EmbeddedFlash DFU_storage;

#endif //STORAGE_EMBEDDEDFLASH_H
