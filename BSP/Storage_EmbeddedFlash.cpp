//
// Created by 26757 on 25-4-30.
//

#include "Storage_EmbeddedFlash.h"
#include <algorithm>

// FLASH中存放数据的区域,也用于烧录时擦除这些数据,注意:只读!
// __attribute__((section(".flash_data"))) uint8_t storage_buffer_to_erase[FLASH_PAGE_SIZE * 5];

Storage_EmbeddedFlash storage{0x0801D800, 0x00002800}; // 最后10KB作为储存空间

void Storage_EmbeddedFlash::write_page_bytes(const uint32_t page, const uint32_t addr,
                                             const uint8_t *pdata, uint32_t count) {
    if (count == 0) return;
    // 限制写入范围
    count = std::min(count, FLASH_PAGE_SIZE - addr);

    // 读出当前页数据
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
        page_buffer[i] = *reinterpret_cast<volatile uint8_t *>(FLASH_BASE + page * FLASH_PAGE_SIZE + i);
    }

    for (uint32_t i = 0; i < count; i++) {
        page_buffer[addr + i] = pdata[i];
    }

    HAL_FLASH_Unlock(); //解锁Flash

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // 标明Flash执行页面只做擦除操作
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = page; // 声明要擦除的地址
    EraseInitStruct.NbPages = 1; // 说明要擦除的页数,此参数必须是Min_Data = 1和Max_Data =(最大页数-初始页的值)之间的值
    uint32_t PageError = 0; // 设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
    while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) {} // 调用擦除函数擦除

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / 8; i++) {
        while (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                           FLASH_BASE + page * FLASH_PAGE_SIZE + i * 8,
                                           *reinterpret_cast<uint64_t *>(page_buffer + i * 8))) {}
    }

    HAL_FLASH_Lock(); //锁住Flash
}

void Storage_EmbeddedFlash::write(const uint32_t addr, uint8_t *buff, uint32_t count) {
    // 限制写入范围
    count = std::min(count, storage_size - addr);

    const uint32_t start_page = (STORAGE_ADDRESS_BASE + addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    const uint32_t end_page = (STORAGE_ADDRESS_BASE + addr + count - FLASH_BASE) / FLASH_PAGE_SIZE;
    const uint32_t page_num = end_page - start_page + 1;
    const uint32_t offset = addr % FLASH_PAGE_SIZE;
    if (page_num == 1) {
        write_page_bytes(start_page, offset, buff, count);
    } else {
        for (uint32_t i = 0; i < page_num; i++) {
            if (i == 0)
                write_page_bytes(start_page, offset, buff, FLASH_PAGE_SIZE - offset);
            else if (i == page_num - 1)
                write_page_bytes(start_page + i, 0, buff + FLASH_PAGE_SIZE * i - offset,
                                 count + offset - FLASH_PAGE_SIZE * i);
            else
                write_page_bytes(start_page + i, 0, buff + FLASH_PAGE_SIZE * i - offset, FLASH_PAGE_SIZE);
        }
    }
}

void Storage_EmbeddedFlash::read(const uint32_t addr, uint8_t *buff, uint32_t count) {
    // 限制读取范围
    count = std::min(count, storage_size - addr);

    auto s = reinterpret_cast<uint8_t *>(STORAGE_ADDRESS_BASE + addr);
    for (uint32_t i = 0; i < count; i++) {
        *(buff++) = *(s++);
    }
}
