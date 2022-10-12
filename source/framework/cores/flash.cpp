#include "flash.h"

namespace Intflash {

// just use 1k bytes, not a full sector
static uint8_t eeprom_buffer[EEPROM_SIZE] __attribute__((aligned(8))) = {0};

    
en_result_t FlashErasePage(uint32_t u32Addr)
{
    en_result res = Error;

    if(u32Addr < FLASH_OUTAGE_DATA_ADDR) {
        printf("can NOT erase code area.\n");
        return res;
    }

    EFM_Unlock();
    EFM_FlashCmd(Enable);

    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));

    res = EFM_SectorErase(u32Addr);
    if(res != Ok) {
        printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
    }

    EFM_Lock();

    return res;
}

uint32_t Flash_Updata(uint32_t flashAddr, const void * dataBuf, uint16_t length)
{
    en_result res = Error;
    uint32_t addr = flashAddr;
    uint32_t addr_end = flashAddr + length;
    uint32_t offset = 0;

    EFM_Unlock();
    EFM_FlashCmd(Enable);

    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));

    res = EFM_SectorErase(flashAddr);
    if(res != Ok) {
        printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
    }

    while(addr < addr_end) {
        res = EFM_SingleProgram(addr, *((uint32_t *)((uint8_t *)dataBuf + offset)));
        if(res != Ok) {
            printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
            break;
        }
        addr += 4;
        offset += 4;
    }

    EFM_Lock();

    return res;
}

void eeprom_buffer_fill()
{
    memcpy(eeprom_buffer, (uint8_t *)(FLASH_EEPROM_BASE), sizeof(eeprom_buffer));
}

void eeprom_buffer_flush()
{
    en_result res = Error;
    uint32_t addr = FLASH_EEPROM_BASE;
    uint32_t addr_end = FLASH_EEPROM_BASE + EEPROM_SIZE;
    uint32_t offset = 0;

    EFM_Unlock();
    EFM_FlashCmd(Enable);

    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));

    res = EFM_SectorErase(FLASH_EEPROM_BASE);
    if(res != Ok) {
        printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
    }

    while(addr < addr_end) {
        res = EFM_SingleProgram(addr, *((uint32_t *)((uint8_t *)eeprom_buffer + offset)));
        if(res != Ok) {
            printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
            break;
        }
        addr += 4;
        offset += 4;
    }

    EFM_Lock();
}

uint8_t eeprom_read_byte(const uint32_t pos)
{
    eeprom_buffer_fill();
    return eeprom_buffered_read_byte(pos);
}

void eeprom_write_byte(uint32_t pos, uint8_t value)
{
    eeprom_buffered_write_byte(pos, value);
    eeprom_buffer_flush();
}

uint8_t eeprom_buffered_read_byte(const uint32_t pos)
{
    return eeprom_buffer[pos];
}

void eeprom_buffered_write_byte(uint32_t pos, uint8_t value)
{
    eeprom_buffer[pos] = value;
}


}

