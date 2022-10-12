#ifndef FLASH_H
#define FLASH_H


#include <stdint.h>

#include "hc32_ddl.h"


#ifdef __cplusplus
extern "C" {
#endif


// use 256k to improve compatibility
// 32 sectors, 8k bytes per sector
#define FLASH_SECTOR_TOTAL    32
#define FLASH_SECTOR_SIZE     ((uint32_t)(8*1024U))
#define FLASH_ALL_START       0
#define FLASH_ALL_END         ((uint32_t)0x0003FFFFU)

// use last sector to emulate eeprom
#define FLASH_EEPROM_BASE     ((uint32_t)0x0003E000U)


#define FLASH_BASE            ((uint32_t)0x0003E000U)              /*!< FLASH base address in the alias region */

// just use 1k bytes, not a full sector
#define EEPROM_SIZE           1024


// power outage
#define FLASH_OUTAGE_DATA_ADDR  ((uint32_t)0x0003C000U)



namespace Intflash {
    void eeprom_buffer_fill();
    uint8_t eeprom_buffered_read_byte(const uint32_t pos);
    void eeprom_buffered_write_byte(uint32_t pos, uint8_t value);
    uint8_t eeprom_read_byte(const uint32_t pos);
    void eeprom_write_byte(uint32_t pos, uint8_t value);
    void eeprom_buffer_flush();
     uint32_t Flash_Updata(uint32_t flashAddr, const void * dataBuf, uint16_t length);
     en_result_t FlashErasePage(uint32_t u32Addr);
}

#ifdef __cplusplus
}
#endif

#endif 

