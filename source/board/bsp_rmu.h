#ifndef __BSP_RMU_H__
#define __BSP_RMU_H__


#include "hc32_ddl.h"


typedef enum {
    RST_CAU_POWER_ON    = 0x01,
    RST_CAU_EXTERNAL    = 0x02,
    RST_CAU_BROWN_OUT   = 0x04,
    RST_CAU_WATCHDOG    = 0x08,
    RST_CAU_JTAG        = 0x10,
    RST_CAU_SOFTWARE    = 0x20,
    RST_CAU_BACKUP      = 0x40,
} rst_cause_t;


uint8_t rmu_get_reset_cause(void);
uint8_t rmu_clear_reset_cause(void);


#endif
