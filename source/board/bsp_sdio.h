#ifndef __BSP_SDIO_H__
#define __BSP_SDIO_H__


#include "hc32_ddl.h"
#include "sd_card.h"


#define SDIOC_CD_PORT                   (PortA)
#define SDIOC_CD_PIN                    (Pin10)

#define SDIOC_D0_PORT                   (PortC)
#define SDIOC_D0_PIN                    (Pin08)

#define SDIOC_D1_PORT                   (PortC)
#define SDIOC_D1_PIN                    (Pin09)

#define SDIOC_D2_PORT                   (PortC)
#define SDIOC_D2_PIN                    (Pin10)

#define SDIOC_D3_PORT                   (PortC)
#define SDIOC_D3_PIN                    (Pin11)

#define SDIOC_CK_PORT                   (PortC)
#define SDIOC_CK_PIN                    (Pin12)

#define SDIOC_CMD_PORT                  (PortD)
#define SDIOC_CMD_PIN                   (Pin02)


/* SD sector && count */
#define SD_SECTOR_START                 (0u)
#define SD_SECTOR_COUNT                 (4u)

/* SDIOC unit */
#define SDIOC_UNIT                      (M4_SDIOC1)


extern stc_sd_handle_t stcSdhandle;


void hal_sdio_init();

void sdio_controller_init();

static en_result_t SdiocInitPins(void);
uint8_t sdio_write(uint32_t blockAddress, const uint8_t *data);
uint8_t sdio_read(uint32_t blockAddress, uint8_t *data);

void sdio_raw_test();



#endif

