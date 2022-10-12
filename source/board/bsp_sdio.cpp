#include "bsp_sdio.h"


uint32_t au32WriteBlocks[512];
uint32_t au32ReadBlocks[512];


stc_sdcard_init_t stcCardInitCfg =
{
    SdiocBusWidth4Bit,
    (en_sdioc_clk_freq_t)16000000u,
    SdiocHighSpeedMode,
    NULL,
};

stc_sdcard_dma_init_t stcDmaInitCfg =
{
    M4_DMA1,
    DmaCh0,
};

stc_sd_handle_t stcSdhandle = 
{
    SDIOC_UNIT,
};


void hal_sdio_init()
{
    SdiocInitPins();
    sdio_controller_init();
}

void sdio_controller_init()
{
    en_result_t enTestResult = Ok;

    stcSdhandle.SDIOCx = SDIOC_UNIT;
    stcSdhandle.enDevMode = SdCardDmaMode;
    stcSdhandle.pstcDmaInitCfg = &stcDmaInitCfg;
    if (Ok != SDCARD_Init(&stcSdhandle, &stcCardInitCfg))
    {
        enTestResult = Error;
    }
}

static en_result_t SdiocInitPins(void)
{
    PORT_SetFunc(SDIOC_D0_PORT, SDIOC_D0_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D1_PORT, SDIOC_D1_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D2_PORT, SDIOC_D2_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D3_PORT, SDIOC_D3_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CD_PORT, SDIOC_CD_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CK_PORT, SDIOC_CK_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CMD_PORT, SDIOC_CMD_PIN, Func_Sdio, Disable);

    return Ok;
}

uint8_t sdio_write(uint32_t blockAddress, const uint8_t *data)
{
    en_result_t result = SDCARD_WriteBlocks(&stcSdhandle, blockAddress, 1, (uint8_t *)data, 20000);

    if(result == Ok) {
        return true;
    } else {
        return false;
    }
}

uint8_t sdio_read(uint32_t blockAddress, uint8_t *data)
{
    en_result_t result = SDCARD_ReadBlocks(&stcSdhandle, blockAddress, 1, (uint8_t *)data, 20000);

    if(result == Ok) {
        return true;
    } else {
        return false;
    }
}

void sdio_raw_test()
{
    en_result_t enTestResult = Ok;

    /* Erase SD card */
    if (Ok != SDCARD_Erase(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, 20000u))
    {
        enTestResult = Error;
        printf("SDCARD_Erase error\n");
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32ReadBlocks, 2000u))
    {
        enTestResult = Error;
        printf("SDCARD_ReadBlocks error\n");
    }

    printf("erased: \n");
    for (uint16_t i = 0u; i < ARRAY_SZ(au32WriteBlocks); i++)
    {
        printf("%02X ", au32ReadBlocks[i]);
    }
    printf("\n");

    for (uint16_t i = 0u; i < ARRAY_SZ(au32WriteBlocks); i++)
    {
        au32WriteBlocks[i] = (uint8_t)i;
    }

    /* Write SD card */
    if (Ok != SDCARD_WriteBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32WriteBlocks, 2000u))
    {
        enTestResult = Error;
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32ReadBlocks, 20000u))
    {
        enTestResult = Error;
    }

    printf("writed: \n");
    for (uint16_t i = 0u; i < ARRAY_SZ(au32WriteBlocks); i++)
    {
        printf("%02X ", au32ReadBlocks[i]);
    }
    printf("\n");

    /* Compare read/write data */
    if (0 != memcmp(au32WriteBlocks, au32ReadBlocks, sizeof(au32ReadBlocks)))
    {
        enTestResult = Error;
    }

    if (Ok == enTestResult) {
        printf("test ok\n");
    } else {
        printf("test ng\n");
    }
}




