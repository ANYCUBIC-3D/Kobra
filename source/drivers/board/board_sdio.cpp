#include "board_sdio.h"
#include "bsp_sdio.h"


en_result_t sdio_main(void)
{

}

bool steup_sdio(void)
{
    hal_sdio_init();
    return true;
}

bool SDIO_ReadBlock_DMA(uint32_t blockAddress, uint8_t *data)
{
    return sdio_read(blockAddress, data);
}

bool SDIO_WriteBlockDMA(uint32_t blockAddress, const uint8_t *data)
{
    return sdio_write(blockAddress, (const uint8_t *)data);
}


