

#ifdef ARDUINO_ARCH_STM32F1

#include "../../inc/MarlinConfig.h" // Allow pins/pins.h to set density

bool SDIO_Init() {
	return (steup_sdio());
}

bool SDIO_ReadBlock(uint32_t blockAddress, uint8_t *data) {
	uint32_t retries = 3;
	while (retries--) if (SDIO_ReadBlock_DMA(blockAddress, data)) return true;
	return false;
}

bool SDIO_WriteBlock(uint32_t blockAddress, const uint8_t *data) {
	return SDIO_WriteBlockDMA(blockAddress,data);
}

#endif // ARDUINO_ARCH_STM32F1
