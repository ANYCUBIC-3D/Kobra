/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"
#include "bsp_rmu.h"


// ------------------------
// Public Variables
// ------------------------

uint16_t HAL_adc_result;

// HAL initialization task
void HAL_init() {
  NVIC_SetPriorityGrouping(0x3);
  FastIO_init();

  #if ENABLED(SDSUPPORT) && DISABLED(SDIO_SUPPORT) && (defined(SDSS) && SDSS != -1)
    OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if PIN_EXISTS(AUTO_LEVEL_TX)
    OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
    delay(10);
    OUT_WRITE(AUTO_LEVEL_TX_PIN, LOW);
    delay(300);
    OUT_WRITE(AUTO_LEVEL_TX_PIN, HIGH);
  #endif

  //SetTimerInterruptPriorities();

  #if ENABLED(EMERGENCY_PARSER) && USBD_USE_CDC
    USB_Hook_init();
  #endif
}

void HAL_clear_reset_source()
{
    rmu_clear_reset_cause();
}

uint8_t HAL_get_reset_source()
{
    uint8_t res;

    res = rmu_get_reset_cause();

    return res;
}

void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

uint32_t AD_DMA[3];

// ------------------------
// ADC
// ------------------------
// Init the AD in continuous capture mode
void HAL_adc_init() {}
//TODO: Make sure this doesn't cause any delay
void HAL_adc_start_conversion(const uint8_t adc_pin) {
        if(adc_pin>BOARD_NR_GPIO_PINS)return;
        uint8_t channel = PIN_MAP[adc_pin].adc_channel;
        DDL_ASSERT(channel!=ADC_PIN_INVALID);
        HAL_adc_result = adc_read(ADC1,channel);
        switch(adc_pin)
        {
            case TEMP_BED_PIN: AD_DMA[0] = HAL_adc_result;break;
            case TEMP_0_PIN: AD_DMA[1] = HAL_adc_result;break;
            case POWER_MONITOR_VOLTAGE_PIN: AD_DMA[2] = HAL_adc_result;break;
            default:break;
        }
}
uint16_t HAL_adc_get_result() {return 1000;} // { return HAL_adc_result; }

// Reset the system (to initiate a firmware flash)
void flashFirmware(const int16_t) { NVIC_SystemReset(); }


