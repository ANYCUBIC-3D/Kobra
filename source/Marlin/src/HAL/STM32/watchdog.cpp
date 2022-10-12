/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfig.h"

#if ENABLED(USE_WATCHDOG)

#include "../cores/iwdg.h"
#include "watchdog.h"

bool wdt_init_flag = false;
void HAL_watchdog_refresh() {
    if(!wdt_init_flag)return;
  iwdg_feed();
}

void watchdogSetup() {
  // do whatever. don't remove this function.
}

/**
 * @brief  Initialized the independent hardware watchdog.
 *
 * @return No return
 *
 * @details The watchdog clock is 40Khz. We need a 4 seconds interval, so use a /256 preescaler and 625 reload value (counts down to 0)
 */
void watchdog_init() {
  iwdg_init();
  wdt_init_flag = true;
}

#endif // USE_WATCHDOG
#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
