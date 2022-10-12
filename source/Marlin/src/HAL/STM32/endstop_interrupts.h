/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
#pragma once

#include "../../module/endstops.h"

// One ISR for all EXT-Interrupts
inline void endstop_ISR() { endstops.update(); }

inline void setup_endstop_interrupts() {
  #if HAS_X_MIN
    attachInterrupt(X_MIN_PIN, ExtInt_X_MIN_Callback, 0, CHANGE);
  #endif
  #if HAS_Y_MIN
    attachInterrupt(Y_MIN_PIN, ExtInt_Y_MIN_Callback, 1, CHANGE);
  #endif
  #if HAS_Z_MIN
    attachInterrupt(Z_MIN_PIN, ExtInt_Z_MIN_Callback, 2, CHANGE);
  #endif
  #if HAS_Z2_MIN
    attachInterrupt(Z2_MIN_PIN, ExtInt_Z2_MIN_Callback, 3, CHANGE);
  #endif
  #if HAS_Z_MIN_PROBE_PIN
    attachInterrupt(Z_MIN_PROBE_PIN, ExtInt_Z_MIN_PROBE_Callback, 4, CHANGE);
  #endif
}
