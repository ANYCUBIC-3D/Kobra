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

/**
 * Fast I/O interfaces for STM32
 * These use GPIO register access for fast port manipulation.
 */

// ------------------------
// Public Variables
// ------------------------

//extern GPIO_TypeDef * FastIOPortMap[];

// ------------------------
// Public functions
// ------------------------

void FastIO_init(); // Must be called before using fast io macros

// ------------------------
// Defines
// ------------------------

#define _BV32(b) (1UL << (b))

#ifndef PWM
  #define PWM OUTPUT
#endif


#define _GET_MODE(IO)           gpio_get_mode(IO)
#define _SET_MODE(IO,M)         gpio_set_mode(IO,M)
#define _SET_OUTPUT(IO)         _SET_MODE(IO, OUTPUT)                             //!< Output Push Pull Mode & GPIO_NOPULL

#define WRITE(IO,V)             (V>0? PORT_SetBitsMapp(IO) : PORT_ResetBitsMapp(IO))
#define READ(IO)                (PORT_GetBitMapp(IO) ? HIGH : LOW)
#define TOGGLE(IO)              (PORT_ToggleMapp(IO))

#define OUT_WRITE(IO,V)         do{ _SET_OUTPUT(IO); WRITE(IO,V); }while(0)

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              //!< Input Floating Mode
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       //!< Input with Pull-up activation
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     //!< Input with Pull-down activation
#define SET_OUTPUT(IO)          OUT_WRITE(IO, LOW)

#define IS_INPUT(IO)
#define IS_OUTPUT(IO)

#define PWM_PIN(P)              0//digitalPinHasPWM(P)
#define NO_COMPILE_TIME_PWM

// digitalRead/Write wrappers
#define extDigitalRead(IO)    digitalRead(IO)
#define extDigitalWrite(IO,V) digitalWrite(IO,V)
