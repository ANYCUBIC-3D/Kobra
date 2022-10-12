#ifndef _BSP_H
#define  _BSP_H


#include "string.h"
//#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>

//#define ENABLE_TYPE_007

#include "hc32_ddl.h"
#include "../core/macros.h"
#include "../core/boards.h"

#include "board_tim0.h"
#include "board_cfg.h" 
#include "board_gpio.h" 
#include "board_adc.h"
#include "../pins/pins.h"
#include "board_sdio.h"
#include "../cores/usart.h"
#include "../../arduino/HardwareSerial.h"

#include "../cores/libmaple_types.h"
#include "../cores/wirish_types.h"
#include "../cores/flash.h"

#ifdef __cplusplus
extern "C"{
#endif

extern uint32_t F_CPU;
#define CYCLES_PER_MICROSECOND  (F_CPU / 1000000UL)

typedef enum ExtIntTriggerMode {
    RISING, /**< To trigger an interrupt when the pin transitions LOW
                 to HIGH */
    FALLING, /**< To trigger an interrupt when the pin transitions
                  HIGH to LOW */
    CHANGE /**< To trigger an interrupt when the pin transitions from
                LOW to HIGH or HIGH to LOW (i.e., when the pin
                changes). */
} ExtIntTriggerMode;

static inline __always_inline void nvic_globalirq_enable() {
    asm volatile("cpsie i");
}
static inline __always_inline void nvic_globalirq_disable() {
    asm volatile("cpsid i");
}
static inline __always_inline void interrupts() {
    nvic_globalirq_enable();
}
static inline __always_inline void noInterrupts() {
    nvic_globalirq_disable();
}
void init(void);
void f_cpu_init(uint32_t clock);
extern void setup_Extinterrupt(void);
extern void attachInterrupt(uint8 pin, voidFuncPtr handler, uint8 irqNum,ExtIntTriggerMode mode);
extern void ExtInt_X_MIN_Callback(void);
extern void ExtInt_Y_MIN_Callback(void);
extern void ExtInt_Z_MIN_Callback(void);
extern void ExtInt_Z2_MIN_Callback(void);
extern void ExtInt_Z_MIN_PROBE_Callback(void);



#ifdef __cplusplus
} // extern "C"
#endif

#endif

