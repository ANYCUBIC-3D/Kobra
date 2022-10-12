#ifndef _BSP_TIM0_H_
#define _BSP_TIM0_H_


#include <stdint.h>

#include "hc32_ddl.h"
#include "hc32f46x_clk.h"
#include "hc32f46x_efm.h"
#include "hc32f46x_utility.h"
#include "hc32f46x_sram.h"
#include "hc32f46x_interrupts.h"
#include "hc32f46x_pwc.h"


#ifdef __cplusplus
extern "C"{
#endif

/* Define Timer Unit for example */
#define TMR_UNIT            (M4_TMR02)
#define TMR_INI_GCMA        (INT_TMR02_GCMA)
#define TMR_INI_GCMB        (INT_TMR02_GCMB)
#define OS_TICKS_PER_SEC   1000ul

#define TIMER4_UNIT                     (M4_TMR41)


extern void setup_systick(void);
extern void setup_time2A(const uint32_t frequency);
extern void setup_time2B(const uint32_t frequency);
extern void setup_time4(const uint32_t frequency);
extern void setup_time(void);
void setup_step_tim(const uint32_t frequency);
void setup_temp_tim(const uint32_t frequency);




extern uint32_t millis(void);
extern en_result_t timer_preset_compare(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh,const uint16_t compare,en_functional_state_t counterclr);
extern void tone_set_compare(const uint16_t compare);
extern en_result_t timer_set_compare(const uint8_t timer_num,const uint16_t compare);
extern uint16_t timer_get_count(const uint8_t timer_num);
extern void timer_enable_irq(const uint8_t timer_num,en_functional_state_t state); 
extern bool timer_irq_enabled(M4_TMR0_TypeDef* pstcTim0Reg, const uint8_t timer_num); 

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);


#define os_get_timeslice(x,y)  ((uint32_t)(x>=y?(x-y):0))


extern volatile uint32_t _millis;



#ifdef __cplusplus
}
#endif

#endif




