#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__


#include "hc32_ddl.h"


#define TMR_UNIT            (M4_TMR02)
#define TMR_INI_GCMA        (INT_TMR02_GCMA)
#define TMR_INI_GCMB        (INT_TMR02_GCMB)


#define TMR_SSERIAL_UNIT    (M4_TMR02)
#define TMR_SSERIAL_CH      (Tim0_ChannelB)


#define TMR_SSERIAL_STOP()  TIMER0_Cmd(TMR_SSERIAL_UNIT, TMR_SSERIAL_CH, Disable)
#define TMR_SSERIAL_RESUM() TIMER0_Cmd(TMR_SSERIAL_UNIT, TMR_SSERIAL_CH, Enable)


uint32_t get_pclk1Freq(void);

void timer02A_init(void);
void timer02B_init(void);
void timer01B_init(void);
void timer01B_set_overflow(uint16_t ms);
void timer01B_enable(void);
void timer01B_disable(void);

void timer41_init(void);

extern void timer41_zero_match_irq_cb(void);
extern void timer42_zero_match_irq_cb(void);
void timer42_init(void);
void timer42_init_check(void);
void timer42_set_frequency(const uint32_t frequency);
void timer42_irq_ctrl(en_functional_state_t state);
bool timer42_irq_get(void);
bool timer42_set_compare(const uint16_t compare);
uint16_t timer42_get_count(void);


#endif

