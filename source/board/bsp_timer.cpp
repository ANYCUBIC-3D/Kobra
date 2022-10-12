#include "hc32f46x.h"

#include "bsp_timer.h"
#include "bsp_irq.h"
#include "bsp_init.h"
#include "SoftwareSerial.h"


extern volatile uint32_t _millis;

void Timer02A_CallBack(void)
{
//    PORT_Toggle(PortA, Pin01);
    _millis++;
}

void timer02A_init(void)
{
    stc_clk_freq_t stcClkTmp;

    uint32_t u32Pclk1;

    stc_tim0_base_init_t stcTimerCfg;

    MEM_ZERO_STRUCT(stcTimerCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

// Get pclk1
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv1024;
    stcTimerCfg.Tim0_CmpValue = (uint16_t)(u32Pclk1/1024ul/1000);
    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelA, &stcTimerCfg);

    stc_irq_regi_conf_t stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TMR02_GCMA;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMR02_GCMA;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer02A_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    TIMER0_IntCmd(TMR_UNIT, Tim0_ChannelA, Enable);

    TIMER0_Cmd(TMR_UNIT, Tim0_ChannelA, Enable);
}

extern void Timer01B_CallBack(void);
//void Timer01B_CallBack(void)
//{
//    PORT_Toggle(PortA, Pin04);
//}

void timer01B_init(void)
{
    stc_clk_freq_t stcClkTmp;

    uint32_t u32Pclk1;

    stc_tim0_base_init_t stcTimerCfg;

    MEM_ZERO_STRUCT(stcTimerCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM01, Enable);

// Get pclk1
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv1024;
    stcTimerCfg.Tim0_CmpValue = (uint16_t)(u32Pclk1/1024ul/1000);
    TIMER0_BaseInit(M4_TMR01, Tim0_ChannelB, &stcTimerCfg);

    stc_irq_regi_conf_t stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TMR01_GCMB;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMR01_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer01B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    TIMER0_IntCmd(M4_TMR01, Tim0_ChannelB, Enable);

//    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_set_overflow(uint16_t ms)
{
    uint32_t u32Pclk1;
    stc_clk_freq_t stcClkTmp;

    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    M4_TMR01->CMPBR_f.CMPB = (uint16_t)(u32Pclk1/1024ul/1000*ms);

    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_enable(void)
{
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Enable);
}

void timer01B_disable(void)
{
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Disable);
    M4_TMR01->CNTBR = 0;
}

extern void Timer02B_CallBack(void);

void timer02B_init(void)
{
    stc_clk_freq_t stcClkTmp;

    uint32_t u32Pclk1;

    stc_tim0_base_init_t stcTimerCfg;

    MEM_ZERO_STRUCT(stcTimerCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

// Get pclk1
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;

    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv0;
//    stcCntInit.u16Cycle = get_pclk1Freq() / (1 << stcCntInit.enClkDiv) * period;
    stcTimerCfg.Tim0_CmpValue = (uint16_t)(290);
    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelB, &stcTimerCfg);

    stc_irq_regi_conf_t stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TMR02_GCMB;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMR02_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer02B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    TIMER0_IntCmd(TMR_UNIT, Tim0_ChannelB, Enable);

    TIMER0_Cmd(TMR_UNIT, Tim0_ChannelB, Disable);
}

uint32_t get_pclk1Freq(void)
{
    stc_clk_freq_t stcClkTmp;

    CLK_GetClockFreq(&stcClkTmp);

    return (stcClkTmp.pclk1Freq);
}

void set_timer_overflow(uint16_t overflow)
{
//    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelB, &stcTimerCfg);
    M4_TMR02->CMPBR_f.CMPB = overflow;
}

void set_timer_clk_prescale(en_tim0_clock_div div)
{

//    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelB, &stcTimerCfg);
}

// moved to hal/stm32/timers.h
//void timer41_zero_match_irq_cb(void)
//{
//    static uint32_t u32IrqCnt = 0ul;

//    PORT_Toggle(PortA, Pin01);

//    TIMER4_CNT_ClearIrqFlag(M4_TMR41, Timer4CntZeroMatchInt);
//}

#define TIMER41_CNT_CYCLE_VAL 65535

void timer41_init(void)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41, Enable);

    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;
//    stcCntInit.u16Cycle = get_pclk1Freq() / (1 << stcCntInit.enClkDiv) * period;
//    stcCntInit.u16Cycle = 100000000UL / (1 << 1) * 0.001 = 50000; // 1ms
//    stcCntInit.u16Cycle = 84000000UL / (1 << 1) * 0.001 = 42000; // 1ms
    stcCntInit.u16Cycle = 42000; // 1 ms
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Enable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR41, &stcCntInit);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_TMR41_GCMB;
    stcIrqRegiCfg.pfnCallback = &timer41_zero_match_irq_cb;
    stcIrqRegiCfg.enIntSrc = INT_TMR41_GUDF;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    TIMER4_CNT_ClearCountVal(M4_TMR41);
    TIMER4_CNT_Start(M4_TMR41);
}

// moved to hal/stm32/timers.h
//void timer42_zero_match_irq_cb(void)
//{
//    static uint32_t u32IrqCnt = 0ul;

//    (++u32IrqCnt & 0x00000001ul) ? LED_ON() : LED_OFF();
//    PORT_Toggle(PortA, Pin04);

//    TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt);
//}

#define TIMER42_CNT_CYCLE_VAL 65534

void timer42_init(void)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM42, Enable);

    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv8;
    stcCntInit.u16Cycle = TIMER42_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Enable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR42, &stcCntInit);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_INT_TMR42_GCMB;
    stcIrqRegiCfg.pfnCallback = &timer42_zero_match_irq_cb;
    stcIrqRegiCfg.enIntSrc = INT_TMR42_GUDF;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_00);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    TIMER4_CNT_ClearCountVal(M4_TMR42);
    TIMER4_CNT_Start(M4_TMR42);
}

void timer42_init_check(void)
{
    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);

    TIMER4_CNT_Load(M4_TMR42, &stcCntInit);
}

void timer42_set_frequency(const uint32_t frequency)
{
    uint32_t u32Cycle = 0;
    uint8_t u8ClkDiv = 0;

    stc_timer4_cnt_init_t stcCntInit;

    MEM_ZERO_STRUCT(stcCntInit);

    TIMER4_CNT_Load(M4_TMR42, &stcCntInit);

#if 0
    stcCntInit.enClkDiv = Timer4CntPclkDiv1;

    u32Cycle = get_pclk1Freq() / (2 << stcCntInit.enClkDiv) / frequency;
    u8ClkDiv = (uint8_t)stcCntInit.enClkDiv;

    while(u32Cycle > 65534UL) {
        u8ClkDiv++;
        stcCntInit.enClkDiv = (en_timer4_cnt_clk_div_t)u8ClkDiv;
        u32Cycle = get_pclk1Freq() / (2 << stcCntInit.enClkDiv) / frequency;
        printf("enClkDiv: %d\n", stcCntInit.enClkDiv);
    }

    stcCntInit.u16Cycle = (uint16_t)u32Cycle;

    printf("u16Cycle: %d\n", stcCntInit.u16Cycle);

#else   // use fixed clock division
// STEPPER_TIMER_PRESCALE = 16

    stcCntInit.enClkDiv = Timer4CntPclkDiv16;
    stcCntInit.u16Cycle = get_pclk1Freq() / (1 << Timer4CntPclkDiv16) / frequency;

    printf("u16Cycle: %d\n", stcCntInit.u16Cycle);
#endif

    TIMER4_CNT_Init(M4_TMR42, &stcCntInit);

    TIMER4_CNT_Start(M4_TMR42);
}

void timer42_irq_ctrl(en_functional_state_t state)
{
    if(state == Enable) {
        NVIC_EnableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
    } else if(state == Disable) {
        NVIC_DisableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
    }
}

bool timer42_irq_get()
{
    return (bool)NVIC_GetEnableIRQ(IRQ_INDEX_INT_TMR42_GCMB);
}

bool timer42_set_compare(const uint16_t compare)
{
    TIMER4_CNT_SetCycleVal(M4_TMR42, compare);
}

uint16_t timer42_get_count()
{
    return TIMER4_CNT_GetCountVal(M4_TMR42);
}






