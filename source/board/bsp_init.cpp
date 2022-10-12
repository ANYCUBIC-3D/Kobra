#include "bsp_init.h"
#include "bsp_irq.h"
#include "startup.h"


void clock_init(void)
{
    stc_clk_xtal_cfg_t   stcXtalCfg;
    stc_clk_mpll_cfg_t   stcMpllCfg;
    en_clk_sys_source_t  enSysClkSrc;
    stc_clk_sysclk_cfg_t stcSysClkCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv4;
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv16;
    CLK_SysClkConfig(&stcSysClkCfg);

    CLK_SetPeriClkSource(ClkPeriSrcPclk);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_Lock();

    /* Switch driver ability */
    PWC_HS2HP();

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1u; /* XTAL 8M / 1 */
    stcMpllCfg.plln = 42u;   /* 8M*42 = 336M */
    stcMpllCfg.PllpDiv = 2u; /* MLLP = 168M */
    stcMpllCfg.PllqDiv = 2u; /* MLLQ = 168M */
    stcMpllCfg.PllrDiv = 2u; /* MLLR = 168M */
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

void get_all_clock(void)
{
    stc_clk_freq_t   stcClkFreq;
    stc_pll_clk_freq_t stcPllClkFreq;

	MEM_ZERO_STRUCT(stcClkFreq);
    MEM_ZERO_STRUCT(stcPllClkFreq);

	CLK_GetClockFreq(&stcClkFreq);
	CLK_GetPllClockFreq(&stcPllClkFreq);

	f_cpu_init(stcClkFreq.pclk1Freq);   // used for stepper timer

#if 0
    printf("sysclkFreq: %d\n", stcClkFreq.sysclkFreq);
    printf("hclkFreq:   %d\n", stcClkFreq.hclkFreq);
    printf("exckFreq:   %d\n", stcClkFreq.exckFreq);
    printf("pclk0Freq:  %d\n", stcClkFreq.pclk0Freq);
    printf("pclk1Freq:  %d\n", stcClkFreq.pclk1Freq);
    printf("pclk2Freq:  %d\n", stcClkFreq.pclk2Freq);
    printf("pclk3Freq:  %d\n", stcClkFreq.pclk3Freq);
    printf("pclk4Freq:  %d\n", stcClkFreq.pclk4Freq);

    printf("mpllp:      %d\n", stcPllClkFreq.mpllp);
    printf("mpllq:      %d\n", stcPllClkFreq.mpllq);
    printf("mpllr:      %d\n", stcPllClkFreq.mpllr);
    printf("upllp:      %d\n", stcPllClkFreq.upllp);
    printf("upllq:      %d\n", stcPllClkFreq.upllq);
    printf("upllr:      %d\n", stcPllClkFreq.upllr);
#endif
}

void led_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

    PORT_Init(PortA, Pin01, &stcPortInit);
    PORT_Init(PortA, Pin04, &stcPortInit);

    LED0_OFF();
}

void endstop_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp  = Disable;

#if 0
    PORT_Init(X_MIN_PORT,   X_MIN_PIN,   &stcPortInit);
    PORT_Init(Y_MIN_PORT,   Y_MIN_PIN,   &stcPortInit);
    PORT_Init(Z_MIN_PORT,   Z_MIN_PIN,   &stcPortInit);
    PORT_Init(E0_MIN_PORT,  E0_MIN_PIN,  &stcPortInit);
    PORT_Init(Z_PROBE_PORT, Z_PROBE_PIN, &stcPortInit);
#endif
}

void stepper_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Enable;

#if 0
    PORT_Init(X_ENABLE_PORT,   X_ENABLE_PIN,  &stcPortInit);
    PORT_SetBits(X_ENABLE_PORT, X_ENABLE_PIN);

    stcPortInit.enPullUp  = Disable;

    PORT_Init(X_STEP_PORT,     X_STEP_PIN,    &stcPortInit);
    PORT_Init(X_DIR_PORT,      X_DIR_PIN,     &stcPortInit);

    PORT_Init(Y_ENABLE_PORT,   Y_ENABLE_PIN,  &stcPortInit);
    PORT_Init(Y_STEP_PORT,     Y_STEP_PIN,    &stcPortInit);
    PORT_Init(Y_DIR_PORT,      Y_DIR_PIN,     &stcPortInit);

    PORT_Init(Z_ENABLE_PORT,   Z_ENABLE_PIN,  &stcPortInit);
    PORT_Init(Z_STEP_PORT,     Z_STEP_PIN,    &stcPortInit);
    PORT_Init(Z_DIR_PORT,      Z_DIR_PIN,     &stcPortInit);

    PORT_Init(E0_ENABLE_PORT,  E0_ENABLE_PIN,  &stcPortInit);
    PORT_Init(E0_STEP_PORT,    E0_STEP_PIN,    &stcPortInit);
    PORT_Init(E0_DIR_PORT,     E0_DIR_PIN,     &stcPortInit);
#endif
}

void heater_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

#if 0
    PORT_Init(HEATER_BED_PORT, HEATER_BED_PIN, &stcPortInit);
    PORT_Init(HEATER_0_PORT,   HEATER_0_PIN,   &stcPortInit);

    PORT_ResetBits(HEATER_BED_PORT, HEATER_BED_PIN);
    PORT_ResetBits(HEATER_0_PORT,   HEATER_0_PIN);
#endif
}

void fan_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPullUp  = Disable;

// 0x1C swd on ; 0x1F swd off
    PORT_DebugPortSetting(0x1F, Disable);

#if 0
    PORT_SetFunc(FAN_0_PORT, FAN_0_PIN, Func_Gpio, Disable);
    PORT_SetFunc(FAN_1_PORT, FAN_1_PIN, Func_Gpio, Disable);
    PORT_SetFunc(FAN_2_PORT, FAN_2_PIN, Func_Gpio, Disable);

    PORT_Init(FAN_0_PORT, FAN_0_PIN, &stcPortInit);
    PORT_Init(FAN_1_PORT, FAN_1_PIN, &stcPortInit);
    PORT_Init(FAN_2_PORT, FAN_2_PIN, &stcPortInit);

    FAN_0_OFF();
    FAN_1_OFF();
    FAN_2_OFF();
#endif
}

// PCLK1 max is 100Mhz
// baudrate = clk/(8 ¡Á (2 - over8 ) * (DIV_Integer + 1) )
void uart1_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART1, Enable);
    
    PORT_SetFunc(USART1_RX_PORT, USART1_RX_PIN, USART1_RX_FUNC, Disable);
    PORT_SetFunc(USART1_TX_PORT, USART1_TX_PIN, USART1_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART1_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART1_CH, USART1_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_RI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART1_RIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART1_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_EI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART1_EIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART1_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_TI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART1_TIrqHander;
    stcIrqRegiCfg.enIntSrc = USART1_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART1_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART1_TCIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART1_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART1_CH, UsartRx, Enable);
    USART_FuncCmd(USART1_CH, UsartTx, Enable);
    
    USART_FuncCmd(USART1_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART1_CH, UsartTxCmpltInt, Enable);
}

void uart2_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART2, Enable);
    
    PORT_SetFunc(USART2_RX_PORT, USART2_RX_PIN, USART2_RX_FUNC, Disable);
    PORT_SetFunc(USART2_TX_PORT, USART2_TX_PIN, USART2_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART2_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART2_CH, USART2_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_RI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART2_RIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART2_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_08);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_EI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART2_EIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART2_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_TI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART2_TIrqHander;
    stcIrqRegiCfg.enIntSrc = USART2_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART2_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART2_TCIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART2_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART2_CH, UsartRx, Enable);
    USART_FuncCmd(USART2_CH, UsartTx, Enable);

    USART_FuncCmd(USART2_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART2_CH, UsartTxCmpltInt, Enable);
}

void uart3_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);
    
    PORT_SetFunc(USART3_RX_PORT, USART3_RX_PIN, USART3_RX_FUNC, Disable);
    PORT_SetFunc(USART3_TX_PORT, USART3_TX_PIN, USART3_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART3_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART3_CH, USART3_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_RI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART3_RIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART3_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_EI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART3_EIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART3_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_TI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART3_TIrqHander;
    stcIrqRegiCfg.enIntSrc = USART3_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART3_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART3_TCIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART3_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART3_CH, UsartRx, Enable);
    USART_FuncCmd(USART3_CH, UsartTx, Enable);

    USART_FuncCmd(USART3_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART3_CH, UsartTxCmpltInt, Enable);
}

void uart4_init(void)
{
    en_result_t enRet = Ok;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_16,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART4, Enable);
    
    PORT_SetFunc(USART4_RX_PORT, USART4_RX_PIN, USART4_RX_FUNC, Disable);
    PORT_SetFunc(USART4_TX_PORT, USART4_TX_PIN, USART4_TX_FUNC, Disable);
    
    enRet = USART_UART_Init(USART4_CH, &stcInitCfg);
    if (enRet != Ok) {
        while(1);
    }
    
    enRet = USART_SetBaudrate(USART4_CH, USART4_BAUDRATE);
    if (enRet != Ok) {
        while(1);
    }

    stc_irq_regi_conf_t stcIrqRegiCfg;
    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_RI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART4_RIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART4_INT_RI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_EI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART4_EIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART4_INT_EI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_TI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART4_TIrqHander;
    stcIrqRegiCfg.enIntSrc = USART4_INT_TI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = IRQ_INDEX_USART4_INT_TCI;
    stcIrqRegiCfg.pfnCallback = &BSP_USART4_TCIIrqHander;
    stcIrqRegiCfg.enIntSrc = USART4_INT_TCI;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    USART_FuncCmd(USART4_CH, UsartRx, Enable);
    USART_FuncCmd(USART4_CH, UsartTx, Enable);

    USART_FuncCmd(USART4_CH, UsartRxInt, Enable);
//  USART_FuncCmd(USART3_CH, UsartTxCmpltInt, Enable);
}



