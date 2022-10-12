#include "bsp_adc.h"
#include "bsp_irq.h"


#define ADC_CH_COUNT    3


uint16_t g_adc_value[3];
uint8_t g_adc_idx;


void adc_init(void)
{
    AdcInitConfig();
    adc_pin_init();
    AdcChannelConfig();
    AdcTriggerConfig();

    adc_dma_config();

    ADC_StartConvert(M4_ADC1);
}

static void adc_pin_init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPullUp  = Disable;
    stcPortInit.enPinMode = Pin_Mode_Ana;

    PORT_Init(BOARD_ADC_CH0_PORT, BOARD_ADC_CH0_PIN, &stcPortInit);
    PORT_Init(BOARD_ADC_CH1_PORT, BOARD_ADC_CH1_PIN, &stcPortInit);
    PORT_Init(BOARD_ADC_CH2_PORT, BOARD_ADC_CH2_PIN, &stcPortInit);
}


/**
 *******************************************************************************
 ** \brief  ADC clock configuration.
 **
 ** \note   1) ADCLK max frequency is 60MHz.
 **         2) If PCLK2 and PCLK4 are selected as the ADC clock,
 **            the following conditions must be met:
 **            a. ADCLK(PCLK2) max 60MHz;
 **            b. PCLK4 : ADCLK = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4
 **
 ******************************************************************************/
static void AdcClockConfig(void)
{
#if (ADC_CLK == ADC_CLK_PCLK)
    stc_clk_sysclk_cfg_t stcSysclkCfg;

    /* Set bus clock division, depends on the system clock frequency. */
    stcSysclkCfg.enHclkDiv  = ClkSysclkDiv1;  // 168MHz
    stcSysclkCfg.enExclkDiv = ClkSysclkDiv2;  // 84MHz
    stcSysclkCfg.enPclk0Div = ClkSysclkDiv1;  // 168MHz
    stcSysclkCfg.enPclk1Div = ClkSysclkDiv2;  // 84MHz
    stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // 42MHz
    stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // 42MHz
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv1;  // 84MHz.
    CLK_SysClkConfig(&stcSysclkCfg);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (ADC_CLK == ADC_CLK_MPLLQ)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;

    if (CLKSysSrcMPLL == CLK_GetSysClkSource())
    {
        /*
         * Configure MPLLQ(same as MPLLP and MPLLR) when you
         * configure MPLL as the system clock.
         */
    }
    else
    {
        /* Use XTAL as MPLL source. */
        stcXtalCfg.enFastStartup = Enable;
        stcXtalCfg.enMode = ClkXtalModeOsc;
        stcXtalCfg.enDrv  = ClkXtalLowDrv;
        CLK_XtalConfig(&stcXtalCfg);
        CLK_XtalCmd(Enable);

        /* Set MPLL out 240MHz. */
        stcMpllCfg.pllmDiv = 1u;
        /* mpll = 8M / pllmDiv * plln */
        stcMpllCfg.plln    = 30u;
        stcMpllCfg.PllpDiv = 16u;
        stcMpllCfg.PllqDiv = 16u;
        stcMpllCfg.PllrDiv = 16u;
        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&stcMpllCfg);
        CLK_MpllCmd(Enable);
    }
    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (ADC_CLK == ADC_CLK_UPLLR)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_upll_cfg_t stcUpllCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcUpllCfg);

    /* Use XTAL as UPLL source. */
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Set UPLL out 240MHz. */
    stcUpllCfg.pllmDiv = 2u;
    /* upll = 8M(XTAL) / pllmDiv * plln */
    stcUpllCfg.plln    = 60u;
    stcUpllCfg.PllpDiv = 16u;
    stcUpllCfg.PllqDiv = 16u;
    stcUpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    CLK_SetPeriClkSource(ClkPeriSrcUpllr);
#endif
}

/**
 *******************************************************************************
 ** \brief  ADC initial configuration.
 **
 ******************************************************************************/
static void AdcInitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    MEM_ZERO_STRUCT(stcAdcInit);

    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Enable;
    stcAdcInit.enScanMode   = AdcMode_SAContinuous;
    stcAdcInit.enRschsel    = AdcRschsel_Restart;
    /* 1. Enable ADC1. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &stcAdcInit);
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ******************************************************************************/
static void AdcChannelConfig(void)
{
    stc_adc_ch_cfg_t stcChCfg;

    uint8_t au8Adc1SaSampTime[3] = { 0x60, 0x60, 0x60 };

    MEM_ZERO_STRUCT(stcChCfg);

    stcChCfg.u32Channel  = BOARD_ADC_CH0_CH | BOARD_ADC_CH1_CH | BOARD_ADC_CH2_CH;
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = au8Adc1SaSampTime;

    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

//    ADC_ConfigAvg(M4_ADC1, AdcAvcnt_64);
//    ADC_AddAvgChannel(M4_ADC1, BOARD_ADC_CH0_CH | BOARD_ADC_CH1_CH | BOARD_ADC_CH2_CH);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ******************************************************************************/
static void AdcTriggerConfig(void)
{
    stc_adc_trg_cfg_t stcTrgCfg;

    MEM_ZERO_STRUCT(stcTrgCfg);

    /*
     * If select an event(@ref en_event_src_t) to trigger ADC,
     * AOS must be enabled first.
     */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* ADC1 sequence A will be started by software. */
    ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_A, Disable);
}


/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
static void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode)
{
    uint8_t u8ChIndex;
#if (ADC_CH_REMAP)
    uint8_t u8AdcPin;
#else
    uint8_t u8ChOffset = 0u;
#endif

    if (M4_ADC1 == ADCx)
    {
        u32Channel &= ADC1_PIN_MASK_ALL;
    }
    else
    {
        u32Channel &= ADC2_PIN_MASK_ALL;
#if (!ADC_CH_REMAP)
        u8ChOffset = 4u;
#endif
    }

    u8ChIndex = 0u;
    while (0u != u32Channel)
    {
        if (u32Channel & 0x1ul)
        {
#if (ADC_CH_REMAP)
            u8AdcPin = ADC_GetChannelPinNum(ADCx, u8ChIndex);
            AdcSetPinMode(u8AdcPin, enMode);
#else
            AdcSetPinMode((u8ChIndex + u8ChOffset), enMode);
#endif
        }

        u32Channel >>= 1u;
        u8ChIndex++;
    }
}

/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode)
{
    en_port_t enPort = PortA;
    en_pin_t enPin   = Pin00;
    bool bFlag       = true;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = enMode;
    stcPortInit.enPullUp  = Disable;

    switch (u8AdcPin)
    {
    case ADC1_IN0:
        enPort = PortA;
        enPin  = Pin00;
        break;

    case ADC1_IN1:
        enPort = PortA;
        enPin  = Pin01;
        break;

    case ADC1_IN2:
        enPort = PortA;
        enPin  = Pin02;
        break;

    case ADC1_IN3:
        enPort = PortA;
        enPin  = Pin03;
        break;

    case ADC12_IN4:
        enPort = PortA;
        enPin  = Pin04;
        break;

    case ADC12_IN5:
        enPort = PortA;
        enPin  = Pin05;
        break;

    case ADC12_IN6:
        enPort = PortA;
        enPin  = Pin06;
        break;

    case ADC12_IN7:
        enPort = PortA;
        enPin  = Pin07;
        break;

    case ADC12_IN8:
        enPort = PortB;
        enPin  = Pin00;
        break;

    case ADC12_IN9:
        enPort = PortB;
        enPin  = Pin01;
        break;

    case ADC12_IN10:
        enPort = PortC;
        enPin  = Pin00;
        break;

    case ADC12_IN11:
        enPort = PortC;
        enPin  = Pin01;
        break;

    case ADC1_IN12:
        enPort = PortC;
        enPin  = Pin02;
        break;

    case ADC1_IN13:
        enPort = PortC;
        enPin  = Pin03;
        break;

    case ADC1_IN14:
        enPort = PortC;
        enPin  = Pin04;
        break;

    case ADC1_IN15:
        enPort = PortC;
        enPin  = Pin05;
        break;

    default:
        bFlag = false;
        break;
    }

    if (true == bFlag)
    {
        PORT_Init(enPort, enPin, &stcPortInit);
    }
}


// DMA2 CH0 ~ CH2
void adc_dma_config(void)
{
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    stcDmaCfg.u16BlockSize   = ADC_CH_COUNT;
    stcDmaCfg.u16TransferCnt = 0u;
    stcDmaCfg.u32SrcAddr     = (uint32_t)(&M4_ADC1->DR10);
    stcDmaCfg.u32DesAddr     = (uint32_t)(&g_adc_value[0]);
    stcDmaCfg.u16DesRptSize  = ADC_CH_COUNT;
    stcDmaCfg.u16SrcRptSize  = ADC_CH_COUNT;
    stcDmaCfg.u32DmaLlp      = 0u;
    stcDmaCfg.stcSrcNseqCfg.u16Cnt    = 0u;
    stcDmaCfg.stcSrcNseqCfg.u32Offset = 0u;
    stcDmaCfg.stcDesNseqCfg.u16Cnt    = 0u;
    stcDmaCfg.stcDesNseqCfg.u32Offset = 0u;
    stcDmaCfg.stcDmaChCfg.enSrcInc    = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc    = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enSrcRptEn  = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn  = Enable;
    stcDmaCfg.stcDmaChCfg.enSrcNseqEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesNseqEn = Disable;
    stcDmaCfg.stcDmaChCfg.enTrnWidth  = Dma16Bit;
    stcDmaCfg.stcDmaChCfg.enLlpEn     = Disable;
    stcDmaCfg.stcDmaChCfg.enIntEn     = Disable;

    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);
    DMA_InitChannel(M4_DMA2, DmaCh3, &stcDmaCfg);
    DMA_Cmd(M4_DMA2, Enable);
    DMA_ChannelCmd(M4_DMA2, DmaCh3, Enable);
    DMA_ClearIrqFlag(M4_DMA2, DmaCh3, TrnCpltIrq);
    /* AOS must be enabled to use DMA */
    /* AOS enabled at first. */
    /* If you have enabled AOS before, then the following statement is not needed. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    DMA_SetTriggerSrc(M4_DMA2, DmaCh3, EVT_ADC1_EOCA);
}

