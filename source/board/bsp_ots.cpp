#include "bsp_ots.h"
#include "hc32_ddl.h"


#define OTS_CLK_SEL_XTAL            (0u)
#define OTS_CLK_SEL_HRC             (1u)

/* Select XTAL as OTS clock. */
#define OTS_CLK_SEL                 (OTS_CLK_SEL_XTAL)

#define SYS_CLOCK_FREQ_MHZ          (SystemCoreClock / 1000000ul)

#define TIMEOUT_MS                  (10u)


void bsp_ots_init(void)
{

// OTS must enable LRC
    CLK_LrcCmd(Enable);
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS, Enable);

    stc_ots_init_t stcOtsInit;

    stcOtsInit.enAutoOff = OtsAutoOff_Disable;
    stcOtsInit.enClkSel = OtsClkSel_Xtal;
    stcOtsInit.u8ClkFreq = (uint8_t)SYS_CLOCK_FREQ_MHZ;

    OTS_Init(&stcOtsInit);
}

void test_ots(void)
{
    en_result_t enRet;
    float m_f32Temperature = .0f;

//Start OTS, check OTS, get the temperature value.
    enRet = OTS_StartGetTemp(&m_f32Temperature, TIMEOUT_MS);
    if (Ok == enRet) {
        printf("\nChip temp: %.2f.", m_f32Temperature);
    } else {
        printf("\nOTS error, code %d.", enRet);
    }
}


