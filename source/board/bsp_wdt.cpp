#include "bsp_wdt.h"


// pclk = system_clock/div4 = 50M
// max cycle = 65536
// max feed interval = 65536 / (50000000/8192) = 10.7s
void bsp_wdt_init(void)
{
    stc_wdt_init_t stcWdtInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcWdtInit);

    stcWdtInit.enClkDiv = WdtPclk3Div8192;
    stcWdtInit.enCountCycle = WdtCountCycle65536;
    stcWdtInit.enRefreshRange = WdtRefresh100Pct;
    stcWdtInit.enSleepModeCountEn = Disable;
    stcWdtInit.enRequestType = WdtTriggerResetRequest;
    WDT_Init(&stcWdtInit);
}

void bsp_wdt_refresh(void)
{
    en_result_t enRet = Error;
    enRet = WDT_RefreshCounter();

    if(enRet != Ok) {
        printf("Failed at function: %s, line: %d\n", __FUNCTION__, __LINE__);
    }
}

