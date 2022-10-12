#include "bsp_rmu.h"


uint8_t rmu_get_reset_cause(void)
{
    uint8_t res;

    stc_rmu_rstcause_t stcRmuRstCause;

    MEM_ZERO_STRUCT(stcRmuRstCause);

    RMU_GetResetCause(&stcRmuRstCause);

    if(Set == stcRmuRstCause.enSoftware) {
        res = RST_CAU_SOFTWARE;
    } else if(Set == stcRmuRstCause.enWdt) {
        res = RST_CAU_WATCHDOG;
    } else if(Set == stcRmuRstCause.enRstPin) {
        res = RST_CAU_EXTERNAL;
    }

    return res;
}

uint8_t rmu_clear_reset_cause(void)
{
    RMU_ClrResetFlag();

    return 0;
}


