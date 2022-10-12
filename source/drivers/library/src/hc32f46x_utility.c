/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file hc32f46x_utility.c
 **
 ** A detailed description is available at
 ** @link DdlUtilityGroup Ddl Utility description @endlink
 **
 **   - 2018-11-02  1.0  Zhangxl First version for Device Driver Library Utility.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_utility.h"

#if (DDL_UTILITY_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup DdlUtilityGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint32_t m_u32TickStep = 0UL;
static __IO uint32_t m_u32TickCount = 0UL;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
#ifdef UART_DEBUG_PRINTF
/**
 *******************************************************************************
 ** \brief Data printf via Uart Ch.3
 **
 ** \param [in]  u8Data                 Data to be sent
 **
 ******************************************************************************/
void DebugOutput(uint8_t u8Data)
{
    while(Reset == M4_USART2->SR_f.TXE);
    while(Reset == M4_USART2->SR_f.TC);
    M4_USART2->DR = u8Data;
}

/**
 *******************************************************************************
 ** \brief Re-target putchar function
 **
 ******************************************************************************/
#if defined ( __GNUC__ ) && !defined (__CC_ARM)
int _write(int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        DebugOutput((uint8_t)pBuffer[i]);
    }
    return size;
}
#else
extern "C" int32_t fputc(int32_t ch, FILE *f)
{
    DebugOutput((uint8_t)ch);

    return (ch);
}
#endif

/**
 *******************************************************************************
 ** \brief Set synchronous clock mode baudrate
 **
 ** \param [in] u32Baudrate             Baudrate
 **
 ** \retval Ok                          Configure successfully.
 ** \retval ErrorInvalidParameter       USARTx is invalid
 **
 ******************************************************************************/
static en_result_t SetUartBaudrate(uint32_t u32Baudrate)
{
    en_result_t enRet = Ok;
    uint32_t B;
    uint32_t C;
    uint32_t OVER8;
    float32_t DIV = 0.0f;
    uint64_t u64Tmp = 0u;
    uint32_t DIV_Integer = 0u;
    uint32_t DIV_Fraction = 0xFFFFFFFFul;

    uint32_t u32PClk1 = 0u;
    uint32_t u32UartClk = 0u;

    u32PClk1 = SystemCoreClock / (1ul << (M4_SYSREG->CMU_SCFGR_f.PCLK1S));
    u32UartClk = u32PClk1 / (1ul << (2ul * (M4_USART3->PR_f.PSC)));

    B = u32Baudrate;
    C = u32UartClk;

    if (0ul == C)
    {
        enRet = ErrorInvalidParameter;
    }
    else
    {
        OVER8 = M4_USART3->CR1_f.OVER8;

        /* FBME = 0 Calculation formula */
        /* B = C / (8 * (2 - OVER8) * (DIV_Integer + 1)) */
        /* DIV_Integer = (C / (B * 8 * (2 - OVER8))) - 1 */
        DIV = ((float)C / ((float)B * 8.0f * (2.0f - (float)OVER8))) - 1.0f;
        DIV_Integer = (uint32_t)(DIV);

        if ((DIV < 0.0f) || (DIV_Integer > 0xFFul))
        {
            enRet = ErrorInvalidParameter;
        }
        else
        {
            if ((DIV - (float32_t)DIV_Integer) > 0.00001f)
            {
                /* FBME = 1 Calculation formula */
                /* B = C * (128 + DIV_Fraction) / (8 * (2 - OVER8) * (DIV_Integer + 1) * 256) */
                /* DIV_Fraction = ((8 * (2 - OVER8) * (DIV_Integer + 1) * 256 * B) / C) - 128 */
                /* E = (C * (128 + DIV_Fraction) / (8 * (2 - OVER8) * (DIV_Integer + 1) * 256 * B)) - 1 */
                /* DIV_Fraction = (((2 - OVER8) * (DIV_Integer + 1) * 2048 * B) / C) - 128 */
                u64Tmp = (2u - (uint64_t)OVER8) * ((uint64_t)DIV_Integer + 1u) * (uint64_t)B;
                DIV_Fraction = (uint32_t)(2048ul * u64Tmp/C - 128ul);
            }
            else
            {
            }

            M4_USART3->CR1_f.FBME = (0xFFFFFFFFul == DIV_Fraction) ? 0ul : 1ul;
            M4_USART3->BRR_f.DIV_FRACTION = DIV_Fraction;
            M4_USART3->BRR_f.DIV_INTEGER = DIV_Integer;
            enRet = Ok;
        }
    }
    return enRet;
}

/**
 *******************************************************************************
 ** \brief Debug printf initialization function
 **
 ** \retval Ok                          Process successfully done
 **
 ******************************************************************************/
en_result_t Ddl_UartInit(void)
{
    en_result_t enRet = Ok;

    /* unlock */
    M4_PORT->PWPR = 0xA501u;
    /* usart3_tx gpio  PE5 */
    M4_PORT->PFSRE5_f.FSEL  = 32u;
    /* lock */
    M4_PORT->PWPR = 0xA500u;
    /* enable usart3 */
    M4_MSTP->FCG1_f.USART3 = 0ul;
    /* usart3 init */

    M4_USART3->CR1_f.ML = 0ul;    // LSB
    M4_USART3->CR1_f.MS = 0ul;    // UART mode
    M4_USART3->CR1_f.OVER8 = 1ul; // 8bit sampling mode
    M4_USART3->CR1_f.M = 0ul;     // 8 bit data length
    M4_USART3->CR1_f.PCE = 0ul;   // no parity bit

    /* baudrate set */
    if( Ok != SetUartBaudrate(115200ul))
    {
        enRet = Error;
    }
    else
    {
        /* 1 stop bit, single uart mode */
        M4_USART3->CR2 = 0ul;

        /* CTS disable, Smart Card mode disable */
        M4_USART3->CR3 = 0ul;

        M4_USART3->CR1_f.TE = 1ul;    // TX enable
    }

    return enRet;
}
#endif /* UART_DEBUG_PRINTF_ENABLE */

/**
 *******************************************************************************
 ** \brief Delay function, delay 1ms approximately
 **
 ** \param [in]  u32Cnt                 ms
 **
 ** \retval none
 **
 ******************************************************************************/
void Ddl_Delay1ms(uint32_t u32Cnt)
{
    volatile uint32_t i = 0ul;
    uint32_t u32Cyc = 0ul;

    u32Cyc = SystemCoreClock;
    u32Cyc = u32Cyc / 10000ul;
    while (u32Cnt-- > 0ul)
    {
        i = u32Cyc;
        while (i-- > 0ul)
        {
            ;
        }
    }
}

/**
 *******************************************************************************
 ** \brief Delay function, delay 1us approximately
 **
 ** \param [in]  u32Cnt                 us
 **
 ** \retval none
 **
 ******************************************************************************/
void Ddl_Delay1us(uint32_t u32Cnt)
{
    uint32_t u32Cyc = 1ul;
    volatile uint32_t i = 0ul;

    if(SystemCoreClock > 10000000ul)
    {
        u32Cyc = SystemCoreClock / 10000000ul;
        while(u32Cnt-- > 0ul)
        {
            i = u32Cyc;
            while (i-- > 0ul)
            {
                ;
            }
        }
    }
    else
    {
         while(u32Cnt-- > 0ul)
         {
            ;
         }
    }
}

/**
 *******************************************************************************
 ** \brief This function Initializes the interrupt frequency of the SysTick.
 **
 ** \param [in] u32Freq                 SysTick interrupt frequency (1 to 1000).
 **
 ** \retval Ok                          SysTick Initializes succeed
 ** \retval Error                       SysTick Initializes failed
 **
 ******************************************************************************/
__WEAKDEF en_result_t SysTick_Init(uint32_t u32Freq)
{
    en_result_t enRet = Error;

    if ((0UL != u32Freq) && (u32Freq <= 1000UL))
    {
        m_u32TickStep = 1000UL / u32Freq;
        /* Configure the SysTick interrupt */
        if (0UL == SysTick_Config(SystemCoreClock / u32Freq))
        {
            enRet = Ok;
        }
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief This function provides minimum delay (in milliseconds).
 **
 ** \param [in] u32Delay                Delay specifies the delay time.
 **
 ** \retval None
 **
 ******************************************************************************/
__WEAKDEF void SysTick_Delay(uint32_t u32Delay)
{
    const uint32_t tickStart = SysTick_GetTick();
    uint32_t tickEnd;
    uint32_t tickMax;

    if (m_u32TickStep != 0UL)
    {
        tickMax = 0xFFFFFFFFUL;
        /* Add a freq to guarantee minimum wait */
        if ((u32Delay >= tickMax) || ((tickMax - u32Delay) < m_u32TickStep))
        {
            tickEnd = tickMax;
        }
        else
        {
            tickEnd = u32Delay + m_u32TickStep;
        }

        while ((SysTick_GetTick() - tickStart) < (tickEnd - 1))
        {
        }
    }
}

/**
 *******************************************************************************
 ** \brief This function is called to increment a global variable "u32TickCount".
 ** \note  This variable is incremented in SysTick ISR.
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************/
__WEAKDEF void SysTick_IncTick(void)
{
    m_u32TickCount += m_u32TickStep;
}

/**
 *******************************************************************************
 ** \brief Provides a tick value in millisecond.
 **
 ** \param None
 **
 ** \retval Tick value
 **
 ******************************************************************************/
__WEAKDEF uint32_t SysTick_GetTick(void)
{
    return m_u32TickCount;
}

/**
 *******************************************************************************
 ** \brief Suspend SysTick increment.
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************/
__WEAKDEF void SysTick_Suspend(void)
{
    /* Disable SysTick Interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

/**
 *******************************************************************************
 ** \brief Resume SysTick increment.
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************/
__WEAKDEF void SysTick_Resume(void)
{
    /* Enable SysTick Interrupt */
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

/**
 *******************************************************************************
 ** \brief ddl assert error handle function
 **
 ** \param [in]  file                   Point to the current assert the wrong file
 ** \param [in]  line                   Point line assert the wrong file in the current
 **
 ******************************************************************************/
#ifdef __DEBUG
__WEAKDEF void Ddl_AssertHandler(uint8_t *file, int16_t line)
{
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    while (1)
    {
        ;
    }
}
#endif /* __DEBUG */

//@} // DdlUtilityGroup

#endif /* DDL_UTILITY_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
