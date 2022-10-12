/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
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
/** \file hc32f46x_i2c.c
 **
 ** A detailed description is available at
 ** @link I2cGroup Inter-Integrated Circuit(I2C) description @endlink
 **
 **   - 2018-10-16  1.0  Wangmin  First version for Device Driver Library of I2C.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_i2c.h"
#include "hc32f46x_utility.h"

#if (DDL_I2C_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup I2cGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define I2C_BAUDRATE_MAX                (400000ul)

/*! Parameter validity check for unit. */
#define IS_VALID_UNIT(x)                                                       \
(   ((x) == M4_I2C1)                               ||                          \
    ((x) == M4_I2C2)                               ||                          \
    ((x) == M4_I2C3))

/*! Parameter check for I2C baudrate value !*/
#define IS_VALID_SPEED(speed)           ((speed) <= (I2C_BAUDRATE_MAX))

/*! Parameter check for I2C baudrate calculate prccess !*/
#define IS_VALID_FDIV(fdiv)             ((fdiv) <= 128.0f)
#define IS_VALID_BAUDWIDTH(result)      ((result) == true)

/*! Parameter check for Digital filter config !*/
#define IS_VALID_DIGITAL_FILTER(x)                                             \
(   ((x) == Filter1BaseCycle)                      ||                          \
    ((x) == Filter2BaseCycle)                      ||                          \
    ((x) == Filter3BaseCycle)                      ||                          \
    ((x) == Filter4BaseCycle))

/*! Parameter check for address mode !*/
#define IS_VALID_ADRMODE(x)                                                    \
(   ((x) == Adr7bit)                               ||                          \
    ((x) == Adr10bit))

/*! Parameter check for I2C transfer direction !*/
#define IS_VALID_TRANS_DIR(x)                                                  \
(   ((x) == I2CDirReceive)                         ||                          \
    ((x) == I2CDirTrans))

/*! Parameter check for Time out control switch !*/
#define IS_VALID_TIMOUT_SWITCH(x)                                              \
(   ((x) == TimeoutFunOff)                         ||                          \
    ((x) == LowTimerOutOn)                         ||                          \
    ((x) == HighTimeOutOn)                         ||                          \
    ((x) == BothTimeOutOn))

/*! Parameter check for I2C 7 bit address range !*/
#define IS_VALID_7BIT_ADR(x)           ((x) <= 0x7F)

/*! Parameter check for I2C 10 bit address range !*/
#define IS_VALID_10BIT_ADR(x)          ((x) <= 0x3FF)

/*! Parameter check for readable I2C status bit !*/
#define IS_VALID_RD_STATUS_BIT(x)                                              \
(   ((x) == I2C_SR_STARTF)                         ||                          \
    ((x) == I2C_SR_SLADDR0F)                       ||                          \
    ((x) == I2C_SR_SLADDR1F)                       ||                          \
    ((x) == I2C_SR_TENDF)                          ||                          \
    ((x) == I2C_SR_STOPF)                          ||                          \
    ((x) == I2C_SR_RFULLF)                         ||                          \
    ((x) == I2C_SR_TEMPTYF)                        ||                          \
    ((x) == I2C_SR_ARLOF)                          ||                          \
    ((x) == I2C_SR_ACKRF)                          ||                          \
    ((x) == I2C_SR_NACKF)                          ||                          \
    ((x) == I2C_SR_TMOUTF)                         ||                          \
    ((x) == I2C_SR_MSL)                            ||                          \
    ((x) == I2C_SR_BUSY)                           ||                          \
    ((x) == I2C_SR_TRA)                            ||                          \
    ((x) == I2C_SR_GENCALLF)                       ||                          \
    ((x) == I2C_SR_SMBDEFAULTF)                    ||                          \
    ((x) == I2C_SR_SMBHOSTF)                       ||                          \
    ((x) == I2C_SR_SMBALRTF))

#define IS_VALID_ACK_CONFIG(x)                                                 \
(   ((x) == I2c_ACK)                                ||                         \
    ((x) == I2c_NACK))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8FreqDiv[8] = {1,2,4,8,16,32,64,128};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  static function for baudrate calculating
 ** \param  [in] fDiv      Divisor value input in float type
 ** \retval uint8_t        Divisor value output
 ******************************************************************************/
static uint8_t GetFreqReg(float fDiv)
{
    uint8_t u8Reg = 0u;

    for(uint8_t i=7u; i>0u; i--)
    {
        if(fDiv >= (float)u8FreqDiv[i-1u])
        {
            u8Reg = i;
            break;
        }
    }

    return u8Reg;
}

/**
 *******************************************************************************
 ** \brief Try to wait a status of specified flags
 ** \param [in] pstcI2Cx           Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32Flag            specifies the flag to check,
 **                                This parameter can be one of the following values:
 **                                I2C_SR_STARTF
 **                                I2C_SR_SLADDR0F
 **                                I2C_SR_SLADDR1F
 **                                I2C_SR_TENDF
 **                                I2C_SR_STOPF
 **                                I2C_SR_RFULLF
 **                                I2C_SR_TEMPTYF
 **                                I2C_SR_ARLOF
 **                                I2C_SR_ACKRF: ACK status
 **                                I2C_SR_NACKF: NACK Flag
 **                                I2C_SR_TMOUTF
 **                                I2C_SR_MSL
 **                                I2C_SR_BUSY
 **                                I2C_SR_TRA
 **                                I2C_SR_GENCALLF
 **                                I2C_SR_SMBDEFAULTF
 **                                I2C_SR_SMBHOSTF
 **                                I2C_SR_SMBALRTF
 ** \param [in] enStatus           Expected status, This parameter can be one of
 **                                the following values:
 **              Set
 **              Reset
 ** \param [in] u32Timeout         Maximum count of trying to get a status of a
 **                                flag in status register
 ** \retval Ok                     Successfully gotten the expected status of the specified flags
 ** \retval ErrorTimeout           Failed to get expected status of specified flags.
 ******************************************************************************/
en_result_t I2C_WaitStatus(const M4_I2C_TypeDef *pstcI2Cx, uint32_t u32Flag, en_flag_status_t enStatus, uint32_t u32Timeout)
{
    en_result_t enRet = ErrorTimeout;
    uint32_t u32RegStatusBit;

    for(;;)
    {
        u32RegStatusBit = (pstcI2Cx->SR & u32Flag);
        if(((enStatus == Set) && (u32Flag == u32RegStatusBit))
           || ((enStatus == Reset) && (0UL == u32RegStatusBit)))
        {
            enRet = Ok;
        }

        if((Ok == enRet) || (0UL == u32Timeout))
        {
            break;
        }
        else
        {
            u32Timeout--;
        }
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2C generate start condition
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          new state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_GenerateStart(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.START = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C generate restart condition
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_GenerateReStart(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.RESTART = enNewState;

}

/**
 *******************************************************************************
 ** \brief I2C generate stop condition
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_GenerateStop(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.STOP = enNewState;
}

/**
 *******************************************************************************
 ** \brief Set the baudrate for I2C peripheral.
 ** \param [in] pstcI2Cx           Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32Baudrate        The value of baudrate.
 ** \param [in] u32SclTime         The SCL Rise and Falling timer(Number of period of pclk3)
 ** \param [in] u32Pclk3           Frequency of pclk3
 ** \retval None
 ******************************************************************************/
void I2C_BaudrateConfig(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32Baudrate, uint32_t u32SclTime, uint32_t u32Pclk3)
{
    float fDivIndex = 0.0f;
    uint8_t u8DivIndex;
    uint32_t width = 0ul;
    uint32_t dnfsum = 0ul, divsum = 0ul;
    uint32_t tmp = 0ul;

    /* Check parameters */
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_SPEED(u32Baudrate));

    /* Judge digitial filter status*/
    if(1u == pstcI2Cx->FLTR_f.DNFEN)
    {
        dnfsum = pstcI2Cx->FLTR_f.DNF+1ul;
    }
    else
    {
        dnfsum = 0ul;
    }
    divsum = 2ul;  //default

    if (0ul != u32Baudrate)
    {
        tmp = u32Pclk3/u32Baudrate - u32SclTime;
    }

    /* Calculate the pclk3 div */
    fDivIndex = (float)tmp / ((32.0f + (float)dnfsum + (float)divsum) * 2.0f);

    DDL_ASSERT(IS_VALID_FDIV(fDivIndex));

    u8DivIndex = GetFreqReg(fDivIndex);

    /* Judge if clock divider on*/
    if(0u == u8DivIndex)
    {
        divsum = 3ul;
    }
    else
    {
        divsum = 2ul;
    }
    width =  tmp / u8FreqDiv[u8DivIndex];
    DDL_ASSERT(IS_VALID_BAUDWIDTH((width/2ul) >= (dnfsum + divsum)));

    /* Write register */
    pstcI2Cx->CCR_f.FREQ = u8DivIndex;
    pstcI2Cx->CCR_f.SLOWW = width / 2ul - dnfsum - divsum;
    pstcI2Cx->CCR_f.SHIGHW = width - width / 2ul - dnfsum - divsum;
}

/**
 *******************************************************************************
 ** \brief De-initialize I2C unit
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \retval Ok                      Process finished.
 ******************************************************************************/
en_result_t I2C_DeInit(M4_I2C_TypeDef* pstcI2Cx)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    /* Reset peripheral register and internal status*/
    pstcI2Cx->CR1_f.PE = 0u;
    pstcI2Cx->CR1_f.SWRST = 1u;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Initialize I2C peripheral according to the structure
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] pstcI2C_InitStruct  Pointer to I2C configuration structure
 ** \retval Ok                      Process finished.
 ** \retval ErrorInvalidParameter   Parameter error.
 ******************************************************************************/
en_result_t I2C_Init(M4_I2C_TypeDef* pstcI2Cx, const stc_i2c_init_t* pstcI2C_InitStruct)
{
    en_result_t enRes = Ok;
    if((NULL == pstcI2C_InitStruct) || (NULL == pstcI2Cx))
    {
        enRes = ErrorInvalidParameter;
    }
    else
    {
        DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
        DDL_ASSERT(IS_VALID_SPEED(pstcI2C_InitStruct->u32Baudrate));

        /* Register and internal status reset */
        pstcI2Cx->CR1_f.PE = 0u;
        pstcI2Cx->CR1_f.SWRST = 1u;

        pstcI2Cx->CR1_f.PE = 1u;

        I2C_BaudrateConfig(pstcI2Cx,
                           pstcI2C_InitStruct->u32Baudrate,
                           pstcI2C_InitStruct->u32SclTime,
                           pstcI2C_InitStruct->u32Pclk3);

        pstcI2Cx->CR1_f.ENGC = 0u;
        pstcI2Cx->CR1_f.SWRST = 0u;
        pstcI2Cx->CR1_f.PE = 0u;
    }
    return enRes;
}

/**
 *******************************************************************************
 ** \brief I2C slave address0 config
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \param [in] enAdrMode           Address mode,can be Adr7bit or Adr10bit
 ** \param [in] u32Adr              The slave address
 ** \retval None
 ******************************************************************************/
void I2C_SlaveAdr0Config(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState, en_address_bit_t enAdrMode, uint32_t u32Adr)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT(IS_VALID_ADRMODE(enAdrMode));

    pstcI2Cx->SLR0_f.SLADDR0EN = enNewState;
    pstcI2Cx->SLR0_f.ADDRMOD0 = enAdrMode;
    if(Adr7bit == enAdrMode)
    {
        DDL_ASSERT(IS_VALID_7BIT_ADR(u32Adr));
        pstcI2Cx->SLR0_f.SLADDR0 = u32Adr << 1ul;
    }
    else
    {
        DDL_ASSERT(IS_VALID_10BIT_ADR(u32Adr));
        pstcI2Cx->SLR0_f.SLADDR0 = u32Adr;
    }
}

/**
 *******************************************************************************
 ** \brief I2C slave address1 config
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \param [in] enAdrMode           Address mode,can be Adr7bit or Adr10bit
 ** \param [in] u32Adr              The slave address
 ** \retval None
 ******************************************************************************/
void I2C_SlaveAdr1Config(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState, en_address_bit_t enAdrMode, uint32_t u32Adr)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT(IS_VALID_ADRMODE(enAdrMode));

    pstcI2Cx->SLR1_f.SLADDR1EN = enNewState;
    pstcI2Cx->SLR1_f.ADDRMOD1 = enAdrMode;
    if(Adr7bit == enAdrMode)
    {
        DDL_ASSERT(IS_VALID_7BIT_ADR(u32Adr));
        pstcI2Cx->SLR1_f.SLADDR1 = u32Adr << 1ul;
    }
    else
    {
        DDL_ASSERT(IS_VALID_10BIT_ADR(u32Adr));
        pstcI2Cx->SLR1_f.SLADDR1 = u32Adr;
    }
}

/**
 *******************************************************************************
 ** \brief I2C function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_Cmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.PE = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C fast ACK function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the fast ACK function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_FastAckCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if(Enable == enNewState)
    {
        pstcI2Cx->CR3_f.FACKEN = 0ul;
    }
    else
    {
        pstcI2Cx->CR3_f.FACKEN = 1ul;
    }
}

/**
 *******************************************************************************
 ** \brief I2C bus wait function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the fast ACK function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_BusWaitCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    uint32_t u32CR4_Reg = ((uint32_t)&pstcI2Cx->CR3) + 4ul;
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if(Enable == enNewState)
    {
        *(__IO uint32_t *)u32CR4_Reg |= (1ul << 10ul);
    }
    else
    {
        *(__IO uint32_t *)u32CR4_Reg &= ~(1ul << 10ul);
    }
}

/**
 *******************************************************************************
 ** \brief I2C SMBUS function configuration
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] pstcI2C_SmbusInitStruct
 **                                 Pointer to I2C SMBUS configuration structure
 ** \retval Ok                      Process finished.
 ** \retval ErrorInvalidParameter   Parameter error.
 ******************************************************************************/
en_result_t I2C_SmbusConfig(M4_I2C_TypeDef* pstcI2Cx, const stc_i2c_smbus_init_t* pstcI2C_SmbusInitStruct)
{
    en_result_t enRet = Ok;
    if(NULL != pstcI2C_SmbusInitStruct)
    {
        DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcI2C_SmbusInitStruct->enHostAdrMatchFunc));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcI2C_SmbusInitStruct->enDefaultAdrMatchFunc));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcI2C_SmbusInitStruct->enAlarmAdrMatchFunc));

        pstcI2Cx->CR1_f.SMBHOSTEN = pstcI2C_SmbusInitStruct->enHostAdrMatchFunc;
        pstcI2Cx->CR1_f.SMBDEFAULTEN = pstcI2C_SmbusInitStruct->enDefaultAdrMatchFunc;
        pstcI2Cx->CR1_f.SMBALRTEN = pstcI2C_SmbusInitStruct->enAlarmAdrMatchFunc;
    }
    else
    {
        enRet = ErrorInvalidParameter;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2C SMBUS function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_SmBusCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.SMBUS = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C digital filter function configuration
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enDigiFilterMode    Chose the digital filter mode, This parameter
 **                                 can be one of the following values:
 **                                 Filter1BaseCycle
 **                                 Filter2BaseCycle
 **                                 Filter3BaseCycle
 **                                 Filter4BaseCycle
 ** \retval None
 ******************************************************************************/
void I2C_DigitalFilterConfig(M4_I2C_TypeDef* pstcI2Cx, en_i2c_digital_filter_mode_t enDigiFilterMode)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_DIGITAL_FILTER(enDigiFilterMode));

    pstcI2Cx->FLTR_f.DNF = enDigiFilterMode;
}

/**
 *******************************************************************************
 ** \brief I2C digital filter function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_DigitalFilterCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->FLTR_f.DNFEN = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C analog filter function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_AnalogFilterCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->FLTR_f.ANFEN = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C general call function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_GeneralCallCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.ENGC = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C status bit get
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32StatusBit        specifies the flag to check,
 **                                 This parameter can be one of the following values:
 **                                 I2C_SR_STARTF
 **                                 I2C_SR_SLADDR0F
 **                                 I2C_SR_SLADDR1F
 **                                 I2C_SR_TENDF
 **                                 I2C_SR_STOPF
 **                                 I2C_SR_RFULLF
 **                                 I2C_SR_TEMPTYF
 **                                 I2C_SR_ARLOF
 **                                 I2C_SR_ACKRF: ACK status
 **                                 I2C_SR_NACKF: NACK Flag
 **                                 I2C_SR_TMOUTF
 **                                 I2C_SR_MSL
 **                                 I2C_SR_BUSY
 **                                 I2C_SR_TRA
 **                                 I2C_SR_GENCALLF
 **                                 I2C_SR_SMBDEFAULTF
 **                                 I2C_SR_SMBHOSTF
 **                                 I2C_SR_SMBALRTF
 ** \retval en_flag_status_t        The status of the I2C status flag, may be Set or Reset.
 ******************************************************************************/
en_flag_status_t I2C_GetStatus(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32StatusBit)
{
    en_flag_status_t enRet = Reset;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_RD_STATUS_BIT(u32StatusBit));

    if(0ul != (pstcI2Cx->SR & u32StatusBit))
    {
        enRet = Set;
    }
    else
    {
        enRet = Reset;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief Clear I2C status flag
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32StatusBit        specifies the flag to clear,
 **                                 This parameter can be any combination of the following values:
 **                                 I2C_CLR_STARTFCLR
 **                                 I2C_CLR_SLADDR0FCLR
 **                                 I2C_CLR_SLADDR1FCLR
 **                                 I2C_CLR_TENDFCLR
 **                                 I2C_CLR_STOPFCLR
 **                                 I2C_CLR_RFULLFCLR
 **                                 I2C_CLR_TEMPTYFCLR
 **                                 I2C_CLR_ARLOFCLR
 **                                 I2C_CLR_NACKFCLR
 **                                 I2C_CLR_TMOUTFCLR
 **                                 I2C_CLR_GENCALLFCLR
 **                                 I2C_CLR_SMBDEFAULTFCLR
 **                                 I2C_CLR_SMBHOSTFCLR
 **                                 I2C_CLR_SMBALRTFCLR
 ** \retval None
 ******************************************************************************/
void I2C_ClearStatus(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32StatusBit)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    pstcI2Cx->CLR |= (u32StatusBit & I2C_CLR_MASK);
}

/**
 *******************************************************************************
 ** \brief I2C software reset function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_SoftwareResetCmd(M4_I2C_TypeDef* pstcI2Cx, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    pstcI2Cx->CR1_f.SWRST = enNewState;
}

/**
 *******************************************************************************
 ** \brief I2C interrupt function command
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32IntEn            Specifies the I2C interrupts sources to be configuration
 **                                 This parameter can be any combination of the following values:
 **                                 I2C_CR2_STARTIE
 **                                 I2C_CR2_SLADDR0EN
 **                                 I2C_CR2_SLADDR1EN
 **                                 I2C_CR2_TENDIE
 **                                 I2C_CR2_STOPIE
 **                                 I2C_CR2_RFULLIE
 **                                 I2C_CR2_TEMPTYIE
 **                                 I2C_CR2_ARLOIE
 **                                 I2C_CR2_NACKIE
 **                                 I2C_CR2_TMOURIE
 **                                 I2C_CR2_GENCALLIE
 **                                 I2C_CR2_SMBDEFAULTIE
 **                                 I2C_CR2_SMBHOSTIE
 **                                 I2C_CR2_SMBALRTIE
 ** \param [in] enNewState          New state of the I2Cx function, can be
 **                                 Disable or Enable the function
 ** \retval None
 ******************************************************************************/
void I2C_IntCmd(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32IntEn, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if(Enable == enNewState)
    {
        pstcI2Cx->CR2 |= u32IntEn;
    }
    else
    {
        pstcI2Cx->CR2 &= ~u32IntEn;
    }
}

/**
 *******************************************************************************
 ** \brief I2C write data register
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u8Data              The data to be send
 ** \retval None
 ******************************************************************************/
void I2C_WriteData(M4_I2C_TypeDef* pstcI2Cx, uint8_t u8Data)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    pstcI2Cx->DTR = u8Data;
}

/**
 *******************************************************************************
 ** \brief I2C read data register
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \retval uint8_t                 The value of the received data
 ******************************************************************************/
uint8_t I2C_ReadData(M4_I2C_TypeDef* pstcI2Cx)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    return pstcI2Cx->DRR;
}

/**
 *******************************************************************************
 ** \brief I2C ACK status configuration
 ** \param [in] pstcI2Cx                Pointer to the I2C peripheral register, can
 **                                     be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32AckConfig            I2C ACK configurate.
 **             I2c_ACK:                Send ACK after date received.
 **             I2c_NACK:               Send NACK after date received.
 ** \retval None
 ******************************************************************************/
void I2C_AckConfig(M4_I2C_TypeDef* pstcI2Cx, en_i2c_ack_config_t u32AckConfig)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_ACK_CONFIG(u32AckConfig));

    pstcI2Cx->CR1_f.ACK = u32AckConfig;
}

/**
 *******************************************************************************
 ** \brief I2C clock timer out function config
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] pstcTimoutInit      Pointer to I2C timeout function structure
 ** \retval Ok                      Process finished.
 ** \retval ErrorInvalidParameter   Parameter error.
 ******************************************************************************/
en_result_t I2C_ClkTimeOutConfig(M4_I2C_TypeDef* pstcI2Cx, const stc_clock_timeout_init_t* pstcTimoutInit)
{
    en_result_t enRet = Ok;
    if(NULL != pstcTimoutInit)
    {
        DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
        DDL_ASSERT(IS_VALID_TIMOUT_SWITCH(pstcTimoutInit->enClkTimeOutSwitch));

        pstcI2Cx->SLTR_f.TOUTHIGH = pstcTimoutInit->u16TimeOutHigh;
        pstcI2Cx->SLTR_f.TOUTLOW = pstcTimoutInit->u16TimeOutLow;

        pstcI2Cx->CR3 &= ~0x00000007ul;
        pstcI2Cx->CR3 |= pstcTimoutInit->enClkTimeOutSwitch;
    }
    else
    {
        enRet = ErrorInvalidParameter;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx start
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok                      Start success
 ** \retval ErrorTimeout            Start time out
 ******************************************************************************/
en_result_t I2C_Start(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32Timeout)
{
    en_result_t enRet;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_BUSY, Reset, u32Timeout);

    if(Ok == enRet)
    {
        /* generate start signal */
        I2C_GenerateStart(pstcI2Cx, Enable);
        /* Judge if start success*/
        enRet = I2C_WaitStatus(pstcI2Cx, (I2C_SR_BUSY | I2C_SR_STARTF), Set, u32Timeout);
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx restart
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok                      Restart successfully
 ** \retval ErrorTimeout            Restart time out
 ******************************************************************************/
en_result_t I2C_Restart(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32Timeout)
{
    en_result_t enRet;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    /* Clear start status flag */
    I2C_ClearStatus(pstcI2Cx, I2C_CLR_STARTFCLR);
    /* Send restart condition */
    I2C_GenerateReStart(pstcI2Cx, Enable);
    /* Judge if start success*/
    enRet = I2C_WaitStatus(pstcI2Cx, (I2C_SR_BUSY | I2C_SR_STARTF), Set, u32Timeout);

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx send address
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u8Addr              The address to be sent
 ** \param [in] enDir               Can be I2CDirTrans or I2CDirReceive
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Send successfully
 ** \retval Error:                  Send suscessfully and receive NACK
 ** \retval ErrorTimeout:           Send address time out
 ******************************************************************************/
en_result_t I2C_TransAddr(M4_I2C_TypeDef* pstcI2Cx, uint8_t u8Addr, en_trans_direction_t enDir, uint32_t u32Timeout)
{
    en_result_t enRet;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_7BIT_ADR(u8Addr));
    DDL_ASSERT(IS_VALID_TRANS_DIR(enDir));

    enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TEMPTYF, Set, u32Timeout);

    if(Ok == enRet)
    {
        /* Send I2C address */
        I2C_WriteData(pstcI2Cx, (u8Addr << 1u) | (uint8_t)enDir);

        if(I2CDirTrans == enDir)
        {
            /* If in master transfer process, Need wait transfer end */
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TENDF, Set, u32Timeout);
        }
        else
        {
            /* If in master receive process, Need wait TRA flag */
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TRA, Reset, u32Timeout);
        }

        if(enRet == Ok)
        {
            /* If receive NACK */
            if(I2C_GetStatus(pstcI2Cx, I2C_SR_ACKRF) == Set)
            {
                enRet = Error;
            }
        }
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx send address 10 bit
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u16Addr             The address to be sent
 ** \param [in] enDir               Can be I2CDirTrans or I2CDirReceive
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Send successfully
 ** \retval Error:                  Send suscessfully and receive NACK
 ** \retval ErrorTimeout:           Send address time out
 ******************************************************************************/
en_result_t I2C_Trans10BitAddr(M4_I2C_TypeDef* pstcI2Cx, uint16_t u16Addr, en_trans_direction_t enDir, uint32_t u32Timeout)
{
    en_result_t enRet;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));
    DDL_ASSERT(IS_VALID_10BIT_ADR(u16Addr));
    DDL_ASSERT(IS_VALID_TRANS_DIR(enDir));

    enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TEMPTYF, Set, u32Timeout);
    if(Ok == enRet)
    {
        /* Write 11110 + SLA(bit9:8) + W#(1bit) */
        I2C_WriteData(pstcI2Cx, (uint8_t)((u16Addr>>7u) & 0x06u) | 0xF0u | (uint8_t)I2CDirTrans);
        enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TENDF, Set, u32Timeout);

        if(Ok == enRet)
        {
            /* If receive ACK */
            if(I2C_GetStatus(pstcI2Cx, I2C_SR_ACKRF) == Reset)
            {
                /* Write SLA(bit7:0)*/
                I2C_WriteData(pstcI2Cx, (uint8_t)(u16Addr & 0xFFu));
                enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TENDF, Set, u32Timeout);

                if(Ok == enRet)
                {
                    if(I2C_GetStatus(pstcI2Cx, I2C_SR_ACKRF) != Reset)
                    {
                        enRet = Error;
                    }
                }
            }
            else
            {
                enRet = Error;
            }
        }
    }

    if((I2CDirReceive == enDir) && (Ok == enRet))
    {
        /* Restart */
        I2C_ClearStatus(pstcI2Cx, I2C_CLR_STARTFCLR);
        I2C_GenerateReStart(pstcI2Cx, Enable);
        enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_STARTF, Set, u32Timeout);

        if(Ok == enRet)
        {
            /* Write 11110 + SLA(bit9:8) + R(1bit) */
            I2C_WriteData(pstcI2Cx, (uint8_t)((u16Addr>>7u) & 0x06u) | 0xF0u | (uint8_t)I2CDirReceive);
            /* If in master receive process, Need wait TRA flag */
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TRA, Reset, u32Timeout);

            if(Ok == enRet)
            {
                /* If receive NACK */
                if(I2C_GetStatus(pstcI2Cx, I2C_SR_ACKRF) != Reset)
                {
                    enRet = Error;
                }
            }
        }
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx send data
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] au8TxData           The data array to be sent
 ** \param [in] u32Size             Number of data in array pau8TxData
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Send successfully
 ** \retval ErrorTimeout:           Send data time out
 ** \retval ErrorInvalidParameter:  au8TxData is NULL
 ******************************************************************************/
en_result_t I2C_TransData(M4_I2C_TypeDef* pstcI2Cx, uint8_t const au8TxData[], uint32_t u32Size, uint32_t u32Timeout)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    en_result_t enRet = Ok;
    __IO uint32_t u32Cnt = 0ul;

    if(au8TxData != NULL)
    {
        while((u32Cnt != u32Size) && (enRet == Ok))
        {
            /* Wait tx buffer empty */
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TEMPTYF, Set, u32Timeout);

            if(enRet == Ok)
            {
                /* Send one byte data */
                I2C_WriteData(pstcI2Cx, au8TxData[u32Cnt]);

                /* Wait transfer end */
                enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_TENDF, Set, u32Timeout);

                /* If receive NACK in slave tx mode */
                if(I2C_GetStatus(pstcI2Cx, I2C_SR_NACKF) == Set)
                {
                    I2C_ClearStatus(pstcI2Cx, I2C_CLR_NACKFCLR);
                    /* Exit data transfer */
                    break;
                }

                u32Cnt++;
            }
        }
    }
    else
    {
        enRet = ErrorInvalidParameter;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx receive data
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [out] au8RxData          Array to hold the received data
 ** \param [in] u32Size             Number of data to be received
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Receive successfully
 ** \retval ErrorTimeout:           Receive data time out
 ** \retval ErrorInvalidParameter:  au8RxData is NULL
 ******************************************************************************/
en_result_t I2C_ReceiveData(M4_I2C_TypeDef* pstcI2Cx, uint8_t au8RxData[], uint32_t u32Size, uint32_t u32Timeout)
{
    en_result_t enRet = Ok;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    if(au8RxData != NULL)
    {
        uint32_t u32FastAckDis = (pstcI2Cx->CR3_f.FACKEN);
        for(uint32_t i=0ul; i<u32Size; i++)
        {
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_RFULLF, Set, u32Timeout);

            if(0ul == u32FastAckDis)
            {
                if((u32Size >= 2ul) && (i == (u32Size - 2ul)))
                {
                    I2C_AckConfig(pstcI2Cx, I2c_NACK);
                }
            }
            else
            {
                if(i != (u32Size - 1ul))
                {
                    I2C_AckConfig(pstcI2Cx, I2c_ACK);
                }
                else
                {
                    I2C_AckConfig(pstcI2Cx, I2c_NACK);
                }
            }

            if(enRet == Ok)
            {
                 /* read data from register */
                au8RxData[i] = I2C_ReadData(pstcI2Cx);
            }
            else
            {
                break;
            }
        }
        I2C_AckConfig(pstcI2Cx, I2c_ACK);
    }
    else
    {
        enRet = ErrorInvalidParameter;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx master receive data and stop
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [out] au8RxData          Array to hold the received data
 ** \param [in] u32Size             Number of data to be received
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Receive successfully
 ** \retval ErrorTimeout:           Receive data time out
 ** \retval ErrorInvalidParameter:  au8RxData is NULL
 ******************************************************************************/
en_result_t I2C_MasterDataReceiveAndStop(M4_I2C_TypeDef* pstcI2Cx, uint8_t au8RxData[], uint32_t u32Size, uint32_t u32Timeout)
{
    en_result_t enRet = Ok;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    if(au8RxData != NULL)
    {
        uint32_t u32FastAckDis = (pstcI2Cx->CR3_f.FACKEN);
        for(uint32_t i=0ul; i<u32Size; i++)
        {
            enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_RFULLF, Set, u32Timeout);

            if(0ul == u32FastAckDis)
            {
                if((u32Size >= 2ul) && (i == (u32Size - 2ul)))
                {
                    I2C_AckConfig(pstcI2Cx, I2c_NACK);
                }
            }
            else
            {
                if(i != (u32Size - 1ul))
                {
                    I2C_AckConfig(pstcI2Cx, I2c_ACK);
                }
                else
                {
                    I2C_AckConfig(pstcI2Cx, I2c_NACK);
                }
            }

            if(enRet == Ok)
            {
                /* Stop before read last data */
                if(i == (u32Size - 1ul))
                {
                    I2C_ClearStatus(pstcI2Cx, I2C_CLR_STOPFCLR);
                    I2C_GenerateStop(pstcI2Cx, Enable);
                }

                 /* read data from register */
                au8RxData[i] = I2C_ReadData(pstcI2Cx);

                /* Wait stop flag after DRR read */
                if(i == (u32Size - 1ul))
                {
                    enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_STOPF, Set, u32Timeout);
                }
            }
            else
            {
                break;
            }
        }
        I2C_AckConfig(pstcI2Cx, I2c_ACK);
    }
    else
    {
        enRet = ErrorInvalidParameter;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief I2Cx stop
 ** \param [in] pstcI2Cx            Pointer to the I2C peripheral register, can
 **                                 be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param [in] u32Timeout          Maximum count of trying to get a status of a
 **                                 flag in status register
 ** \retval Ok:                     Receive successfully
 ** \retval ErrorTimeout:           Receive data time out
 ******************************************************************************/
en_result_t I2C_Stop(M4_I2C_TypeDef* pstcI2Cx, uint32_t u32Timeout)
{
    en_result_t enRet;

    DDL_ASSERT(IS_VALID_UNIT(pstcI2Cx));

    /* Clear stop flag */
    while((Set == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF)) && (u32Timeout > 0ul))
    {
        I2C_ClearStatus(pstcI2Cx, I2C_CLR_STOPFCLR);
        u32Timeout--;
    }
    I2C_GenerateStop(pstcI2Cx, Enable);
    /* Wait stop flag */
    enRet = I2C_WaitStatus(pstcI2Cx, I2C_SR_STOPF, Set, u32Timeout);

    return enRet;
}

//@} // I2cGroup

#endif /* DDL_I2C_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
