#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__


#include "hc32_ddl.h"


#ifdef __cplusplus
extern "C"
{
#endif


#define BOARD_ADC_CH0_PORT       (PortC)
#define BOARD_ADC_CH0_PIN        (Pin00)
#define BOARD_ADC_CH0_CH         (ADC1_CH10)

#define BOARD_ADC_CH1_PORT       (PortC)
#define BOARD_ADC_CH1_PIN        (Pin01)
#define BOARD_ADC_CH1_CH         (ADC1_CH11)

#define BOARD_ADC_CH2_PORT       (PortC)
#define BOARD_ADC_CH2_PIN        (Pin02)
#define BOARD_ADC_CH2_CH         (ADC1_CH12)

/* Timer definition for this example. */
#define TMR_UNIT                 (M4_TMR02)


extern uint16_t  AdcCH0SampleBuf[256];
extern uint16_t  AdcCH1SampleBuf[256];
extern uint16_t  AdcCH2SampleBuf[256];

extern uint32_t  AdcCH0Value;
extern uint32_t  AdcCH1Value;
extern uint32_t  AdcCH2Value;

extern uint16_t g_adc_value[3];
extern uint8_t g_adc_idx;

void adc_init(void);

static void adc_pin_init(void);

static void AdcClockConfig(void);

static void AdcInitConfig(void);

static void AdcChannelConfig(void);

static void AdcTriggerConfig(void);


static void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode);

static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode);

void adc_dma_config(void);


void BSP_DMA2CH0_TcIrqHander(void);

void BSP_DMA2CH1_TcIrqHander(void);

void BSP_DMA2CH2_TcIrqHander(void);


void AdcConfig(void);



#ifdef __cplusplus
}
#endif


#endif

