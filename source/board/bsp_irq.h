#ifndef __BSP_IRQ_H__
#define __BSP_IRQ_H__


#include "hc32_ddl.h"

#include "bsp_init.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define IRQ_INDEX_USART1_INT_RI         Int000_IRQn
#define IRQ_INDEX_USART1_INT_EI         Int001_IRQn
#define IRQ_INDEX_USART1_INT_TI         Int002_IRQn
#define IRQ_INDEX_USART1_INT_TCI        Int003_IRQn

#define IRQ_INDEX_USART2_INT_RI         Int004_IRQn
#define IRQ_INDEX_USART2_INT_EI         Int005_IRQn
#define IRQ_INDEX_USART2_INT_TI         Int006_IRQn
#define IRQ_INDEX_USART2_INT_TCI        Int007_IRQn

#define IRQ_INDEX_USART3_INT_RI         Int008_IRQn
#define IRQ_INDEX_USART3_INT_EI         Int009_IRQn
#define IRQ_INDEX_USART3_INT_TI         Int010_IRQn
#define IRQ_INDEX_USART3_INT_TCI        Int011_IRQn

#define IRQ_INDEX_USART4_INT_RI         Int012_IRQn
#define IRQ_INDEX_USART4_INT_EI         Int013_IRQn
#define IRQ_INDEX_USART4_INT_TI         Int014_IRQn
#define IRQ_INDEX_USART4_INT_TCI        Int015_IRQn

#define IRQ_INDEX_INT_DMA2_TC0          Int016_IRQn
#define IRQ_INDEX_INT_DMA2_TC1          Int017_IRQn
#define IRQ_INDEX_INT_DMA2_TC2          Int018_IRQn

#define IRQ_INDEX_INT_TMR01_GCMA        Int019_IRQn
#define IRQ_INDEX_INT_TMR01_GCMB        Int020_IRQn

#define IRQ_INDEX_INT_TMR02_GCMA        Int021_IRQn
#define IRQ_INDEX_INT_TMR02_GCMB        Int022_IRQn

#define IRQ_INDEX_INT_TMR41_GCMB        Int023_IRQn
#define IRQ_INDEX_INT_TMR42_GCMB        Int024_IRQn


extern uint8_t g_uart2_rx_buf[128];
extern uint8_t g_uart2_rx_index;


void BSP_USART1_RIIrqHander(void);
void BSP_USART1_EIIrqHander(void);
void BSP_USART1_TIrqHander(void);
void BSP_USART1_TCIIrqHander(void);

void BSP_USART2_RIIrqHander(void);
void BSP_USART2_EIIrqHander(void);
void BSP_USART2_TIrqHander(void);
void BSP_USART2_TCIIrqHander(void);

void BSP_USART3_RIIrqHander(void);
void BSP_USART3_EIIrqHander(void);
void BSP_USART3_TIrqHander(void);
void BSP_USART3_TCIIrqHander(void);

void BSP_USART4_RIIrqHander(void);
void BSP_USART4_EIIrqHander(void);
void BSP_USART4_TIrqHander(void);
void BSP_USART4_TCIIrqHander(void);
    

#ifdef __cplusplus
}
#endif


#endif

