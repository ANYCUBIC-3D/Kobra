#include "bsp_irq.h"

#include "HardwareSerial.h"


uint8_t g_uart2_rx_buf[128];
uint8_t g_uart2_rx_index;


extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;


void BSP_USART1_RIIrqHander(void)
{
    uint8_t c = USART_RecData(USART1_CH);
    Serial1._rx_complete_callback(c);
//  RingBuf_Write(&Usart1RingBuf,(uint8_t)tmp);
}

void BSP_USART1_EIIrqHander(void)
{
  if(Set == USART_GetStatus(USART1_CH, UsartFrameErr))
    USART_ClearStatus(USART1_CH, UsartFrameErr);
  if(Set == USART_GetStatus(USART1_CH, UsartParityErr))
    USART_ClearStatus(USART1_CH, UsartParityErr);
  if (Set == USART_GetStatus(USART1_CH, UsartOverrunErr))
    USART_ClearStatus(USART1_CH, UsartOverrunErr);
}


void BSP_USART1_TIrqHander(void)
{

}

void BSP_USART1_TCIIrqHander(void)
{

}

void BSP_USART2_RIIrqHander(void)
{
    uint8_t c = USART_RecData(USART2_CH);
    Serial2._rx_complete_callback(c);

//  RingBuf_Write(&Usart2RingBuf,(uint8_t)tmp);
}

void BSP_USART2_EIIrqHander(void)
{
  if(Set == USART_GetStatus(USART2_CH, UsartFrameErr))
    USART_ClearStatus(USART2_CH, UsartFrameErr);
  if(Set == USART_GetStatus(USART2_CH, UsartParityErr))
    USART_ClearStatus(USART2_CH, UsartParityErr);
  if (Set == USART_GetStatus(USART2_CH, UsartOverrunErr))
    USART_ClearStatus(USART2_CH, UsartOverrunErr);
}

void BSP_USART2_TIrqHander(void)
{

}

void BSP_USART2_TCIIrqHander(void)
{

}

void BSP_USART3_RIIrqHander(void)
{
  uint8_t c = USART_RecData(USART3_CH);
  Serial3._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void BSP_USART3_EIIrqHander(void)
{
  if(Set == USART_GetStatus(USART3_CH, UsartFrameErr))
    USART_ClearStatus(USART3_CH, UsartFrameErr);
  if(Set == USART_GetStatus(USART3_CH, UsartParityErr))
    USART_ClearStatus(USART3_CH, UsartParityErr);
  if (Set == USART_GetStatus(USART3_CH, UsartOverrunErr))
    USART_ClearStatus(USART3_CH, UsartOverrunErr);
}

void BSP_USART3_TIrqHander(void)
{

}

void BSP_USART3_TCIIrqHander(void)
{

}

void BSP_USART4_RIIrqHander(void)
{
  uint8_t c = USART_RecData(USART4_CH);
  Serial4._rx_complete_callback(c);
//  RingBuf_Write(&Usart3RingBuf,(uint8_t)tmp);
}

void BSP_USART4_EIIrqHander(void)
{
  if(Set == USART_GetStatus(USART4_CH, UsartFrameErr))
    USART_ClearStatus(USART4_CH, UsartFrameErr);
  if(Set == USART_GetStatus(USART4_CH, UsartParityErr))
    USART_ClearStatus(USART4_CH, UsartParityErr);
  if (Set == USART_GetStatus(USART4_CH, UsartOverrunErr))
    USART_ClearStatus(USART4_CH, UsartOverrunErr);
}

void BSP_USART4_TIrqHander(void)
{

}

void BSP_USART4_TCIIrqHander(void)
{

}


