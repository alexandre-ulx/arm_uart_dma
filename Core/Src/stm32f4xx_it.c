#include "main.h"
#include "stm32f4xx_it.h"

extern uint32_t irq_dma_counter;
extern uint32_t irq_uart_counter;

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

void DMA1_Stream5_IRQHandler(void)
{
  /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
        irq_dma_counter++;
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
        irq_dma_counter++;
    }
}

void USART2_IRQHandler(void)
{
  if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
        irq_uart_counter++;
    }
}
