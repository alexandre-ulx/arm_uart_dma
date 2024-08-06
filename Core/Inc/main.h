#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "uart_sync.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define BUFFER_SIZE 25

extern struct SyncUART_buffer_t sync_buffer;

void Error_Handler(void);

void usart_rx_check(void);

#define UART_TRANSMIT_STRING(uart, string) \
  do { \
    for (int i = 0; string[i] != '\0'; i++) { \
      LL_USART_TransmitData8(uart, string[i]); \
      while (!LL_USART_IsActiveFlag_TXE(uart)) {} \
    } \
  } while(0)


#define LOG_WRN(...) \
  do { \
    UART_TRANSMIT_STRING(USART1, "\033[33mWRN: \033[0m"); \
    UART_TRANSMIT_STRING(USART1, __VA_ARGS__); \
  } while(0)

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
