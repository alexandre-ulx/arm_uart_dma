#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"

void SystemClock_Config(void);

/**
 * @brief Static instance of the buffer structure for synchronous UART.
 *
 * This variable is used to store the data received via UART.
 */
struct SyncUART_buffer_t sync_buffer = {0};

struct CO_Static_MemoryMaintenance_t Sync_uart_rxTx = {
	.pfuncRx = (void *)NULL,
	.pfuncTx = (void *)NULL,
    .tx = {
        .maxNodes = 0,
    }
};

/**
 * @brief Global variables for IRQ counters.
 * 
 * These variables keep track of the number of IRQs triggered for DMA and UART.
 * They are used to count the number of times the corresponding IRQ handlers are called.
 */
uint32_t irq_dma_counter  = 0,
         irq_uart_counter = 0;

/**
 * @brief Process the received data and transmit it through USART1.
 *
 * This function takes a pointer to the data and the length of the data as input parameters.
 * It transmits the data byte by byte through USART1.
 * Before transmitting the data, it sends the string "Received: [" through USART1.
 * After transmitting all the data, it sends the string "]\r\n" through USART1.
 *
 * @param data Pointer to the data to be processed and transmitted.
 * @param len Length of the data in bytes.
 */
void usart_process_data(const void* data, int len) {
    UART_TRANSMIT_STRING(USART1, "Received: [");

    // envia os dados recebidos para a USART1
    for (int i = 0; i < len; i++) {
        LL_USART_TransmitData8(USART1, ((uint8_t*)data)[i]);
        while (!LL_USART_IsActiveFlag_TXE(USART1)) {}
    }
    
    UART_TRANSMIT_STRING(USART1, "]\r\n");
}

uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len)
{
	static const uint32_t table[16] = {
		0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU,
		0x76dc4190U, 0x6b6b51f4U, 0x4db26158U, 0x5005713cU,
		0xedb88320U, 0xf00f9344U, 0xd6d6a3e8U, 0xcb61b38cU,
		0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U, 0xbdbdf21cU,
	};

	crc = ~crc;

	for (size_t i = 0; i < len; i++) {
		uint8_t byte = data[i];

		crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
		crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
	}

	return (~crc);
}

uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
	return crc32_ieee_update(0x0, data, len);
}

void usart_rx_check(void) {

    /* Calculate current position in buffer and check for new data available */
    uint16_t rec_bytes_size = ARRAY_LEN(sync_buffer.temp_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);

    if (rec_bytes_size >= sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t)) {
        // Extract the length from the buffer
        uint16_t length = sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t)] |
                          (sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t) + 1] << 8);

        // Calculate the expected total packet size
        uint16_t expected_size = sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t) + length + sizeof(uint32_t);

        // Check if we have received the entire packet
        if (rec_bytes_size >= expected_size) {
            struct SyncUART_Pack_t *packet = (struct SyncUART_Pack_t *)sync_buffer.temp_buffer;

            // Verify the packet identity
            if (packet->header.identity == SYNC_UART_IDENTIFY_TX) {
                if (packet->length <= MAX_DATA_SIZE) {
                    // Verify the CRC
                    uint32_t received_crc = packet->crc32 = sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t) + length] |
                                            (sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t) + length + 1] << 8) |
                                            (sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t) + length + 2] << 16) |
                                            (sync_buffer.temp_buffer[sizeof(struct SyncUART_Mux_t) + sizeof(uint16_t) + length + 3] << 24);
                    uint32_t calculated_crc = crc32_ieee(sync_buffer.temp_buffer, expected_size - sizeof(packet->crc32));

                    if (received_crc == calculated_crc) {
                        
                        // copy data validate to RAM.
                        memcpy(&Sync_uart_rxTx.tx, packet->data, packet->length);

                        // sequency frame error check
                        if ((packet->header.sequency_number - Sync_uart_rxTx.last_sequency) != 1) {
                          Sync_uart_rxTx.last_sequency = Sync_uart_rxTx.tx.sequency_error = Sync_uart_rxTx.sequency_error++;
                        }

                        // signal to process received
                        if (Sync_uart_rxTx.pfuncRx != NULL) {
                          Sync_uart_rxTx.pfuncRx((void *)&Sync_uart_rxTx.rx);
                        }

                    } else {
                        LOG_WRN("CRC mismatch: clearing buffer\r\n");
                    }
                } else {
                    LOG_WRN("Packet length exceeds maximum data size: clearing buffer\r\n");
                }
            } else {
                // Invalid identity, clear buffer
                LOG_WRN("Invalid packet identity: clearing buffer\r\n");
            }
        }

        // reset dma buffer
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
        memset(sync_buffer.temp_buffer, 0, expected_size);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)sync_buffer.temp_buffer);
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, ARRAY_LEN(sync_buffer.temp_buffer));
        LL_DMA_ClearFlag_HT5(DMA1);
        LL_DMA_ClearFlag_TC5(DMA1);
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
    }
}

int main(void)
{

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  uint32_t counter = 0;

  while (1)
  {
    // UART_TRANSMIT_STRING(USART1, "Counter: ");
    // UART_TRANSMIT_STRING(USART1, "\r\n");
    counter = counter + 1;
    LL_mDelay(1000);
  }
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {

  }
}
