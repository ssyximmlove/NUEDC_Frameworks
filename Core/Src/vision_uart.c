//
// Created by Nymphaea on 25-7-29.
//

#include "../Inc/vision_uart.h"


// --- 缓冲区定义 for USART2 (Vision) ---
#define DMA_RX_BUFFER_SIZE_U2   256  // DMA硬件直接操作的缓冲区
#define RING_BUFFER_SIZE_U2     1024 // 软件环形缓冲区

// 定义缓冲区
uint8_t g_dma_rx_buffer_u2[DMA_RX_BUFFER_SIZE_U2];
uint8_t g_ring_buffer_u2[RING_BUFFER_SIZE_U2];

// 定义环形缓冲区的读写指针
volatile uint16_t g_ring_buffer_head_u2 = 0;
volatile uint16_t g_ring_buffer_tail_u2 = 0;

/**
 * @brief 初始化并启动USART2的DMA+IDLE中断接收
 */
void Vision_UART_Init(void)
{
    // 使用HAL库最现代、最推荐的函数来启动DMA接收
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma_rx_buffer_u2, DMA_RX_BUFFER_SIZE_U2) != HAL_OK)
    {
        Error_Handler();
    }
    printf("USART2 DMA with IDLE Interrupt Initialized (Optimized).\n");
}

/**
 * @brief 从环形缓冲区中读取数据 (增加临界区保护)
 */
uint16_t Vision_UART_Read(uint8_t* buffer, uint16_t len)
{
    uint16_t bytes_to_read = 0;

    // 进入临界区：关中断，防止在读取时被中断修改tail指针
    __disable_irq();

    uint16_t available = (g_ring_buffer_head_u2 + RING_BUFFER_SIZE_U2 - g_ring_buffer_tail_u2) % RING_BUFFER_SIZE_U2;
    bytes_to_read = (len > available) ? available : len;

    for (uint16_t i = 0; i < bytes_to_read; i++)
    {
        buffer[i] = g_ring_buffer_u2[g_ring_buffer_tail_u2];
        g_ring_buffer_tail_u2 = (g_ring_buffer_tail_u2 + 1) % RING_BUFFER_SIZE_U2;
    }

    // 退出临界区：开中断
    __enable_irq();

    return bytes_to_read;
}

/**
 * @brief 检查环形缓冲区中当前有多少字节的可用数据
 */
uint16_t Vision_UART_Available(void)
{
    return (g_ring_buffer_head_u2 + RING_BUFFER_SIZE_U2 - g_ring_buffer_tail_u2) % RING_BUFFER_SIZE_U2;
}
