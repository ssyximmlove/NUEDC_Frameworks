//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/isr.h"

#include "tim.h"
#include "vision_uart.h"

/**
 * @brief 初始化按键定时器
 */
void HAL_Timer6_Init(void) {
    HAL_TIM_Base_Start_IT(&htim6);
    printf("50Hz Key Interrupt Started\n");
}

/**
 * @brief 定时器中断回调函数（f = 50Hz)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_Key_Timer_IRQHandler();



    }
}

/**
 * @brief 全局的UART接收事件回调函数 (这是一个弱函数，我们在这里重新实现它)
 * @param huart 触发回调的UART句柄
 * @param Size  本次DMA接收到的数据长度
 * @note  这个函数会处理所有串口的IDLE中断事件
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        // --- 处理来自 USART2 (Vision) 的数据 ---
        for (uint16_t i = 0; i < Size; i++)
        {
            uint16_t next_head = (g_ring_buffer_head_u2 + 1) % RING_BUFFER_SIZE_U2;
            if (next_head == g_ring_buffer_tail_u2) {
                // 视觉模块的环形缓冲区已满，丢弃新数据
                break;
            }
            g_ring_buffer_u2[g_ring_buffer_head_u2] = g_dma_rx_buffer_u2[i];
            g_ring_buffer_head_u2 = next_head;
        }
        // 重新启动USART2的接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma_rx_buffer_u2, DMA_RX_BUFFER_SIZE_U2);
    }
    else if (huart->Instance == USART3)
    {
        // --- 处理来自 USART3 (Stepper) 的数据 ---
        for (uint16_t i = 0; i < Size; i++)
        {
            uint16_t next_head = (g_ring_buffer_head_u3 + 1) % RING_BUFFER_SIZE_U3;
            if (next_head == g_ring_buffer_tail_u3) {
                // 步进电机模块的环形缓冲区已满，丢弃新数据
                break;
            }
            g_ring_buffer_u3[g_ring_buffer_head_u3] = g_dma_rx_buffer_u3[i];
            g_ring_buffer_head_u3 = next_head;
        }
        // 重新启动USART3的接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dma_rx_buffer_u3, DMA_RX_BUFFER_SIZE_U3);
    }
}