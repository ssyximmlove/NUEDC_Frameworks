//
// Created by Nymphaea on 25-7-28.
//

#ifndef ISR_H
#define ISR_H

#include "key.h"
#include "gimbal.h"
#include "tim.h"
#include "usart.h"

// --- 外部引用 vision_uart (USART2) 的变量 ---
extern volatile uint16_t g_ring_buffer_head_u2;
extern volatile uint16_t g_ring_buffer_tail_u2;
extern uint8_t g_dma_rx_buffer_u2[];
extern uint8_t g_ring_buffer_u2[];
#define DMA_RX_BUFFER_SIZE_U2   256
#define RING_BUFFER_SIZE_U2     1024

// --- 外部引用 stepper_uart (USART3) 的变量 ---
extern volatile uint16_t g_ring_buffer_head_u3;
extern volatile uint16_t g_ring_buffer_tail_u3;
extern uint8_t g_dma_rx_buffer_u3[];
extern uint8_t g_ring_buffer_u3[];
#define DMA_RX_BUFFER_SIZE_U3   128
#define RING_BUFFER_SIZE_U3     512

void HAL_Timer6_Init(void);

#endif //ISR_H
