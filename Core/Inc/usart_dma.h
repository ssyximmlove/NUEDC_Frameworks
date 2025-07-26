//
// Created by Nymphaea on 25-7-26.
//

#ifndef USART_DMA_H
#define USART_DMA_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define RX_BUFFER_SIZE 512
#define RING_BUFFER_SIZE 1024

/* Exported types ------------------------------------------------------------*/
// 环形缓冲区结构体已在 main.h 中定义

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void USART_DMA_Init(void);
void RingBuffer_Write(RingBuffer_t *rb, uint8_t *data, uint16_t len);
uint16_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint16_t len);
uint16_t RingBuffer_Available(RingBuffer_t *rb);
void RingBuffer_Clear(RingBuffer_t *rb);
void USART2_DMA_RX_Process(void);
/* IDLE空闲中断 */
void USART2_IDLE_Callback(void);
/* 获取接收到的数据长度 */
uint16_t USART_DMA_GetDataLength(void);

/* 读取指定长度的数据 */
uint16_t USART_DMA_ReadData(uint8_t *buffer, uint16_t max_len);

/* 检查是否有新数据 */
uint8_t USART_DMA_HasNewData(void);

#endif //USART_DMA_H
