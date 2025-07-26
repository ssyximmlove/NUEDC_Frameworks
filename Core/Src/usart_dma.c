//
// Created by Nymphaea on 25-7-26.
//

#include "../Inc/usart_dma.h"

/* Private variables ---------------------------------------------------------*/
uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
RingBuffer_t uart_ring_buffer = {0};
uint16_t last_dma_pos = 0;

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief  初始化 USART DMA 接收
 * @param  None
 * @retval None
 */
void USART_DMA_Init(void)
{
    // 清空环形缓冲区
    RingBuffer_Clear(&uart_ring_buffer);

    // 重置 DMA 位置
    last_dma_pos = 0;

    // 启用 USART2 IDLE 中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    // 启动 USART2 DMA 循环接收
    HAL_UART_Receive_DMA(&huart2, rx_dma_buffer, RX_BUFFER_SIZE);
}

/**
 * @brief  向环形缓冲区写入数据
 * @param  rb: 环形缓冲区指针
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval None
 */
void RingBuffer_Write(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        if(rb->count < RING_BUFFER_SIZE)
        {
            rb->buffer[rb->head] = data[i];
            rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
            rb->count++;
        }
        else
        {
            // 缓冲区满，覆盖旧数据
            rb->buffer[rb->head] = data[i];
            rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
            rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        }
    }
}

/**
 * @brief  从环形缓冲区读取数据
 * @param  rb: 环形缓冲区指针
 * @param  data: 数据存储指针
 * @param  len: 要读取的最大长度
 * @retval 实际读取的数据长度
 */
uint16_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
    uint16_t read_count = 0;

    while(rb->count > 0 && read_count < len)
    {
        data[read_count] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        rb->count--;
        read_count++;
    }

    return read_count;
}

/**
 * @brief  获取环形缓冲区可用数据长度
 * @param  rb: 环形缓冲区指针
 * @retval 可用数据长度
 */
uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
    return rb->count;
}

/**
 * @brief  清空环形缓冲区
 * @param  rb: 环形缓冲区指针
 * @retval None
 */
void RingBuffer_Clear(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief  处理 DMA 接收的数据
 * @param  None
 * @retval None
 */
void USART2_DMA_RX_Process(void)
{
    uint16_t current_dma_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint16_t data_len = 0;

    if(current_dma_pos != last_dma_pos)
    {
        if(current_dma_pos > last_dma_pos)
        {
            // 正常情况：没有环绕
            data_len = current_dma_pos - last_dma_pos;
            RingBuffer_Write(&uart_ring_buffer, &rx_dma_buffer[last_dma_pos], data_len);
        }
        else
        {
            // DMA 缓冲区环绕的情况
            data_len = RX_BUFFER_SIZE - last_dma_pos;
            RingBuffer_Write(&uart_ring_buffer, &rx_dma_buffer[last_dma_pos], data_len);

            if(current_dma_pos > 0)
            {
                RingBuffer_Write(&uart_ring_buffer, &rx_dma_buffer[0], current_dma_pos);
            }
        }

        last_dma_pos = current_dma_pos;
    }
}

/**
 * @brief  USART2 IDLE 中断回调函数
 * @param  None
 * @retval None
 */
void USART2_IDLE_Callback(void)
{
    uint32_t temp;

    // 清除 IDLE 标志位
    temp = huart2.Instance->SR;
    temp = huart2.Instance->DR;

    // 处理 DMA 接收的数据
    USART2_DMA_RX_Process();
}

/**
 * @brief  获取环形缓冲区中可用数据长度
 * @retval 可用数据长度
 */
uint16_t USART_DMA_GetDataLength(void)
{
    return RingBuffer_Available(&uart_ring_buffer);
}

/**
 * @brief  从环形缓冲区读取数据
 * @param  buffer: 数据存储指针
 * @param  max_len: 要读取的最大长度
 * @retval 实际读取的数据长度
 */
uint16_t USART_DMA_ReadData(uint8_t *buffer, uint16_t max_len)
{
    return RingBuffer_Read(&uart_ring_buffer, buffer, max_len);
}

/**
 * @brief  判断环形缓冲区是否有新数据
 * @retval 1: 有新数据，0: 无新数据
 */
uint8_t USART_DMA_HasNewData(void)
{
    return (RingBuffer_Available(&uart_ring_buffer) > 0) ? 1 : 0;
}
