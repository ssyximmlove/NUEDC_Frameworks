//
// Created by Nymphaea on 25-7-29.
//

#ifndef VISION_UART_H
#define VISION_UART_H

#include "main.h"
#include <stdint.h>
#include "usart.h" // 引入usart.h以使用huart2句柄
#include <string.h>
#include <stdio.h>

/**
 * @brief 初始化USART2的DMA接收功能（使用IDLE中断和环形缓冲区）
 */
void Vision_UART_Init(void);

/**
 * @brief 从USART2的环形缓冲区中读取数据
 * @param buffer 用于存放读取数据的缓冲区
 * @param len    要读取的最大长度
 * @return 实际从环形缓冲区中读取到的字节数
 */
uint16_t Vision_UART_Read(uint8_t* buffer, uint16_t len);

/**
 * @brief 检查USART2的环形缓冲区中当前有多少字节的可用数据
 * @return 可用数据的字节数
 */
uint16_t Vision_UART_Available(void);

#endif //VISION_UART_H
