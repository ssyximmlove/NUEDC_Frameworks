//
// Created by Nymphaea on 25-7-29.
//

#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>

#include "main.h"
#include <stdint.h>
#include "usart.h" // 引入usart.h以使用huart2句柄
#include <string.h>
#include <stdio.h>

#define TARGET_X 160
#define TARGET_Y 120
#define TARGET_TOLERANCE 5

#define VISION_PACKET_HEADER    0xAA
#define VISION_PACKET_TRAILER_R 0x0D // '\r'
#define VISION_PACKET_TRAILER_N 0x0A // '\n'

#define CMD_TARGET_FOUND        0x01
#define CMD_TARGET_NOT_FOUND    0x02
#define VISION_DATA_TIMEOUT_MS  500

typedef enum {
    PARSER_STATE_WAIT_HEADER,
    PARSER_STATE_WAIT_CMD,
    PARSER_STATE_WAIT_DATA,
    PARSER_STATE_WAIT_TRAILER_R,
    PARSER_STATE_WAIT_TRAILER_N
} ParserState_t;

typedef struct {
    uint16_t x;              // Target X coordinate
    uint16_t y;              // Target Y coordinate
    bool is_found;          // Flag indicating if the target was found in the last valid packet
    uint32_t last_update_time; // Timestamp of the last update
} VisionData_t;



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


/**
 * @brief 处理视觉模块数据
 */
void Vision_ProcessData(void);

/*
 * @brief 获取视觉模块数据
 */
bool Vision_GetTarget(uint16_t *x, uint16_t *y);

#endif //VISION_UART_H
