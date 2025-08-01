//
// Created by Nymphaea on 25-7-29.
//

#ifndef STEPPER_UART_H
#define STEPPER_UART_H
#include <stdint.h>
#include "usart.h" // 引入usart.h以使用huart句柄
#include <string.h>
#include <stdio.h>


typedef enum {
    STATE_SEND_PITCH,
    STATE_WAIT_PITCH,
    STATE_SEND_YAW,
    STATE_WAIT_YAW
} GimbalCommState;

extern volatile GimbalCommState g_gimbal_state;


/**
 * @brief 初始化步进电机串口（USART3）的DMA接收功能
 */
void Stepper_UART_Init(void);

/**
 * @brief 从步进电机串口的环形缓冲区中读取数据
 * @param buffer 用于存放读取数据的缓冲区
 * @param len    要读取的最大长度
 * @return 实际读取到的字节数
 */
uint16_t Stepper_UART_Read(uint8_t* buffer, uint16_t len);

/**
 * @brief 检查步进电机串口的环形缓冲区中当前有多少字节的可用数据
 * @return 可用数据的字节数
 */
uint16_t Stepper_UART_Available(void);


extern float g_yaw_angle;
extern float g_pitch_angle;

void Stepper_UART_HandleData(void);


#endif //STEPPER_UART_H
