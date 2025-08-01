//
// Created by Nymphaea on 25-7-29.
//

#include "stepper_uart.h"

#include "gimbal.h"

// --- c ---
#define DMA_RX_BUFFER_SIZE_U3   128  // DMA硬件直接操作的缓冲区
#define RING_BUFFER_SIZE_U3     512  // 软件环形缓冲区

// 定义缓冲区
uint8_t g_dma_rx_buffer_u3[DMA_RX_BUFFER_SIZE_U3];
uint8_t g_ring_buffer_u3[RING_BUFFER_SIZE_U3];

// 定义环形缓冲区的读写指针
volatile uint16_t g_ring_buffer_head_u3 = 0;
volatile uint16_t g_ring_buffer_tail_u3 = 0;

volatile GimbalCommState g_gimbal_state = STATE_SEND_PITCH;

/**
 * @brief 初始化并启动USART3的DMA+IDLE中断接收
 */
void Stepper_UART_Init(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dma_rx_buffer_u3, DMA_RX_BUFFER_SIZE_U3) != HAL_OK)
    {
        Error_Handler();
    }
    printf("Stepper UART (USART3) DMA+IDLE receiver initialized.\n");
}

/**
 * @brief 从环形缓冲区中读取数据
 */
uint16_t Stepper_UART_Read(uint8_t* buffer, uint16_t len)
{
    uint16_t bytes_to_read = 0;
    __disable_irq(); // 进入临界区
    uint16_t available = (g_ring_buffer_head_u3 + RING_BUFFER_SIZE_U3 - g_ring_buffer_tail_u3) % RING_BUFFER_SIZE_U3;
    bytes_to_read = (len > available) ? available : len;
    for (uint16_t i = 0; i < bytes_to_read; i++)
    {
        buffer[i] = g_ring_buffer_u3[g_ring_buffer_tail_u3];
        g_ring_buffer_tail_u3 = (g_ring_buffer_tail_u3 + 1) % RING_BUFFER_SIZE_U3;
    }
    __enable_irq(); // 退出临界区
    return bytes_to_read;
}

/**
 * @brief 检查环形缓冲区中当前有多少字节的可用数据
 */
uint16_t Stepper_UART_Available(void)
{
    return (g_ring_buffer_head_u3 + RING_BUFFER_SIZE_U3 - g_ring_buffer_tail_u3) % RING_BUFFER_SIZE_U3;
}

#define STEPPER_ANGLE_FRAME_LEN 8

float g_yaw_angle = 0.0f;
float g_pitch_angle = 0.0f;

void Stepper_UART_Angle_Current_ProcessFrame(const uint8_t* frame)
{
    if (frame[1] == 0x36 && frame[7] == 0x6B) {
        int32_t pos = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6];
        if (frame[2] == 0x01) pos = -pos;
        float angle = (float)pos * 360.0f / 65536.0f;
        if (frame[0] == YAW_MOTOR_ADDR) {
            g_yaw_angle = angle;
        } else if (frame[0] == PITCH_MOTOR_ADDR) {
            g_pitch_angle = angle;
        }

    } else {
        // 错误帧，打印原始数据
        printf("Stepper UART Frame Error: ");
        for (int i = 0; i < 8; i++) printf("%02X ", frame[i]);
        printf("\n");
    }
}

void Stepper_UART_HandleData(void)
{
    while (Stepper_UART_Available() >= STEPPER_ANGLE_FRAME_LEN) {
        uint8_t frame[STEPPER_ANGLE_FRAME_LEN];
        Stepper_UART_Read(frame, STEPPER_ANGLE_FRAME_LEN);
        Stepper_UART_Angle_Current_ProcessFrame(frame);
        if (frame[0] == PITCH_MOTOR_ADDR && g_gimbal_state == STATE_WAIT_PITCH) {
            g_gimbal_state = STATE_SEND_YAW;
        } else if (frame[0] == YAW_MOTOR_ADDR && g_gimbal_state == STATE_WAIT_YAW) {
            g_gimbal_state = STATE_SEND_PITCH;
        }
    }
}
