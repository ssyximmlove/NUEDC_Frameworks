//
// Created by Nymphaea on 25-7-29.
//

#include "../Inc/vision_uart.h"
#include "arm_math.h"

static volatile VisionData_t s_vision_data = {0};

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
    __disable_irq();
    uint16_t available = (g_ring_buffer_head_u2 + RING_BUFFER_SIZE_U2 - g_ring_buffer_tail_u2) % RING_BUFFER_SIZE_U2;
    __enable_irq();
    return available;
}

/**
 * @brief Parses a single byte from the UART stream using a state machine.
 * @param byte The byte to parse.
 */
static void ParseByte(uint8_t byte) {
    static ParserState_t parser_state = PARSER_STATE_WAIT_HEADER;
    static uint8_t current_cmd = 0;
    static uint8_t data_buffer[4]; // For coordinates
    static uint8_t data_received_count = 0;

    switch (parser_state) {
        case PARSER_STATE_WAIT_HEADER:
            if (byte == VISION_PACKET_HEADER) {
                parser_state = PARSER_STATE_WAIT_CMD;
            }
            break;

        case PARSER_STATE_WAIT_CMD:
            current_cmd = byte;
            if (current_cmd == CMD_TARGET_FOUND) {
                data_received_count = 0;
                parser_state = PARSER_STATE_WAIT_DATA;
            } else if (current_cmd == CMD_TARGET_NOT_FOUND) {
                parser_state = PARSER_STATE_WAIT_TRAILER_R;
            } else {
                // Invalid command, reset and wait for the next header
                parser_state = PARSER_STATE_WAIT_HEADER;
            }
            break;

        case PARSER_STATE_WAIT_DATA:
            data_buffer[data_received_count++] = byte;
            if (data_received_count >= 4) {
                parser_state = PARSER_STATE_WAIT_TRAILER_R;
            }
            break;

        case PARSER_STATE_WAIT_TRAILER_R:
            if (byte == VISION_PACKET_TRAILER_R) {
                parser_state = PARSER_STATE_WAIT_TRAILER_N;
            } else {
                // Invalid trailer, reset
                parser_state = PARSER_STATE_WAIT_HEADER;
            }
            break;

        case PARSER_STATE_WAIT_TRAILER_N:
            if (byte == VISION_PACKET_TRAILER_N) {
                // Packet is complete and valid, process it
                if (current_cmd == CMD_TARGET_FOUND) {
                    // Assuming little-endian: [x_low, x_high, y_low, y_high]
                    s_vision_data.x = (uint16_t)((data_buffer[1] << 8) | data_buffer[0]);
                    s_vision_data.y = (uint16_t)((data_buffer[3] << 8) | data_buffer[2]);
                    s_vision_data.is_found = true;
                    s_vision_data.last_update_time = HAL_GetTick();
                    // printf("Vision: Target FOUND at (X: %u, Y: %u)\n",s_vision_data.x,s_vision_data.y);
                } else if (current_cmd == CMD_TARGET_NOT_FOUND) {
                    s_vision_data.is_found = false;
                    s_vision_data.last_update_time = HAL_GetTick();
                    // printf("Vision: Target NOT_FOUND\n");
                }
            }
            // Reset for the next packet, regardless of whether the tail was valid or not
            parser_state = PARSER_STATE_WAIT_HEADER;
            break;

        default:
            // Should not happen, but as a safeguard, reset the state
            parser_state = PARSER_STATE_WAIT_HEADER;
            break;
    }
}
void Vision_ProcessData(void) {
    uint8_t byte_buffer[64]; // Process up to 64 bytes at a time
    uint16_t bytes_to_read = Vision_UART_Available();

    if (bytes_to_read > 0) {
        uint16_t bytes_read = Vision_UART_Read(byte_buffer, sizeof(byte_buffer));
        for (uint16_t i = 0; i < bytes_read; i++) {
            ParseByte(byte_buffer[i]);
        }
    }

    // Timeout check: If no valid packet has been received for a while, assume the target is lost.
    if (s_vision_data.is_found && (HAL_GetTick() - s_vision_data.last_update_time > VISION_DATA_TIMEOUT_MS)) {
        s_vision_data.is_found = false;
        printf("Vision target lost due to timeout.\n");
    }
}

bool Vision_GetTarget(uint16_t *x, uint16_t *y) {
    __disable_irq();
    bool found = s_vision_data.is_found;
    if (found) {
        *x = s_vision_data.x;
        *y = s_vision_data.y;
    }
    __enable_irq();
    return found;
}
