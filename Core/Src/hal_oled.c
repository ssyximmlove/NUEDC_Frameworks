//
// @file: hal_oled.c

//

#include "hal_oled.h"

#include <stdbool.h>
#include <stdio.h>

// 配置
extern I2C_HandleTypeDef hi2c1;
#define U8G2_I2C_HANDLE (&hi2c1)
#define OLED_I2C_ADDRESS 0x3C
#define SCREEN_BUFFER_SIZE 1040
static uint8_t u8g2_buf[SCREEN_BUFFER_SIZE];

u8g2_t u8g2;

/**
 * @brief U8g2的底层硬件通信回调函数 (阻塞I2C版本)
 * @note  这是连接U8g2和STM32硬件的桥梁
 */
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer_idx;
    uint8_t *data;

    switch (msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0)
            {
                if (buffer_idx < SCREEN_BUFFER_SIZE) {
                    u8g2_buf[buffer_idx++] = *data;
                }
                data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_INIT:
            /* 在CubeMX中已经初始化，此处无需操作 */
            break;

        case U8X8_MSG_BYTE_SET_DC:
            /* I2C模式下忽略 */
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buffer_idx = 0;
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            // 调用HAL库的普通“阻塞”发送函数
            // 这个函数会一直占用CPU，直到所有数据都发送完毕或者发生错误。
            if (HAL_I2C_Master_Transmit(U8G2_I2C_HANDLE, (OLED_I2C_ADDRESS << 1), u8g2_buf, buffer_idx, HAL_MAX_DELAY) != HAL_OK)
            {
                // 发送失败的错误处理
            }
            break;

        default:
            return 0;
    }
    return 1;
}

/**
 * @brief U8g2的GPIO和延时回调函数
 * @note  这个函数为U8g2提供硬件延时和GPIO控制（主要用于SPI模式或Reset引脚）
 */
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
        // CubeMX已经完成了GPIO和时钟的初始化，所以这里留空
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;

            // HAL库提供的毫秒延时函数
        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
            break;
        case U8X8_MSG_GPIO_CS:
        case U8X8_MSG_GPIO_DC:
            break;
        case U8X8_MSG_GPIO_RESET:
            break;
        default:
            // U8g2期望对未知的消息返回1
            u8x8_SetGPIOResult(u8x8, 1);
            break;
    }
    return 1;
}

HAL_StatusTypeDef HAL_OLED_Init(void)
{
    printf("I2C scanning started...\n");
    bool device_found = false;

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK)
        {
            printf("Device found at address: 0x%02X (7-bit), 0x%02X (8-bit)\n", addr, addr << 1);

            // 检查是否是OLED设备地址
            if (addr == OLED_I2C_ADDRESS) {
                device_found = true;
            }
        }
    }

    printf("I2C scanning completed\n");

    // 如果未找到OLED设备，直接返回错误
    if (!device_found) {
        printf("OLED device not found at address 0x%02X\n", OLED_I2C_ADDRESS);
        return HAL_ERROR;
    }

    printf("OLED device found, initializing u8g2...\n");

    // 初始化u8g2，选择合适的显示驱动
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_i2c, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // 唤醒显示屏
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    printf("OLED initialization completed successfully\n");
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_Clear(void)
{
    u8g2_ClearBuffer(&u8g2);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_DrawString(uint8_t x, uint8_t y, const char* str)
{
    u8g2_DrawStr(&u8g2, x, y, str);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_DrawPixel(uint8_t x, uint8_t y)
{
    u8g2_DrawPixel(&u8g2, x, y);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    u8g2_DrawLine(&u8g2, x1, y1, x2, y2);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_DrawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    u8g2_DrawBox(&u8g2, x, y, w, h);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_DrawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    u8g2_DrawFrame(&u8g2, x, y, w, h);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_SendBuffer(void)
{
    u8g2_SendBuffer(&u8g2);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_OLED_SetFont(const uint8_t *font)
{
    u8g2_SetFont(&u8g2, font);
    return HAL_OK;
}

void HAL_OLED_Test() {
    // Test 1: Basic text display
    HAL_OLED_Clear();
    HAL_OLED_SetFont(u8g2_font_ncenB08_tr);
    HAL_OLED_DrawString(0, 15, "HAL OLED Test");
    HAL_OLED_DrawString(0, 30, "Hello World!");
    HAL_OLED_SendBuffer();
    HAL_Delay(2000);

    // Test 2: Different fonts
    HAL_OLED_Clear();
    HAL_OLED_SetFont(u8g2_font_6x10_tr);
    HAL_OLED_DrawString(0, 10, "Small Font");
    HAL_OLED_SetFont(u8g2_font_ncenB10_tr);
    HAL_OLED_DrawString(0, 25, "Medium Font");
    HAL_OLED_SetFont(u8g2_font_ncenB14_tr);
    HAL_OLED_DrawString(0, 45, "Large Font");
    HAL_OLED_SendBuffer();
    HAL_Delay(2000);

    // Test 3: Graphics shapes
    HAL_OLED_Clear();
    HAL_OLED_DrawFrame(5, 5, 50, 30);
    HAL_OLED_DrawBox(65, 5, 50, 30);
    HAL_OLED_DrawLine(5, 45, 120, 45);
    HAL_OLED_DrawLine(5, 50, 120, 60);
    for(int i = 0; i < 10; i++)
    {
        HAL_OLED_DrawPixel(10 + i*2, 55);
    }
    HAL_OLED_SendBuffer();
    HAL_Delay(2000);

    // Test 4: Animation test
    for(int i = 0; i < 100; i++)
    {
        HAL_OLED_Clear();
        HAL_OLED_SetFont(u8g2_font_ncenB08_tr);
        HAL_OLED_DrawString(0, 15, "Animation Test");
        HAL_OLED_DrawBox(i, 25, 10, 10);
        HAL_OLED_DrawFrame(0, 40, 128, 20);
        HAL_OLED_SendBuffer();
        HAL_Delay(50);
    }

    // Test 5: Text scrolling
    char scroll_text[] = "This is a scrolling text demonstration...";
    for(int pos = 0; pos > -200; pos -= 2)
    {
        HAL_OLED_Clear();
        HAL_OLED_SetFont(u8g2_font_ncenB08_tr);
        HAL_OLED_DrawString(0, 15, "Scroll Test:");
        HAL_OLED_DrawString(pos, 35, scroll_text);
        HAL_OLED_SendBuffer();
        HAL_Delay(30);
    }

    HAL_Delay(1000);
}