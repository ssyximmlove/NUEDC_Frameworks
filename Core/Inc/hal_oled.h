//
// Created by Nymphaea on 25-7-24.
//

#ifndef HAL_OLED_H
#define HAL_OLED_H

#include "main.h"
#include "u8g2.h"

// 对外声明我们的硬件通信回调函数
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

HAL_StatusTypeDef HAL_OLED_Init(void);
HAL_StatusTypeDef HAL_OLED_Clear(void);
HAL_StatusTypeDef HAL_OLED_DrawString(uint8_t x, uint8_t y, const char* str);
HAL_StatusTypeDef HAL_OLED_DrawPixel(uint8_t x, uint8_t y);
HAL_StatusTypeDef HAL_OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
HAL_StatusTypeDef HAL_OLED_DrawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
HAL_StatusTypeDef HAL_OLED_DrawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
HAL_StatusTypeDef HAL_OLED_SendBuffer(void);
HAL_StatusTypeDef HAL_OLED_SetFont(const uint8_t *font);

void HAL_OLED_Test();

#endif //HAL_OLED_H
