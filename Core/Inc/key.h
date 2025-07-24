//
// Created by Nymphaea on 25-7-24.
//

#ifndef KEY_H
#define KEY_H

#include "main.h"
#include <stdbool.h>

// 按键引脚定义
#define KEY1_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_3
#define KEY3_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_4
#define KEY4_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_5
#define KEY5_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_6

// 消抖时间定义（ms）
#define KEY_DEBOUNCE_TIME 20

// 按键状态定义
typedef enum {
    KEY_NONE = 0,
    KEY1_PRESSED,
    KEY2_PRESSED,
    KEY3_PRESSED,
    KEY4_PRESSED,
    KEY5_PRESSED
} KeyState_t;

// 按键扫描模式
typedef enum {
    KEY_MODE_SINGLE = 0,    // 单次按键
    KEY_MODE_CONTINUOUS     // 连续按键
} KeyMode_t;

// 按键状态机状态
typedef enum {
    KEY_STATE_IDLE = 0,     // 空闲状态
    KEY_STATE_DEBOUNCE,     // 消抖状态
    KEY_STATE_PRESSED,      // 按下状态
    KEY_STATE_RELEASE       // 释放状态
} KeyStateMachine_t;

// 按键结构体
typedef struct {
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
    KeyStateMachine_t state;
    uint32_t debounce_timer;
    bool is_pressed;
    bool key_event;
} KeyInfo_t;

// 函数声明

void HAL_Key_Init_Timer(void);
void HAL_Key_Timer_IRQHandler(void);

KeyState_t HAL_Key_Scan(KeyMode_t mode);
bool HAL_Key_IsPressed(uint8_t key_num);
void HAL_Key_WaitForRelease(void);
void HAL_Key_Process(void);
#endif //KEY_H
