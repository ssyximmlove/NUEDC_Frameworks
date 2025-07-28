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

#define KEY_BOARD_Port GPIOA
#define KEY_BOARD_Pin GPIO_PIN_0

// 消抖时间定义（ms）
#define KEY_DEBOUNCE_TIME 20
#define KEY_LONG_PRESS_TIME    800
#define KEY_DOUBLE_CLICK_GAP   300

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SINGLE_CLICK,
    KEY_EVENT_DOUBLE_CLICK,
    KEY_EVENT_LONG_PRESS
} KeyEventType_t;

// 按键状态定义
typedef enum {
    KEY_NONE = 0,
    KEY1_PRESSED,
    KEY2_PRESSED,
    KEY3_PRESSED,
    KEY4_PRESSED,
    KEY5_PRESSED,
    KEY_BOARD_PRESSED,
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
    bool active_level; // true: 低电平有效，false: 高电平有效
    uint32_t last_press_time;
    uint32_t last_release_time;
    uint8_t click_count;
    bool long_press_reported;
    KeyEventType_t event_type;
} KeyInfo_t;

// 函数声明

void HAL_Key_Init_Timer(void);
void HAL_Key_Timer_IRQHandler(void);

KeyState_t HAL_Key_Scan(KeyMode_t mode);
bool HAL_Key_IsPressed(uint8_t key_num);
void HAL_Key_WaitForRelease(void);
void HAL_Key_Process(void);
KeyEventType_t HAL_Key_GetEvent(uint8_t key_num);

void key_test(void); // 测试函数
#endif //KEY_H
