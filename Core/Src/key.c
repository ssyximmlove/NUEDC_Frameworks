//
// Created by Nymphaea on 25-7-24.
//

#include "key.h"

#include <stdio.h>

#include "tim.h"

// 按键信息数组
static KeyInfo_t keys[6] = {

    {GPIOE, GPIO_PIN_2, KEY_STATE_IDLE, 0, false, false,true},  // KEY1
    {GPIOE, GPIO_PIN_3, KEY_STATE_IDLE, 0, false, false,true},  // KEY2
    {GPIOE, GPIO_PIN_4, KEY_STATE_IDLE, 0, false, false,true},  // KEY3
    {GPIOE, GPIO_PIN_5, KEY_STATE_IDLE, 0, false, false,true},  // KEY4
    {GPIOE, GPIO_PIN_6, KEY_STATE_IDLE, 0, false, false,true},   // KEY5
{GPIOA,GPIO_PIN_0,KEY_STATE_IDLE,0,false,false,false}, // KEY_BOARD
};

/**
 * @brief 初始化按键定时器
 */
void HAL_Key_Init_Timer(void) {
    HAL_TIM_Base_Start_IT(&htim6);
    printf("50Hz Key Interrupt Started\n");
}


/**
 * @brief 单个按键状态机处理
 * @param key_info 按键信息指针
 */
static void key_state_machine(KeyInfo_t* key_info)
{
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(key_info->gpio_port, key_info->gpio_pin);
    bool current_level = key_info->active_level ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
    uint32_t now = HAL_GetTick();

    switch (key_info->state) {
        case KEY_STATE_IDLE:
            if (current_level) {
                key_info->state = KEY_STATE_DEBOUNCE;
                key_info->debounce_timer = now;
            }
            // 检查单击/双击超时
            if (key_info->click_count == 1 && (now - key_info->last_release_time > KEY_DOUBLE_CLICK_GAP)) {
                key_info->event_type = KEY_EVENT_SINGLE_CLICK;
                key_info->click_count = 0;
            }
            break;
        case KEY_STATE_DEBOUNCE:
            if (current_level) {
                if ((now - key_info->debounce_timer) >= KEY_DEBOUNCE_TIME) {
                    key_info->state = KEY_STATE_PRESSED;
                    key_info->is_pressed = true;
                    key_info->key_event = true;
                    key_info->last_press_time = now;
                    key_info->long_press_reported = false;
                }
            } else {
                key_info->state = KEY_STATE_IDLE;
            }
            break;
        case KEY_STATE_PRESSED:
            // 长按检测
            if (!key_info->long_press_reported && (now - key_info->last_press_time > KEY_LONG_PRESS_TIME)) {
                key_info->event_type = KEY_EVENT_LONG_PRESS;
                key_info->long_press_reported = true;
                key_info->click_count = 0; // 长按不再计入单/双击
            }
            if (!current_level) {
                key_info->state = KEY_STATE_RELEASE;
                key_info->debounce_timer = now;
                key_info->is_pressed = false;
                key_info->last_release_time = now;
                if (!key_info->long_press_reported) {
                    key_info->click_count++;
                    if (key_info->click_count == 2 && (now - key_info->last_release_time < KEY_DOUBLE_CLICK_GAP)) {
                        key_info->event_type = KEY_EVENT_DOUBLE_CLICK;
                        key_info->click_count = 0;
                    }
                }
            }
            break;
        case KEY_STATE_RELEASE:
            if (!current_level) {
                if ((now - key_info->debounce_timer) >= KEY_DEBOUNCE_TIME) {
                    key_info->state = KEY_STATE_IDLE;
                }
            } else {
                key_info->state = KEY_STATE_PRESSED;
                key_info->last_press_time = now;
                key_info->long_press_reported = false;
            }
            break;
    }
}


/**
 * @brief 按键扫描函数
 * @param mode 扫描模式：单次或连续
 * @return 按键状态
 */
KeyState_t HAL_Key_Scan(KeyMode_t mode)
{
    for (int i = 0; i < 6; i++) {
        if (mode == KEY_MODE_SINGLE) {
            if (keys[i].key_event) {
                keys[i].key_event = false;
                return (KeyState_t)(i + 1);
            }
        } else {
            if (keys[i].is_pressed) {
                return (KeyState_t)(i + 1);
            }
        }
    }
    return KEY_NONE;
}
/**
 * @brief 检测指定按键是否被按下
 * @param key_num 按键编号 (1-5)
 * @return true: 按下, false: 未按下
 */
bool HAL_Key_IsPressed(uint8_t key_num)
{
    if (key_num < 1 || key_num > 6) {
        return false;
    }
    return keys[key_num - 1].is_pressed;
}

/**
 * @brief 等待所有按键释放
 */
void HAL_Key_WaitForRelease(void)
{
    do {
        HAL_Delay(10);
    } while (
        keys[0].is_pressed || keys[1].is_pressed || keys[2].is_pressed ||
        keys[3].is_pressed || keys[4].is_pressed || keys[5].is_pressed
    );
}


/**
 * @brief 定时器中断回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_Key_Timer_IRQHandler();
    }
}
/**
 * @brief 按键定时器中断处理函数
 */
void HAL_Key_Timer_IRQHandler(void)
{
    // 直接处理按键状态机，无需频率控制
    for (int i = 0; i < 6; i++) {
        key_state_machine(&keys[i]);
    }
}

KeyEventType_t HAL_Key_GetEvent(uint8_t key_num)
{
    if (key_num < 1 || key_num > 6) return KEY_EVENT_NONE;
    KeyEventType_t evt = keys[key_num-1].event_type;
    keys[key_num-1].event_type = KEY_EVENT_NONE;
    return evt;
}

// 五向按键测试
void key_test(void)
{
    static uint32_t continuous_counter = 0;
    static bool last_any_pressed = false;
    static uint32_t last_hold_update = 0;

    KeyState_t continuous_key = HAL_Key_Scan(KEY_MODE_CONTINUOUS);
    uint32_t now = HAL_GetTick();

    if (continuous_key != KEY_NONE) {
        if (!last_any_pressed) {
            last_hold_update = now;
            continuous_counter = 0;
            last_any_pressed = true;
            printf("[Key] Continuous mode - Key %d held\n", continuous_key);
        }
        // 每秒加一
        if (now - last_hold_update >= 1000) {
            continuous_counter++;
            last_hold_update += 1000;
            printf("[Key] Hold count: %lu\n", continuous_counter);
        }
    } else {
        if (last_any_pressed) {
            printf("[Key] All keys released, total hold count: %lu\n", continuous_counter);
            last_any_pressed = false;
            continuous_counter = 0;
        }
    }

    // 检测单击、双击、长按
    for (int i = 0; i < 6; i++) {
        KeyEventType_t evt = HAL_Key_GetEvent(i+1);
        if (evt == KEY_EVENT_SINGLE_CLICK) {
            printf("[Key] Key %d Single\n", i+1);
        } else if (evt == KEY_EVENT_DOUBLE_CLICK) {
            printf("[Key] Key %d Double\n", i+1);
        } else if (evt == KEY_EVENT_LONG_PRESS) {
            printf("[Key] Key %d Long Press\n", i+1);
        }
    }
}

/**

// 在main函数中的使用示例
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // 初始化UART用于printf输出
    MX_USART1_UART_Init();

    printf("按键状态机测试开始...\n");
    printf("请按下按键进行测试\n");
    printf("KEY1-KEY5分别对应GPIOE的PIN2-PIN6\n\n");

    while(1)
    {
        // 运行按键测试
        key_test();

        // 特殊功能测试：等待所有按键释放
        if (HAL_Key_IsPressed(1) && HAL_Key_IsPressed(2)) {
            printf("检测到KEY1+KEY2组合按键，等待释放...\n");
            HAL_Key_WaitForRelease();
            printf("所有按键已释放\n");
        }

        HAL_Delay(100);  // 100ms测试间隔
    }
}

**/