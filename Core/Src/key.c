//
// Created by Nymphaea on 25-7-24.
//

#include "key.h"

#include <stdio.h>

// 按键信息数组
static KeyInfo_t keys[5] = {
    {GPIOE, GPIO_PIN_2, KEY_STATE_IDLE, 0, false, false},  // KEY1
    {GPIOE, GPIO_PIN_3, KEY_STATE_IDLE, 0, false, false},  // KEY2
    {GPIOE, GPIO_PIN_4, KEY_STATE_IDLE, 0, false, false},  // KEY3
    {GPIOE, GPIO_PIN_5, KEY_STATE_IDLE, 0, false, false},  // KEY4
    {GPIOE, GPIO_PIN_6, KEY_STATE_IDLE, 0, false, false}   // KEY5
};

static uint32_t last_scan_time = 0;

/**
 * @brief 单个按键状态机处理
 * @param key_info 按键信息指针
 */
static void key_state_machine(KeyInfo_t* key_info)
{
    bool current_level = (HAL_GPIO_ReadPin(key_info->gpio_port, key_info->gpio_pin) == GPIO_PIN_RESET);
    uint32_t current_time = HAL_GetTick();

    switch (key_info->state) {
        case KEY_STATE_IDLE:
            if (current_level) {
                key_info->state = KEY_STATE_DEBOUNCE;
                key_info->debounce_timer = current_time;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            if (current_level) {
                if ((current_time - key_info->debounce_timer) >= KEY_DEBOUNCE_TIME) {
                    key_info->state = KEY_STATE_PRESSED;
                    key_info->is_pressed = true;
                    key_info->key_event = true;
                }
            } else {
                key_info->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            if (!current_level) {
                key_info->state = KEY_STATE_RELEASE;
                key_info->debounce_timer = current_time;
            }
            break;

        case KEY_STATE_RELEASE:
            if (!current_level) {
                if ((current_time - key_info->debounce_timer) >= KEY_DEBOUNCE_TIME) {
                    key_info->state = KEY_STATE_IDLE;
                    key_info->is_pressed = false;
                }
            } else {
                key_info->state = KEY_STATE_PRESSED;
            }
            break;
    }
}

/**
 * @brief 按键处理函数（需要定时调用）
 */
void HAL_Key_Process(void)
{
    uint32_t current_time = HAL_GetTick();

    // 控制扫描频率，避免过于频繁
    if ((current_time - last_scan_time) < 5) {
        return;
    }
    last_scan_time = current_time;

    // 处理所有按键的状态机
    for (int i = 0; i < 5; i++) {
        key_state_machine(&keys[i]);
    }
}

/**
 * @brief 按键扫描函数
 * @param mode 扫描模式：单次或连续
 * @return 按键状态
 */
KeyState_t HAL_Key_Scan(KeyMode_t mode)
{
    // 处理状态机
    HAL_Key_Process();

    for (int i = 0; i < 5; i++) {
        if (mode == KEY_MODE_SINGLE) {
            // 单次模式：检测按键事件
            if (keys[i].key_event) {
                keys[i].key_event = false;  // 清除事件标志
                return (KeyState_t)(i + 1);
            }
        } else {
            // 连续模式：检测按键状态
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
    if (key_num < 1 || key_num > 5) {
        return false;
    }

    HAL_Key_Process();
    return keys[key_num - 1].is_pressed;
}

/**
 * @brief 等待所有按键释放
 */
void HAL_Key_WaitForRelease(void)
{
    do {
        HAL_Key_Process();
        HAL_Delay(10);
    } while (keys[0].is_pressed || keys[1].is_pressed || keys[2].is_pressed ||
             keys[3].is_pressed || keys[4].is_pressed);
}

// TODO: 五向按键测试
void key_test(void)
{
    static uint32_t test_counter = 0;
    static uint32_t continuous_counter = 0;

    printf("\n=== 按键测试 ===\n");
    printf("测试次数: %lu\n", ++test_counter);

    // 测试单次扫描模式
    KeyState_t single_key = HAL_Key_Scan(KEY_MODE_SINGLE);
    if (single_key != KEY_NONE) {
        printf("单次模式 - 按键%d被按下\n", single_key);

        switch(single_key) {
            case KEY1_PRESSED:
                printf("  -> 执行KEY1功能：菜单向上\n");
                break;
            case KEY2_PRESSED:
                printf("  -> 执行KEY2功能：菜单向下\n");
                break;
            case KEY3_PRESSED:
                printf("  -> 执行KEY3功能：确认选择\n");
                break;
            case KEY4_PRESSED:
                printf("  -> 执行KEY4功能：返回上级\n");
                break;
            case KEY5_PRESSED:
                printf("  -> 执行KEY5功能：设置模式\n");
                break;
            default:
                break;
        }
    }

    // 测试连续扫描模式
    KeyState_t continuous_key = HAL_Key_Scan(KEY_MODE_CONTINUOUS);
    if (continuous_key != KEY_NONE) {
        continuous_counter++;
        printf("连续模式 - 按键%d持续按下 (计数: %lu)\n", continuous_key, continuous_counter);

        if (continuous_key == KEY1_PRESSED) {
            printf("  -> 数值持续增加...\n");
        } else if (continuous_key == KEY2_PRESSED) {
            printf("  -> 数值持续减少...\n");
        }
    } else {
        if (continuous_counter > 0) {
            printf("连续按键结束，总计数: %lu\n", continuous_counter);
            continuous_counter = 0;
        }
    }

    // 测试单个按键状态检测
    for (int i = 1; i <= 5; i++) {
        if (HAL_Key_IsPressed(i)) {
            printf("按键%d当前处于按下状态\n", i);
        }
    }

    // 显示所有按键当前状态
    printf("按键状态: ");
    for (int i = 1; i <= 5; i++) {
        printf("K%d:%s ", i, HAL_Key_IsPressed(i) ? "按下" : "释放");
    }
    printf("\n");
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