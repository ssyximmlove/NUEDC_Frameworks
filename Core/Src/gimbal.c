//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/gimbal.h"

#include "key.h"


// --- 私有宏定义 ---
#define GIMBAL_CALIB_TIMEOUT_MS 10000 // 校准模式超时时间 (10秒)

// --- 模块级变量 ---
// 云台校准模式标志的定义
bool gimbal_calib_mode = false;

// 云台初始化状态 (当前未使用，为未来扩展保留)
static GimbalInit_t gimbal_init_status = {0};


// --- 私有辅助函数声明 ---
static void Gimbal_EnterCalibMode(void);
static void Gimbal_ExitCalibMode(void);
static void Gimbal_SetCurrentAsZero(void);
static void Gimbal_ReturnToZero(void);
static void Gimbal_SetMotorsEnable(bool enable);

/**
 * @brief 统一控制云台两个电机的使能状态
 * @param enable true: 使能, false: 失能
 */
static void Gimbal_SetMotorsEnable(bool enable)
{
    Emm_V5_En_Control(YAW_MOTOR_ADDR, enable, false);
    Emm_V5_En_Control(PITCH_MOTOR_ADDR, enable, false);
}

/**
 * @brief 进入云台校准模式的逻辑
 */
static void Gimbal_EnterCalibMode(void)
{
    gimbal_calib_mode = true;
    Gimbal_SetMotorsEnable(false);
    printf(">>> Entering gimbal calibration mode <<<\n");
    printf("Motors disabled. Please manually adjust the gimbal position.\n");
}

/**
 * @brief 退出云台校准模式的逻辑
 */
static void Gimbal_ExitCalibMode(void)
{
    gimbal_calib_mode = false;
    Gimbal_SetMotorsEnable(true);
    printf(">>> Exiting gimbal calibration mode <<<\n");
}

/**
 * @brief 将当前位置设置为新的零点
 */
static void Gimbal_SetCurrentAsZero(void)
{
    printf("Recording current position as zero...\n");
    // 1. 临时使能电机以锁定位置
    Gimbal_SetMotorsEnable(true);
    HAL_Delay(50); // 短暂延时确保电机使能生效

    // 2. 发送设置零点指令
    Emm_V5_Reset_CurPos_To_Zero(YAW_MOTOR_ADDR);
    Emm_V5_Origin_Set_O(YAW_MOTOR_ADDR, true); // 将当前位置设为可存储的硬件零点
    Emm_V5_Reset_CurPos_To_Zero(PITCH_MOTOR_ADDR);
    Emm_V5_Origin_Set_O(PITCH_MOTOR_ADDR, true);

    HAL_Delay(50); // 等待指令执行

    // 3. 再次失能电机，返回手动调整状态
    Gimbal_SetMotorsEnable(false);
    printf("Zero position set. Motors disabled again.\n");
}

/**
 * @brief 控制云台返回已设定的零点
 */
static void Gimbal_ReturnToZero(void)
{
    printf("Gimbal returning to zero position...\n");
    // 触发YAW和PITCH轴回零，模式0（就近），多机同步标志为true
    Emm_V5_Origin_Trigger_Return(YAW_MOTOR_ADDR, 0, true);
    Emm_V5_Origin_Trigger_Return(PITCH_MOTOR_ADDR, 0, true);
    // 广播地址0，触发所有设置了同步标志的电机同时运动
    Emm_V5_Synchronous_motion(0);
}


/**
 * @brief 云台校准模式状态机处理函数
 * @note  此函数应在定时器中断中以固定频率（如50Hz）调用
 */
void Gimbal_CalibModeHandler(void)
{
    static uint32_t calib_entry_time = 0;
    KeyEventType_t evt = HAL_Key_GetEvent(GIMBAL_CALIB_KEY_NUM);

    if (!gimbal_calib_mode) {
        // --- 正常模式下的按键处理 ---
        if (evt == KEY_EVENT_SINGLE_CLICK) {
            Gimbal_ReturnToZero();
        } else if (evt == KEY_EVENT_LONG_PRESS) {
            Gimbal_EnterCalibMode();
            calib_entry_time = HAL_GetTick();
        }
    } else {
        // --- 校准模式下的处理 ---
        // 检查超时
        if (HAL_GetTick() - calib_entry_time >= GIMBAL_CALIB_TIMEOUT_MS) {
            printf("Calibration timeout.\n");
            Gimbal_ExitCalibMode();
            return;
        }

        // 检查按键事件
        if (evt == KEY_EVENT_SINGLE_CLICK) {
            Gimbal_SetCurrentAsZero();
            // 设置零点后重置超时计时器，允许用户继续微调
            calib_entry_time = HAL_GetTick();
            printf("Timeout reset. You can continue adjusting or long-press to exit.\n");
        } else if (evt == KEY_EVENT_LONG_PRESS) {
            Gimbal_ExitCalibMode();
        }
    }
}