//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/gimbal.h"

#include <stdlib.h>
#include "arm_math.h"
#include "key.h"


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
static void Gimbal_RectTest(void);
static void Gimbal_CircleTest(void);
static void Gimbal_VelocityCircleTest();
static void Gimbal_VelocityLineTest(int32_t start_yaw, int32_t start_pitch, int32_t end_yaw, int32_t end_pitch, float32_t duration_s);

/**
 * @brief 云台校准模式状态机处理函数
 */
void Gimbal_CalibModeHandler(void)
{
    static uint32_t calib_entry_time = 0;
    KeyEventType_t evt = HAL_Key_GetEvent(GIMBAL_CALIB_KEY_NUM);

    if (!gimbal_calib_mode) {
        // --- 正常模式下的按键处理 ---
        if (evt == KEY_EVENT_SINGLE_CLICK) {
            Gimbal_ReturnToZero();
        }
        else if (evt == KEY_EVENT_DOUBLE_CLICK) {
            // [测试] 在双击时运行完整的测试序列
            Gimbal_RectTest();
            // Gimbal_CircleTest();
            // Gimbal_VelocityCircleTest();
            // Gimbal_VelocityLineTest(0, 0, 10000, 50000, 5.0f);
        }
        else if (evt == KEY_EVENT_LONG_PRESS) {
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

void Gimbal_MoveToAbsolute(int32_t yaw_pos, int32_t pitch_pos, uint16_t speed)
{
    uint8_t acc = 50;
    printf("Moving to (YAW: %ld, PITCH: %ld)\n", yaw_pos, pitch_pos);

    // 正确处理方向和步数
    uint8_t yaw_dir = (yaw_pos >= 0) ? 0 : 1;
    uint32_t yaw_steps = abs(yaw_pos);
    uint8_t pitch_dir = (pitch_pos >= 0) ? 0 : 1;
    uint32_t pitch_steps = abs(pitch_pos);

    // 配置电机指令，使用绝对位置模式(raF=true)和同步启动模式(snF=true)
    Emm_V5_Pos_Control(YAW_MOTOR_ADDR, yaw_dir, speed, acc, yaw_steps, true, true);
    Emm_V5_Pos_Control(PITCH_MOTOR_ADDR, pitch_dir, speed, acc, pitch_steps, true, true);

    // 使用广播地址0触发所有同步标记的电机同时运动
    Emm_V5_Synchronous_motion(0);
}
static void Gimbal_InverseKinematics(float32_t target_x, float32_t target_y, float32_t target_z, int32_t *steps_yaw, int32_t *steps_pitch)
{
    float32_t distance_xy;
    float32_t yaw_angle_rad, pitch_angle_rad;
    float32_t yaw_angle_deg, pitch_angle_deg;

    // 计算XY平面距离: distance_xy = sqrt(x^2 + y^2)
    // [优化] 使用 arm_sqrt_f32 替代 sqrtf
    arm_sqrt_f32(target_x * target_x + target_y * target_y, &distance_xy);

    // 计算Yaw角(水平旋转角) in radians
    // [优化] 使用 arm_atan2_f32 替代 atan2f
    // 注意：arm_atan2_f32 需要一个结果指针
    arm_status status = arm_atan2_f32(target_y, target_x, &yaw_angle_rad);
    if (status != ARM_MATH_SUCCESS) {
        // 处理错误，例如设置为0
        yaw_angle_rad = 0.0f;
    }

    // 计算Pitch角(垂直仰角) in radians
    // [优化] 使用 arm_atan2_f32 替代 atan2f
    status = arm_atan2_f32(target_z, distance_xy, &pitch_angle_rad);
    if (status != ARM_MATH_SUCCESS) {
        pitch_angle_rad = 0.0f;
    }

    // 将弧度转换为角度
    yaw_angle_deg = yaw_angle_rad * RAD_TO_DEG;
    pitch_angle_deg = pitch_angle_rad * RAD_TO_DEG;

    // 将角度转换为有符号的步数
    *steps_yaw = (int32_t)(yaw_angle_deg / GIMBAL_ANGLE_PER_STEP);
    *steps_pitch = (int32_t)(pitch_angle_deg / GIMBAL_ANGLE_PER_STEP);
}
/**
 * @brief [移植并修复] 使用极坐标方式控制云台运动
 */
void Gimbal_MovePolar(uint32_t radius, float32_t angle, uint16_t speed)
{
    float32_t rad = angle * DEG_TO_RAD;

    // [优化] 使用 arm_cos_f32 和 arm_sin_f32 替代 cosf 和 sinf
    int32_t x = (int32_t)(radius * arm_cos_f32(rad));
    int32_t y = (int32_t)(radius * arm_sin_f32(rad)); // Pitch轴通常对应Y

    Gimbal_MoveToAbsolute(x, y, speed);
}

void Gimbal_MoveToXYZ(float32_t x, float32_t y, float32_t z, uint16_t speed)
{
    int32_t steps_yaw, steps_pitch;
    Gimbal_InverseKinematics(x, y, z, &steps_yaw, &steps_pitch);
    Gimbal_MoveToAbsolute(steps_yaw, steps_pitch, speed);
}


static void Gimbal_RectTest(void)
{
    // --- 参数定义 ---
    const uint32_t side_length = 40;    // 定义一个较小的边长 (单位: 步数)
    const uint16_t speed = 10;          // 定义一个合适的运动速度 (单位: RPM)
    const uint32_t move_delay_ms = 1000; // 等待每条边运动完成的延时

    printf("\n\n--- GIMBAL SQUARE TRACE TEST ---\n");
    printf("Side: %lu steps, Speed: %u RPM\n", side_length, speed);

    // 计算半边长，用于定义坐标
    const int32_t half_side = side_length / 2;

    // --- 定义正方形的四个角点坐标 (Yaw, Pitch) ---
    // 1: 右上, 2: 左上, 3: 左下, 4: 右下
    const int32_t corner1[2] = {  half_side,  half_side };
    const int32_t corner2[2] = { -half_side,  half_side };
    const int32_t corner3[2] = { -half_side, -half_side };
    const int32_t corner4[2] = {  half_side, -half_side };

    // --- 开始画图 ---
    // 1. 移动到起始点 (右下角)
    printf("1. Moving to starting corner (Bottom-Right)...\n");
    Gimbal_MoveToAbsolute(corner4[0], corner4[1], speed);
    HAL_Delay(move_delay_ms);

    // 2. 移动到右上角 (向上运动)
    printf("2. Tracing side 1 (Up)...\n");
    Gimbal_MoveToAbsolute(corner1[0], corner1[1], speed);
    HAL_Delay(move_delay_ms);

    // 3. 移动到左上角 (向左运动)
    printf("3. Tracing side 2 (Left)...\n");
    Gimbal_MoveToAbsolute(corner2[0], corner2[1], speed);
    HAL_Delay(move_delay_ms);

    // 4. 移动到左下角 (向下运动)
    printf("4. Tracing side 3 (Down)...\n");
    Gimbal_MoveToAbsolute(corner3[0], corner3[1], speed);
    HAL_Delay(move_delay_ms);

    // 5. 移动到右下角 (向右运动，完成闭环)
    printf("5. Tracing side 4 (Right)...\n");
    Gimbal_MoveToAbsolute(corner4[0], corner4[1], speed);
    HAL_Delay(move_delay_ms);

    // 6. 测试完成，返回零点
    printf("6. Square trace complete. Returning to zero...\n");
    Gimbal_ReturnToZero();
    HAL_Delay(move_delay_ms);

    printf("--- GIMBAL SQUARE TRACE TEST COMPLETE ---\n\n");
}
static void Gimbal_CircleTest(void)
{
    // --- 参数定义 ---
    const uint32_t radius = 100;          // 定义半径 (单位: 步数)
    const uint16_t speed = 100;           // 定义一个合适的运动速度 (单位: RPM)
    const uint16_t num_points = 720;
    const uint32_t point_delay_ms = 10; // 每个点之间的短暂延时，使运动更平滑

    printf("\n\n--- GIMBAL CIRCLE TRACE TEST ---\n");
    printf("Radius: %lu steps, Speed: %u RPM, Points: %u\n", radius, speed, num_points);

    // --- 准备工作：移动到圆的起始点 (3点钟方向) ---
    printf("1. Moving to starting point (3 o'clock)...\n");
    Gimbal_MoveToAbsolute(radius, 0, speed);
    HAL_Delay(2000); // 等待移动到起始点

    // --- 开始画圆 ---
    printf("2. Tracing circle...\n");
    const float angle_increment_rad = (2.0f * PI) / num_points;

    for (int i = 1; i <= num_points; i++) {
        float current_angle_rad = i * angle_increment_rad;

        // 计算目标坐标 (Yaw, Pitch)
        int32_t x = (int32_t)(radius * cosf(current_angle_rad));
        int32_t y = (int32_t)(radius * sinf(current_angle_rad));

        // 移动到下一个点
        Gimbal_MoveToAbsolute(x, y, speed);

        // 短暂延时，让电机有时间移动
        HAL_Delay(point_delay_ms);
    }

    // --- 测试完成，返回零点 ---
    printf("3. Circle trace complete. Returning to zero...\n");
    Gimbal_ReturnToZero();
    HAL_Delay(2500);

    printf("--- GIMBAL CIRCLE TRACE TEST COMPLETE ---\n\n");
}

static void Gimbal_VelocityCircleTest(void)
{
    // --- 核心参数 ---
    const float32_t radius_steps = 200.0f;      // [优化] 增大半径，效果更明显
    const float32_t circle_duration_s = 10.0f;  // 画完一圈需要的时间 (秒)
    const uint16_t update_rate_hz = 50;         // [核心优化] 大幅提高更新频率，让轨迹更平滑
    const float32_t dt_s = 1.0f / update_rate_hz; // 每次更新的时间步长 (秒)
    const float32_t angular_velocity_rad_s = (2.0f * PI) / circle_duration_s; // 角速度 ω (弧度/秒)

    printf("\n\n--- GIMBAL VELOCITY CIRCLE TEST ---\n");
    printf("Radius: %.0f steps, Duration: %.1fs, Update Rate: %u Hz\n", radius_steps, circle_duration_s, update_rate_hz);
    printf(">>> Press button %d again to stop <<<\n", GIMBAL_CALIB_KEY_NUM);
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);

    // --- 转换系数 ---
    // (步/秒) -> RPM:  (steps/sec) / (steps/rev) * (60 sec/min)
    const float32_t steps_per_sec_to_rpm = 60.0f / GIMBAL_STEPS_PER_REVOLUTION;

    float32_t current_time_s = 0.0f;
    bool should_stop = false;

    // --- 循环画圆 ---
    while (current_time_s < circle_duration_s && !should_stop)
    {
        // [BUG修复] 检查停止信号：只要有任何按键事件，就停止
        if (HAL_Key_GetEvent(GIMBAL_CALIB_KEY_NUM) != KEY_EVENT_NONE) {
            should_stop = true;
            continue;
        }

        float32_t current_angle_rad = angular_velocity_rad_s * current_time_s;

        // 计算两轴的目标切线速度 (单位: 步/秒)
        // v_yaw = d(R*cos(ωt))/dt = -R*ω*sin(ωt)
        // v_pitch = d(R*sin(ωt))/dt = R*ω*cos(ωt)
        float32_t yaw_vel_sps = -radius_steps * angular_velocity_rad_s * arm_sin_f32(current_angle_rad);
        float32_t pitch_vel_sps = radius_steps * angular_velocity_rad_s * arm_cos_f32(current_angle_rad);

        // 将速度转换为电机指令 (RPM + 方向)
        float32_t abs_yaw_vel, abs_pitch_vel;
        arm_abs_f32(&yaw_vel_sps, &abs_yaw_vel, 1);
        arm_abs_f32(&pitch_vel_sps, &abs_pitch_vel, 1);

        uint16_t yaw_rpm = (uint16_t)(abs_yaw_vel * steps_per_sec_to_rpm);
        uint16_t pitch_rpm = (uint16_t)(abs_pitch_vel * steps_per_sec_to_rpm);
        uint8_t yaw_dir = (yaw_vel_sps < 0) ? 1 : 0;
        uint8_t pitch_dir = (pitch_vel_sps >= 0) ? 0 : 1;

        // 限制最高速度，防止参数设置过激
        if (yaw_rpm > 200) yaw_rpm = 200;
        if (pitch_rpm > 200) pitch_rpm = 200;

        // 发送速度控制指令 (snF=false, 立即执行)
        Emm_V5_Vel_Control(YAW_MOTOR_ADDR, yaw_dir, yaw_rpm, 10, false);
        Emm_V5_Vel_Control(PITCH_MOTOR_ADDR, pitch_dir, pitch_rpm, 10, false);

        // 等待下一个时间步
        HAL_Delay((uint32_t)(dt_s * 1000));
        current_time_s += dt_s;
    }

    // --- 测试结束，停止电机并归零 ---
    printf("\nVelocity circle test finished. Stopping motors...\n");
    // [BUG修复] 确保激光可靠关闭
    HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
    Emm_V5_Stop_Now(YAW_MOTOR_ADDR, false);
    Emm_V5_Stop_Now(PITCH_MOTOR_ADDR, false);

    HAL_Delay(500);

    printf("Returning to zero...\n");
    Gimbal_ReturnToZero();
    HAL_Delay(2000);

    printf("--- GIMBAL VELOCITY CIRCLE TEST COMPLETE ---\n\n");
}

static void Gimbal_VelocityLineTest(int32_t start_yaw, int32_t start_pitch, int32_t end_yaw, int32_t end_pitch, float32_t duration_s)
{
    // 步数差
    int32_t delta_yaw = end_yaw - start_yaw;
    int32_t delta_pitch = end_pitch - start_pitch;

    // 总步数
    float32_t total_steps = arm_sqrt_f32((float32_t)(delta_yaw * delta_yaw + delta_pitch * delta_pitch), &total_steps);

    // 每轴速度（步/秒）
    float32_t yaw_vel = delta_yaw / duration_s;
    float32_t pitch_vel = delta_pitch / duration_s;

    // 步/秒转RPM
    const float32_t steps_to_rpm = 60.0f / GIMBAL_STEPS_PER_REVOLUTION;

    // 方向
    uint8_t yaw_dir = (yaw_vel < 0) ? 1 : 0;
    uint8_t pitch_dir = (pitch_vel < 0) ? 1 : 0;

    // 绝对值
    float32_t abs_yaw, abs_pitch;
    arm_abs_f32(&yaw_vel, &abs_yaw, 1);
    arm_abs_f32(&pitch_vel, &abs_pitch, 1);
    uint16_t yaw_rpm = (uint16_t)(abs_yaw * steps_to_rpm);
    uint16_t pitch_rpm = (uint16_t)(abs_pitch * steps_to_rpm);

    // 先切换为速度模式并使能
    Emm_V5_Modify_Ctrl_Mode(YAW_MOTOR_ADDR, false, 2);
    Emm_V5_Modify_Ctrl_Mode(PITCH_MOTOR_ADDR, false, 2);
    Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
    Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
    HAL_Delay(100);

    float32_t elapsed = 0.0f;
    const uint16_t update_rate_hz = 50;
    const float32_t dt_s = 1.0f / update_rate_hz;

    printf("\n--- GIMBAL VELOCITY LINE TEST ---\n");

    while (elapsed < duration_s) {
        Emm_V5_Vel_Control(YAW_MOTOR_ADDR, yaw_dir, yaw_rpm, 10, false);
        Emm_V5_Vel_Control(PITCH_MOTOR_ADDR, pitch_dir, pitch_rpm, 10, false);

        if (HAL_Key_GetEvent(GIMBAL_CALIB_KEY_NUM) == KEY_EVENT_SINGLE_CLICK) {
            break;
        }

        HAL_Delay((uint32_t)(dt_s * 1000));
        elapsed += dt_s;
    }

    Emm_V5_Stop_Now(YAW_MOTOR_ADDR, false);
    Emm_V5_Stop_Now(PITCH_MOTOR_ADDR, false);

    printf("--- VELOCITY LINE TEST END ---\n");
}
