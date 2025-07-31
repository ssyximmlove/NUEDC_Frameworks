//
// Created by Nymphaea on 25-7-28.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "Emm_V5.h"
#include "stdbool.h"
#include <stdio.h>
#include "arm_math.h"


// 按键定义
#define GIMBAL_CALIB_KEY_NUM 6

// 云台轴定义
#define YAW_MOTOR_ADDR      1
#define PITCH_MOTOR_ADDR    2

// 例如: 1.8度电机, 16细分 -> (360 / 1.8) * 16 = 3200
#define GIMBAL_STEPS_PER_REVOLUTION 3200
#define GIMBAL_ANGLE_PER_STEP       (360.0f / GIMBAL_STEPS_PER_REVOLUTION)

#define GIMBAL_CALIB_TIMEOUT_MS 10000 // 校准模式超时时间 (10秒)

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)


// 云台校准模式标志
extern bool gimbal_calib_mode;

// 云台状态枚举
typedef enum {
    GIMBAL_INIT_MANUAL_ADJUST = 0,  // 手动调整状态
    GIMBAL_INIT_POSITION_SET,       // 位置已设定状态
    GIMBAL_INIT_COMPLETED           // 初始化完成状态
} GimbalInitState_t;

// 云台初始化结构体
typedef struct {
    GimbalInitState_t state;        // 当前状态
    bool yaw_origin_set;            // Yaw轴零点是否已设定
    bool pitch_origin_set;          // Pitch轴零点是否已设定
    uint32_t init_start_time;       // 初始化开始时间
} GimbalInit_t;

typedef enum {
    GIMBAL_MODE_INIT = 0,
    GIMBAL_MODE_NORMAL
} GimbalMode_t;

// 函数声明
void Gimbal_CalibModeHandler(void);
void Gimbal_MovePolar(uint32_t radius, float angle, uint16_t speed);
void Gimbal_MoveToXYZ(float x, float y, float z, uint16_t speed);
#endif //GIMBAL_H
