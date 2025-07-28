//
// Created by Nymphaea on 25-7-28.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "Emm_V5.h"
#include "stdbool.h"
#include <stdio.h>


// 云台轴定义
#define YAW_MOTOR_ADDR      1
#define PITCH_MOTOR_ADDR    2

// 云台校准模式标志
static bool gimbal_calib_mode = false;

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
void Gimbal_Calibration_Mode_Check(void);
void Triggered_to_Zero(void);

#endif //GIMBAL_H
