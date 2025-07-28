//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/gimbal.h"

#include "key.h"


// 全局变量
static GimbalInit_t gimbal_init = {0};
#define GIMBAL_CALIB_TIMEOUT_TICKS 500 // 10秒超时（50Hz*10s）


void Gimbal_CalibModeHandler(void)
{
    static uint16_t calib_timeout_cnt = 0;

    KeyEventType_t evt = HAL_Key_GetEvent(6); // Key 6 is KEY_BOARD

    if (!gimbal_calib_mode) {
        if (evt == KEY_EVENT_SINGLE_CLICK) {
            Emm_V5_Origin_Trigger_Return(YAW_MOTOR_ADDR, 0, 1);
            Emm_V5_Origin_Trigger_Return(PITCH_MOTOR_ADDR, 0, 1);
            Emm_V5_Synchronous_motion(0);
            printf("Back to Zero");
        }
        // Not in calibration mode, long press to enter
        else if (evt == KEY_EVENT_LONG_PRESS) {
            gimbal_calib_mode = true;
            printf(">>> Entering gimbal calibration mode <<<\n");
            // Disable motors
            Emm_V5_En_Control(YAW_MOTOR_ADDR, false, false);
            Emm_V5_En_Control(PITCH_MOTOR_ADDR, false, false);
            printf("Motors disabled. Please manually adjust the gimbal position.\n");
        }
        return;
    }

    calib_timeout_cnt++;
    if (calib_timeout_cnt >= GIMBAL_CALIB_TIMEOUT_TICKS) {
        gimbal_calib_mode = false;
        Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
        Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
        printf(">>> Calibration timeout, exiting mode <<<\n");
        calib_timeout_cnt = 0;
        return;
    }
    if (evt == KEY_EVENT_SINGLE_CLICK) {
        printf("Recording current position as zero...\n");
        // Temporarily enable motors
        Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
        Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
        // Set zero position
        Emm_V5_Reset_CurPos_To_Zero(YAW_MOTOR_ADDR);
        Emm_V5_Origin_Set_O(YAW_MOTOR_ADDR, true);
        Emm_V5_Reset_CurPos_To_Zero(PITCH_MOTOR_ADDR);
        Emm_V5_Origin_Set_O(PITCH_MOTOR_ADDR, true);
        // Disable motors again
        Emm_V5_En_Control(YAW_MOTOR_ADDR, false, false);
        Emm_V5_En_Control(PITCH_MOTOR_ADDR, false, false);
        printf("Zero position set. Motors disabled again.\n");
    } else if (evt == KEY_EVENT_LONG_PRESS) {
        gimbal_calib_mode = false;
        Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
        Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
        printf(">>> Exiting gimbal calibration mode <<<\n");
    }
}