//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/gimbal.h"

#include "key.h"


// 全局变量
static GimbalInit_t gimbal_init = {0};


void Gimbal_Calibration_Mode_Check(void)
{
    KeyEventType_t evt = HAL_Key_GetEvent(6); // Key 6 is KEY_BOARD

    if (!gimbal_calib_mode) {
        // Not in calibration mode, long press to enter
        if (evt == KEY_EVENT_LONG_PRESS) {
            gimbal_calib_mode = true;
            printf(">>> Entering gimbal calibration mode <<<\n");
            // Disable motors
            Emm_V5_En_Control(YAW_MOTOR_ADDR, false, false);
            Emm_V5_En_Control(PITCH_MOTOR_ADDR, false, false);
            printf("Motors disabled. Please manually adjust the gimbal position.\n");
        }
        return;
    }

    // In calibration mode
    if (evt == KEY_EVENT_SINGLE_CLICK) {
        printf("Recording current position as zero...\n");
        // Temporarily enable motors
        Emm_V5_En_Control(YAW_MOTOR_ADDR, true, false);
        Emm_V5_En_Control(PITCH_MOTOR_ADDR, true, false);
        HAL_Delay(100);
        // Set zero position
        Emm_V5_Reset_CurPos_To_Zero(YAW_MOTOR_ADDR);
        Emm_V5_Origin_Set_O(YAW_MOTOR_ADDR, true);
        Emm_V5_Reset_CurPos_To_Zero(PITCH_MOTOR_ADDR);
        Emm_V5_Origin_Set_O(PITCH_MOTOR_ADDR, true);
        HAL_Delay(100);
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

void Triggered_to_Zero() {
    KeyEventType_t evt = HAL_Key_GetEvent(6); // Key 6 is KEY_BOARD
    if (evt == KEY_EVENT_SINGLE_CLICK) {
        Emm_V5_Origin_Trigger_Return(YAW_MOTOR_ADDR,0,1);
        Emm_V5_Origin_Trigger_Return(PITCH_MOTOR_ADDR,0,1);
        Emm_V5_Synchronous_motion(0);
    }
}