//
// Created by Nymphaea on 25-7-28.
//

#include "../Inc/isr.h"

#include "tim.h"

/**
 * @brief 初始化按键定时器
 */
void HAL_Timer6_Init(void) {
    HAL_TIM_Base_Start_IT(&htim6);
    printf("50Hz Key Interrupt Started\n");
}

/**
 * @brief 定时器中断回调函数（f = 50Hz)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_Key_Timer_IRQHandler();
        Gimbal_Calibration_Mode_Check();
    }
}