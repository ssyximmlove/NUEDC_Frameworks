/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key_Board_Pin GPIO_PIN_0
#define Key_Board_GPIO_Port GPIOA
#define Laser_Pin GPIO_PIN_4
#define Laser_GPIO_Port GPIOC
#define LED_Board_Pin GPIO_PIN_2
#define LED_Board_GPIO_Port GPIOB
#define Servo_1_Pin GPIO_PIN_12
#define Servo_1_GPIO_Port GPIOD
#define Servo_2_Pin GPIO_PIN_13
#define Servo_2_GPIO_Port GPIOD
#define Servo_3_Pin GPIO_PIN_14
#define Servo_3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define LazerON HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,1);
#define LazerOFF HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,0);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
