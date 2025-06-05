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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define initPWM_Pin GPIO_PIN_0
#define initPWM_GPIO_Port GPIOA
#define DC_stop_time_Pin GPIO_PIN_1
#define DC_stop_time_GPIO_Port GPIOA
#define Speed_up_time_Pin GPIO_PIN_2
#define Speed_up_time_GPIO_Port GPIOA
#define EXTI_A_Pin GPIO_PIN_3
#define EXTI_A_GPIO_Port GPIOA
#define EXTI_A_EXTI_IRQn EXTI3_IRQn
#define EXTI_B_Pin GPIO_PIN_4
#define EXTI_B_GPIO_Port GPIOA
#define EXTI_B_EXTI_IRQn EXTI4_IRQn
#define EXTI_C_Pin GPIO_PIN_5
#define EXTI_C_GPIO_Port GPIOA
#define EXTI_C_EXTI_IRQn EXTI9_5_IRQn
#define PWM_1_Pin GPIO_PIN_2
#define PWM_1_GPIO_Port GPIOB
#define Shunt_DO_Pin GPIO_PIN_11
#define Shunt_DO_GPIO_Port GPIOB
#define Stop_DO_Pin GPIO_PIN_12
#define Stop_DO_GPIO_Port GPIOB
#define start_DI_Pin GPIO_PIN_13
#define start_DI_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_14
#define PWM_2_GPIO_Port GPIOB
#define PWM_3_Pin GPIO_PIN_11
#define PWM_3_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
typedef enum {
  Device_Idle                       = 0,     // 
  Device_Speed_up                   = 1,     // 
  Device_Shunt_on                   = 2,
  Device_Stop                       = 3,
} DeviceStatus_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
