/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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
#define alarm_Pin GPIO_PIN_13
#define alarm_GPIO_Port GPIOC
#define DC_stop_time_Pin GPIO_PIN_0
#define DC_stop_time_GPIO_Port GPIOA
#define SpeedUP_Pin GPIO_PIN_1
#define SpeedUP_GPIO_Port GPIOA
#define InitPWM_Pin GPIO_PIN_2
#define InitPWM_GPIO_Port GPIOA
#define SimPh1_out_Pin GPIO_PIN_4
#define SimPh1_out_GPIO_Port GPIOA
#define SimPh2_out_Pin GPIO_PIN_5
#define SimPh2_out_GPIO_Port GPIOA
#define SimPh3_out_Pin GPIO_PIN_6
#define SimPh3_out_GPIO_Port GPIOA
#define Phase_C_Pin GPIO_PIN_7
#define Phase_C_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_0
#define Phase_A_GPIO_Port GPIOB
#define Phase_B_Pin GPIO_PIN_1
#define Phase_B_GPIO_Port GPIOB
#define start_DI_Pin GPIO_PIN_12
#define start_DI_GPIO_Port GPIOB
#define Start_enable_relay_Pin GPIO_PIN_13
#define Start_enable_relay_GPIO_Port GPIOB
#define stop_DCen_DI_Pin GPIO_PIN_14
#define stop_DCen_DI_GPIO_Port GPIOB
#define DC_on_DO_relay_Pin GPIO_PIN_15
#define DC_on_DO_relay_GPIO_Port GPIOB
#define Reverse_enable_DO_relay_Pin GPIO_PIN_8
#define Reverse_enable_DO_relay_GPIO_Port GPIOA
#define TIM2_CH1_phA_Pin GPIO_PIN_15
#define TIM2_CH1_phA_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define simulation 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
