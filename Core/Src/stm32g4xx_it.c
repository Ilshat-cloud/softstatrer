/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern uint16_t ARR_TIM;
uint8_t Phase_A_flag=0;
uint8_t Phase_B_flag=0;
uint8_t Phase_C_flag=0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim17;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
  HAL_GPIO_WritePin(PWM_A_GPIO_Port,PWM_A_Pin,GPIO_PIN_RESET);
  if (ARR_TIM<8000){
    HAL_TIM_Base_Stop_IT(&htim3);
    if(ARR_TIM>10)
    {
      TIM3->CNT=0;
      TIM3->ARR=ARR_TIM;
      Phase_A_flag=1;
      HAL_TIM_Base_Start_IT(&htim3);  
    }else{
      HAL_GPIO_WritePin(PWM_A_GPIO_Port,PWM_A_Pin,GPIO_PIN_SET);
    }
  }
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXTI_A_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  HAL_GPIO_WritePin(PWM_B_GPIO_Port,PWM_B_Pin,GPIO_PIN_RESET);
  if (ARR_TIM<8000){
    HAL_TIM_Base_Stop_IT(&htim4);
    if(ARR_TIM>10)
    {
      TIM4->CNT=0;
      TIM4->ARR=ARR_TIM;
      Phase_B_flag=1;
      HAL_TIM_Base_Start_IT(&htim4);  
    }else{
      HAL_GPIO_WritePin(PWM_B_GPIO_Port,PWM_B_Pin,GPIO_PIN_SET);
    }
  }
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXTI_B_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  HAL_GPIO_WritePin(PWM_C_GPIO_Port,PWM_C_Pin,GPIO_PIN_RESET);
  if (ARR_TIM<8000){
    HAL_TIM_Base_Stop_IT(&htim8);
    if(ARR_TIM>10)
    {
      TIM8->CNT=0;
      TIM8->ARR=ARR_TIM;
      Phase_C_flag=1;
      HAL_TIM_Base_Start_IT(&htim8);  
    }else{
      HAL_GPIO_WritePin(PWM_C_GPIO_Port,PWM_C_Pin,GPIO_PIN_SET);
    }
  }
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXTI_C_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(Phase_A_flag==1){ 
    HAL_GPIO_WritePin(PWM_A_GPIO_Port,PWM_A_Pin,GPIO_PIN_SET);
    TIM3->ARR=500;
    Phase_A_flag=0;
  }else{
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_GPIO_WritePin(PWM_A_GPIO_Port,PWM_A_Pin,GPIO_PIN_RESET);
  }
  
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  if(Phase_B_flag==1){ 
    HAL_GPIO_WritePin(PWM_B_GPIO_Port,PWM_B_Pin,GPIO_PIN_SET);
    TIM4->ARR=500;
    Phase_B_flag=0;
  }else{
    HAL_TIM_Base_Stop_IT(&htim4);
    HAL_GPIO_WritePin(PWM_B_GPIO_Port,PWM_B_Pin,GPIO_PIN_RESET);
  }

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */

  /* USER CODE END TIM8_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */
  if(Phase_C_flag==1){ 
    HAL_GPIO_WritePin(PWM_C_GPIO_Port,PWM_C_Pin,GPIO_PIN_SET);
    TIM8->ARR=500;
    Phase_C_flag=0;
  }else{
    HAL_TIM_Base_Stop_IT(&htim8);
    HAL_GPIO_WritePin(PWM_C_GPIO_Port,PWM_C_Pin,GPIO_PIN_RESET);
  }
  /* USER CODE END TIM8_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
