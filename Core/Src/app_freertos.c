/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern ADC_HandleTypeDef hadc1;
DeviceStatus_t status=Device_Idle;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim15;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void SetPWMAll(uint16_t pwm);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartmainTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartmainTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartmainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartmainTask */
void StartmainTask(void *argument)
{
  /* USER CODE BEGIN StartmainTask */
  //шим где 0 это 100% а >9000 это ноль
  osDelay(2);
  uint16_t dma[3];
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,3);
  osDelay(2);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop_DMA(&hadc1);
  uint16_t init_pwm =9000-dma[0]*2;
  uint32_t dc_stop_time=(uint32_t)dma[1];         //0-4 секунды
  uint16_t speed_up_time=dma[2];        //0-4 секунды
  uint32_t timer;
  uint32_t diff;
  /* Infinite loop */
  for(;;)
  {
    switch (status){
    case Device_Idle:
      SetPWMAll(9001);
      if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_SET)
      {
        osDelay(2);
        if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_SET)
        {
          status=Device_Speed_up;
          SetPWMAll(init_pwm);
          timer=osKernelGetTickCount();
        }
      }
      break;
    case Device_Speed_up:
      if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_RESET)
      {
        osDelay(2);
        if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_RESET)
        {
          status=Device_Stop;
          break;
        }
      }
      diff=osKernelGetTickCount()-timer;
      if (diff<speed_up_time){
        float pwm=(float)init_pwm-(float)init_pwm*((float)diff/(float)speed_up_time);
        if (pwm < 0) pwm = 0;
        SetPWMAll((uint16_t)pwm);
      }else{
        status=Device_Shunt_on;
      }
      
      break;
    case Device_Shunt_on:
      SetPWMAll(0);
      if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_RESET)
      {
        osDelay(2);
        if(HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_RESET)
        {
          status=Device_Stop;
        }
      }
      break;
    case Device_Stop:
      SetPWMAll(9001);;
      osDelay(4);
      HAL_GPIO_WritePin(Stop_DO_GPIO_Port,Stop_DO_Pin,GPIO_PIN_SET);
      osDelay(dc_stop_time);
      HAL_GPIO_WritePin(Stop_DO_GPIO_Port,Stop_DO_Pin,GPIO_PIN_RESET);
      status=Device_Idle;
      break;      
    }
    
    osDelay(1);
    HAL_GPIO_TogglePin(Led_GPIO_Port,Led_Pin);
  }
  /* USER CODE END StartmainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void SetPWMAll(uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pwm);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
}
/* USER CODE END Application */

