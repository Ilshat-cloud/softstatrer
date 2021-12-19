/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t phase_a_work=0,phase_b_work=0,phase_c_work=0,DC_stop_en=0, Start_time, Stop_time, init_pwm, start=0;
uint16_t PWM=0, time_from_interrupt=0;
uint8_t Time_b, Time_c, Time_a=0;

/* USER CODE END Variables */
/* Definitions for main_task */
osThreadId_t main_taskHandle;
const osThreadAttr_t main_task_attributes = {
  .name = "main_task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 128 * 4
};
/* Definitions for StopTimer */
osTimerId_t StopTimerHandle;
const osTimerAttr_t StopTimer_attributes = {
  .name = "StopTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void main_func(void *argument);
void StopTimer01(void *argument);

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

  /* Create the timer(s) */
  /* creation of StopTimer */
  StopTimerHandle = osTimerNew(StopTimer01, osTimerOnce, NULL, &StopTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(main_func, NULL, &main_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_main_func */
/**
  * @brief  Function implementing the main_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_func */
void main_func(void *argument)
{
  /* USER CODE BEGIN main_func */
  volatile uint16_t dma[3];
  HAL_GPIO_WritePin(alarm_GPIO_Port,alarm_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Start_enable_relay_GPIO_Port,Start_enable_relay_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_on_DO_relay_GPIO_Port,DC_on_DO_relay_Pin, GPIO_PIN_RESET);    
  HAL_GPIO_WritePin(Reverse_enable_DO_relay_GPIO_Port,Reverse_enable_DO_relay_Pin, GPIO_PIN_RESET);
  if (HAL_GPIO_ReadPin(stop_DCen_DI_GPIO_Port,stop_DCen_DI_Pin)==GPIO_PIN_SET)
  {
      DC_stop_en=1;
  }
  HAL_TIM_Base_Start(&htim3);  //this one for phase control
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,3);
  osDelay(2);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop_DMA(&hadc1);
  Start_time=dma[1]/4;  //10 ms per quant of time (100Hz interrupt)  1024 equal to 10 s
  Stop_time=dma[0]/8;  //10 ms per quant of time (100Hz interrupt)  512 equal to 5 s
  init_pwm=300+dma[2]/8;  //  from 30% to 70%  1000==100%
  
  osDelay(10);

  
  /* Infinite loop */
  for(;;)
  {

    if (HAL_GPIO_ReadPin(start_DI_GPIO_Port,start_DI_Pin)==GPIO_PIN_SET)
      {
        if (start==1)
        {
          start=2; //start ==2 ON command second cicle
          time_from_interrupt=0;
        }else{
          start=1;
        }          
      }else{
        if ((start!=3)&(start==2)|(start==4))//start ==3 OFF command 
          {
            start=3;
            time_from_interrupt=0;
          }
        
      } 

    if (phase_a_work&(phase_c_work|phase_b_work)) //if phases more than one
    {
        if (Time_b<Time_c) //error phases
        {
          HAL_GPIO_WritePin(alarm_GPIO_Port,alarm_Pin, GPIO_PIN_RESET);
        } else{
          HAL_GPIO_WritePin(alarm_GPIO_Port,alarm_Pin, GPIO_PIN_SET);
          start=0;          
        }
    }
    
    //-------------------start command -------------------------------------//
    if (start==2)
    {
      //wait for a start time over
      if ((time_from_interrupt<Start_time)&(Start_time>0)) // speed up
        {
          PWM=(1000-init_pwm)*(time_from_interrupt/Start_time)+init_pwm;
          PWM=(PWM>1000)?1000:PWM;
        }else{
          PWM=1000;
          start=4;     
        }

    }
    //========================================================================//
    
    //-------------------start relay --------------------------------//
    if (start==4)  // start ==4 means started 
    {
      HAL_GPIO_WritePin(Start_enable_relay_GPIO_Port,Start_enable_relay_Pin, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(Start_enable_relay_GPIO_Port,Start_enable_relay_Pin, GPIO_PIN_RESET);
    }
    //========================================================================//
     
    //-------------------reverse !enable --------------------------------//
    if (start==0)
    {
      PWM=0;
      HAL_GPIO_WritePin(Reverse_enable_DO_relay_GPIO_Port,Reverse_enable_DO_relay_Pin, GPIO_PIN_RESET);
    }else{
      HAL_GPIO_WritePin(Reverse_enable_DO_relay_GPIO_Port,Reverse_enable_DO_relay_Pin, GPIO_PIN_SET);
    }
    //========================================================================//
    
    //-------------------stop command --------------------------------//
    if (start==3)  // start ==3 means stop command 
    {
      if (DC_stop_en)
      {
        PWM=0;
        osDelay(5);
        HAL_GPIO_WritePin(DC_on_DO_relay_GPIO_Port,DC_on_DO_relay_Pin, GPIO_PIN_SET);
        if (time_from_interrupt>Stop_time)//wait for a stop time over
        {
          start=0;        
        }
      }else{
        if ((time_from_interrupt<Stop_time)&(Stop_time>0)) // speed down
          {
            PWM=1000-init_pwm*(time_from_interrupt/Stop_time);
            PWM=(PWM<init_pwm)?init_pwm:PWM;
          }else{
            PWM=0;
            start=0;
          }
        
      }
    }else{
      HAL_GPIO_WritePin(Start_enable_relay_GPIO_Port,Start_enable_relay_Pin, GPIO_PIN_RESET);
    }
    //========================================================================//

    TIM2->CCR1=PWM;
    TIM3->CCR1=PWM;
    TIM4->CCR1=PWM;
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(2);
  }
  /* USER CODE END main_func */
}

/* StopTimer01 function */
void StopTimer01(void *argument)
{
  /* USER CODE BEGIN StopTimer01 */

  /* USER CODE END StopTimer01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */





//---------------------------exti--------------------//

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    phase_a_work=1;
    time_from_interrupt++;
    Time_a=0;
    if (start>0){HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);}
}
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    phase_b_work=1;
    if (start>0){HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);}
    Time_a++;
    Time_b=Time_a;
}
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    phase_c_work=1;
    if (start>0){HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);}
    Time_a++;
    Time_c=Time_a;
}






/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
