/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "User_Task.h"
#include "stm32f4xx_it.h"
#include "UART.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RUN_LED_Flash */
osThreadId_t RUN_LED_FlashHandle;
const osThreadAttr_t RUN_LED_Flash_attributes = {
  .name = "RUN_LED_Flash",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for State_LED_Flash */
osThreadId_t State_LED_FlashHandle;
const osThreadAttr_t State_LED_Flash_attributes = {
  .name = "State_LED_Flash",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KEY_Scan */
osThreadId_t KEY_ScanHandle;
const osThreadAttr_t KEY_Scan_attributes = {
  .name = "KEY_Scan",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Set_And_Show */
osThreadId_t Set_And_ShowHandle;
const osThreadAttr_t Set_And_Show_attributes = {
  .name = "Set_And_Show",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Slow_Compute */
osThreadId_t Slow_ComputeHandle;
const osThreadAttr_t Slow_Compute_attributes = {
  .name = "Slow_Compute",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UART_Debug */
osThreadId_t UART_DebugHandle;
const osThreadAttr_t UART_Debug_attributes = {
  .name = "UART_Debug",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for High_Compute */
osThreadId_t High_ComputeHandle;
const osThreadAttr_t High_Compute_attributes = {
  .name = "High_Compute",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GPIO_IT_Trigger */
osThreadId_t GPIO_IT_TriggerHandle;
const osThreadAttr_t GPIO_IT_Trigger_attributes = {
  .name = "GPIO_IT_Trigger",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RUN_Control */
osThreadId_t RUN_ControlHandle;
const osThreadAttr_t RUN_Control_attributes = {
  .name = "RUN_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Visual_Identity */
osThreadId_t Visual_IdentityHandle;
const osThreadAttr_t Visual_Identity_attributes = {
  .name = "Visual_Identity",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for GPIO_Tigger_State */
osMessageQueueId_t GPIO_Tigger_StateHandle;
uint8_t GPIO_Tigger_Buffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t GPIO_Tigger_Buffer_ControlBlock;
const osMessageQueueAttr_t GPIO_Tigger_State_attributes = {
  .name = "GPIO_Tigger_State",
  .cb_mem = &GPIO_Tigger_Buffer_ControlBlock,
  .cb_size = sizeof(GPIO_Tigger_Buffer_ControlBlock),
  .mq_mem = &GPIO_Tigger_Buffer,
  .mq_size = sizeof(GPIO_Tigger_Buffer)
};
/* Definitions for Visual_Identity_Queue */
osMessageQueueId_t Visual_Identity_QueueHandle;
uint8_t Visual_Identity_QueueBuffer[ 4 * 66 ];
osStaticMessageQDef_t Visual_Identity_QueueControlBlock;
const osMessageQueueAttr_t Visual_Identity_Queue_attributes = {
  .name = "Visual_Identity_Queue",
  .cb_mem = &Visual_Identity_QueueControlBlock,
  .cb_size = sizeof(Visual_Identity_QueueControlBlock),
  .mq_mem = &Visual_Identity_QueueBuffer,
  .mq_size = sizeof(Visual_Identity_QueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RUN_LED_Flash_Task(void *argument);
void State_LED_Flash_Task(void *argument);
void KEY_Scan_Task(void *argument);
void Set_And_Show_Task(void *argument);
void Slow_Compute_Task(void *argument);
void UART_Debug_Task(void *argument);
void High_Compute_Task(void *argument);
void GPIO_IT_Trigger_Task(void *argument);
void RUN_Control_Task(void *argument);
void Visual_Identity_Task(void *argument);

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

  /* Create the queue(s) */
  /* creation of GPIO_Tigger_State */
  GPIO_Tigger_StateHandle = osMessageQueueNew (16, sizeof(uint32_t), &GPIO_Tigger_State_attributes);

  /* creation of Visual_Identity_Queue */
  Visual_Identity_QueueHandle = osMessageQueueNew (4, 66, &Visual_Identity_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RUN_LED_Flash */
  RUN_LED_FlashHandle = osThreadNew(RUN_LED_Flash_Task, NULL, &RUN_LED_Flash_attributes);

  /* creation of State_LED_Flash */
  State_LED_FlashHandle = osThreadNew(State_LED_Flash_Task, NULL, &State_LED_Flash_attributes);

  /* creation of KEY_Scan */
  KEY_ScanHandle = osThreadNew(KEY_Scan_Task, NULL, &KEY_Scan_attributes);

  /* creation of Set_And_Show */
  Set_And_ShowHandle = osThreadNew(Set_And_Show_Task, NULL, &Set_And_Show_attributes);

  /* creation of Slow_Compute */
  Slow_ComputeHandle = osThreadNew(Slow_Compute_Task, NULL, &Slow_Compute_attributes);

  /* creation of UART_Debug */
  UART_DebugHandle = osThreadNew(UART_Debug_Task, NULL, &UART_Debug_attributes);

  /* creation of High_Compute */
  High_ComputeHandle = osThreadNew(High_Compute_Task, NULL, &High_Compute_attributes);

  /* creation of GPIO_IT_Trigger */
  GPIO_IT_TriggerHandle = osThreadNew(GPIO_IT_Trigger_Task, NULL, &GPIO_IT_Trigger_attributes);

  /* creation of RUN_Control */
  RUN_ControlHandle = osThreadNew(RUN_Control_Task, NULL, &RUN_Control_attributes);

  /* creation of Visual_Identity */
  Visual_IdentityHandle = osThreadNew(Visual_Identity_Task, NULL, &Visual_Identity_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RUN_LED_Flash_Task */
/**
* @brief Function implementing the RUN_LED_Flash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RUN_LED_Flash_Task */
__weak void RUN_LED_Flash_Task(void *argument)
{
  /* USER CODE BEGIN RUN_LED_Flash_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RUN_LED_Flash_Task */
}

/* USER CODE BEGIN Header_State_LED_Flash_Task */
/**
* @brief Function implementing the State_LED_Flash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_State_LED_Flash_Task */
__weak void State_LED_Flash_Task(void *argument)
{
  /* USER CODE BEGIN State_LED_Flash_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END State_LED_Flash_Task */
}

/* USER CODE BEGIN Header_KEY_Scan_Task */
/**
* @brief Function implementing the KEY_Scan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KEY_Scan_Task */
__weak void KEY_Scan_Task(void *argument)
{
  /* USER CODE BEGIN KEY_Scan_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KEY_Scan_Task */
}

/* USER CODE BEGIN Header_Set_And_Show_Task */
/**
* @brief Function implementing the Set_And_Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Set_And_Show_Task */
__weak void Set_And_Show_Task(void *argument)
{
  /* USER CODE BEGIN Set_And_Show_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Set_And_Show_Task */
}

/* USER CODE BEGIN Header_Slow_Compute_Task */
/**
* @brief Function implementing the Slow_Compute thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Slow_Compute_Task */
__weak void Slow_Compute_Task(void *argument)
{
  /* USER CODE BEGIN Slow_Compute_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Slow_Compute_Task */
}

/* USER CODE BEGIN Header_UART_Debug_Task */
/**
* @brief Function implementing the UART_Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Debug_Task */
__weak void UART_Debug_Task(void *argument)
{
  /* USER CODE BEGIN UART_Debug_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART_Debug_Task */
}

/* USER CODE BEGIN Header_High_Compute_Task */
/**
* @brief Function implementing the High_Compute thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_High_Compute_Task */
__weak void High_Compute_Task(void *argument)
{
  /* USER CODE BEGIN High_Compute_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END High_Compute_Task */
}

/* USER CODE BEGIN Header_GPIO_IT_Trigger_Task */
/**
* @brief Function implementing the GPIO_IT_Trigger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPIO_IT_Trigger_Task */
__weak void GPIO_IT_Trigger_Task(void *argument)
{
  /* USER CODE BEGIN GPIO_IT_Trigger_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPIO_IT_Trigger_Task */
}

/* USER CODE BEGIN Header_RUN_Control_Task */
/**
* @brief Function implementing the RUN_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RUN_Control_Task */
__weak void RUN_Control_Task(void *argument)
{
  /* USER CODE BEGIN RUN_Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RUN_Control_Task */
}

/* USER CODE BEGIN Header_Visual_Identity_Task */
/**
* @brief Function implementing the Visual_Identity thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Visual_Identity_Task */
__weak void Visual_Identity_Task(void *argument)
{
  /* USER CODE BEGIN Visual_Identity_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Visual_Identity_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

