/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
#include "tim.h"
#include "usart.h"
#include "inv_mpu.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
uint8_t brightness = 100;
/* USER CODE END Variables */
/* Definitions for mpu6050Task */
osThreadId_t mpu6050TaskHandle;
const osThreadAttr_t mpu6050Task_attributes = {
  .name = "mpu6050Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pwmTask */
osThreadId_t pwmTaskHandle;
const osThreadAttr_t pwmTask_attributes = {
  .name = "pwmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for brightTask */
osThreadId_t brightTaskHandle;
const osThreadAttr_t brightTask_attributes = {
  .name = "brightTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for period_Queue */
osMessageQueueId_t period_QueueHandle;
const osMessageQueueAttr_t period_Queue_attributes = {
  .name = "period_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MPU6050Task(void *argument);
void PWMTask(void *argument);
void USARTTask(void *argument);
void BRIGHTTask(void *argument);

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
  /* creation of period_Queue */
  period_QueueHandle = osMessageQueueNew (10, sizeof(uint8_t), &period_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mpu6050Task */
  mpu6050TaskHandle = osThreadNew(MPU6050Task, NULL, &mpu6050Task_attributes);

  /* creation of pwmTask */
  pwmTaskHandle = osThreadNew(PWMTask, NULL, &pwmTask_attributes);

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(USARTTask, NULL, &usartTask_attributes);

  /* creation of brightTask */
  brightTaskHandle = osThreadNew(BRIGHTTask, NULL, &brightTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MPU6050Task */
/**
 * @brief Function implementing the mpu6050Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MPU6050Task */
void MPU6050Task(void *argument)
{
  /* USER CODE BEGIN MPU6050Task */
  char message[50];
  int ret = 0;
  do
  {
    ret = MPU6050_DMP_init();
  } while (ret);
  float roll, yaw, pitch;
  /* Infinite loop */
  for (;;)
  {
    while (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw) != 0)
    {
      osDelay(10);
    }
    sprintf(message, "roll: %.1f, pitch: %.1f, yaw: %.1f\r\n", roll, pitch, yaw);
    HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    osDelay(1000);
  }
  /* USER CODE END MPU6050Task */
}

/* USER CODE BEGIN Header_PWMTask */
/**
 * @brief Function implementing the pwmTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PWMTask */
void PWMTask(void *argument)
{
  /* USER CODE BEGIN PWMTask */
  /* Infinite loop */
  for (;;)
  {	
	  
    for (int i = 0; i < brightness; i++)
    {
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, i);
      osDelay(10);
    }
    for (int i = brightness; i >= 0; i--)
    {
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, i);
      osDelay(10);
    }
  }
  /* USER CODE END PWMTask */
}

/* USER CODE BEGIN Header_USARTTask */
/**
 * @brief Function implementing the usartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_USARTTask */
void USARTTask(void *argument)
{
  /* USER CODE BEGIN USARTTask */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);               // 使能IDLE中断
//	uint8_t tmp;
	char str[50];
//	BaseType_t err;

  /* Infinite loop */
  for (;;)
  {
	  
//	  if(recv_len)
//	  {
//		  sprintf(str,"Brightness is %d\r\n",brightness);
//		  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
//			recv_len = 0;
//	  }
	  
    osDelay(10);
  }
  /* USER CODE END USARTTask */
}

/* USER CODE BEGIN Header_BRIGHTTask */
/**
* @brief Function implementing the brightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BRIGHTTask */
void BRIGHTTask(void *argument)
{
  /* USER CODE BEGIN BRIGHTTask */
	BaseType_t err;
	uint8_t tmp = 0;;
	char str[50];
//	err = xQueueReceive(period_QueueHandle,&tmp,10);
//	 if(err == pdTRUE) {
//		 brightness = tmp;
//	 }
  /* Infinite loop */
  
	 for(;;)
  { 
	  err = xQueueReceive(period_QueueHandle,&tmp,10);
		if(err == pdTRUE)
	  {
		  brightness = tmp;
		  sprintf(str,"Receive:%d\r\n",brightness);
		  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	  }
    osDelay(10);
  }
  /* USER CODE END BRIGHTTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// USART1串口空闲中断处理函数
void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET &&
      huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
	HAL_UART_DMAStop(huart);
    recv_len = MAX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (recv_len > 0 && recv_len < MAX_BUFFER_LEN - 1)
    {
		memcpy(RxData, DMA_Buffer, recv_len);
		RxData[recv_len] = '\0';
		uint8_t res = (uint8_t)atoi(RxData);
		xQueueSendFromISR(period_QueueHandle, &res, NULL);
//		xQueuePeek(period_QueueHandle,&res,portMAX_DELAY);
//		xQueueSend(period_QueueHandle, &res, portMAX_DELAY);
//		brightness = res;
    }
    else
    {
      recv_len = 0;
    }
    __HAL_UNLOCK(huart);
    HAL_UART_Receive_DMA(huart, DMA_Buffer, MAX_BUFFER_LEN);
  }
}
/* USER CODE END Application */

