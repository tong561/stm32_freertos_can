/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include "can.h"
#include "stdio.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId nTaskCANSendHandle;
osThreadId nTaskCANReceiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TaskCANSend(void const * argument);
void TaskCANReceive(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of nTaskCANSend */
  osThreadDef(nTaskCANSend, TaskCANSend, osPriorityLow, 0, 128);
  nTaskCANSendHandle = osThreadCreate(osThread(nTaskCANSend), NULL);

  /* definition and creation of nTaskCANReceive */
  osThreadDef(nTaskCANReceive, TaskCANReceive, osPriorityBelowNormal, 0, 256);
  nTaskCANReceiveHandle = osThreadCreate(osThread(nTaskCANReceive), (void*) &TaskCANReceiveParam);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TaskCANSend */
/**
* @brief Function implementing the nTaskCANSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskCANSend */
void TaskCANSend(void const * argument)
{
  /* USER CODE BEGIN TaskCANSend */
  /* Infinite loop */
  for(;;)
  {
     // 在这里添加 CAN 发送代码，例如：
    uint8_t sendData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
		if(My_CAN_Send_Data(&hcan1, 0x7f1, sendData, 8)) // 调用你自己的 CAN 发送函数
		{
			printf("发送失败%d",1);
		}
		GPIOB->ODR|=0x04;
    osDelay(500); // 延时 1 秒，控制发送频率
		GPIOB->ODR&=~(0x04);
		osDelay(500); // 延时 1 秒，控制发送频率
  }
  /* USER CODE END TaskCANSend */
}

/* USER CODE BEGIN Header_TaskCANReceive */
/**
* @brief Function implementing the nTaskCANReceive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskCANReceive */
void TaskCANReceive(void const * argument)
{
  /* USER CODE BEGIN TaskCANReceive */
  TaskCANReceive_t *can_r_param = (TaskCANReceive_t*)argument;
  static CanRxFrame_t RxData;
  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(can_r_param->Semaphore, portMAX_DELAY) == pdPASS)
    {
      xMessageBufferReceive(
          can_r_param->MsgBuf,
          &RxData,
          sizeof(CanRxFrame_t),
          pdMS_TO_TICKS(10)
        );
        printf("CAN Rx: ID = 0x%X, Len = %d\n", RxData.ID, RxData.Length);
    }
    osDelay(1);
  }
  /* USER CODE END TaskCANReceive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
