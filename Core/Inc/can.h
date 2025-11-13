/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
/************************************结构体*********************************************/
typedef struct {
  MessageBufferHandle_t MsgBuf;//can消息缓冲区句柄声明
  SemaphoreHandle_t Semaphore;  // 二值信号量句柄（替代任务通知）
}TaskCANReceive_t;//任务接收CAN消息的结构体声明
typedef struct 
{
  uint32_t ID; //CAN消息ID
  uint8_t Length;//CAN数据长度
  uint8_t Data[10];//CAN数据数组,留出空间防止溢出。
}CanRxFrame_t ;//CAN接收帧的结构体声明

/*****************************外部变量***************************************************/
extern TaskCANReceive_t TaskCANReceiveParam;//定义两个任务接收can消息的结构体数组



/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void My_CAN1_Init(void);//我的CAN初始化函数
unsigned char My_CAN_Send_Data(CAN_HandleTypeDef *hcan,uint16_t ID,unsigned char *Data,uint8_t Length);//CAN发送函数
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, 
                            uint32_t FiFOx, uint32_t ID, uint32_t Mask_ID);//我的CAN滤波器配置函数

void CAN_FreeRTOS_Init(void); //CAN接收任务初始化函数


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

