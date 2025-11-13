/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
static StaticMessageBuffer_t sCanMsgBufCtrl;          // 控制块（不能为 NULL）
static uint8_t canMsgStorage[240] __attribute__((aligned(4)));;  // 静态数组，内存分配在全局/栈（取决于声明位置）
TaskCANReceive_t TaskCANReceiveParam;//定义两个任务接收can消息的结构体数组


/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void My_CAN1_Init(void)//初始化CAN1模块，并启动中断
{
	HAL_CAN_Start(&hcan1);//启动CAN1模块
	 HAL_CAN_ActivateNotification(&hcan1,  CAN_IT_RX_FIFO0_MSG_PENDING);
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//开启CAN1的接收FIFO0报文挂起中断
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);//开启CAN1的接收FIFO1报文挂起中断
}
/*发送CAN数据帧
参数：hcan:CAN句柄
      ID:CAN ID
      Data:CAN数据
      Length:CAN数据长度
      返回值：0表示成功，非0表示失败
*/

unsigned char My_CAN_Send_Data(CAN_HandleTypeDef *hcan,uint16_t ID,unsigned char *Data,uint8_t Length)
{
	CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox = 0;
  assert_param(hcan != NULL);//断言参数不为空
  if(Length>8U) return 9;//数据长度超过8字节，返回错误
  else if(Length<1U) return 10;//数据长度小于1字节，返回错误
  //配置发送头信息

  TxHeader.StdId=ID;//标准帧ID，此处为实际发送的CAN ID
  TxHeader.ExtId=0x00;//扩展帧ID，此处不使用扩展帧ID，设置为0x00即可
  TxHeader.IDE=CAN_ID_STD;//标准帧ID
  TxHeader.RTR=CAN_RTR_DATA;//数据帧类型，此处为数据帧
  TxHeader.DLC=Length;//数据长度，此处为实际发送的数据长度
  HAL_StatusTypeDef status=HAL_CAN_AddTxMessage(hcan,&TxHeader,Data,&TxMailbox);//发送数据帧，并获取邮箱号
  return (status==HAL_OK)?0:3;
}
/*
@brief  配置CAN1的过滤器（支持标准帧11位/扩展帧29位），使用32位屏蔽模式
@param  hcan: CAN1句柄（&hcan1）
@param  Object_Para: 复合参数 = 过滤器编号（低7位） | 帧格式（高1位：0x80=扩展帧，0x00=标准帧）示例：0x00=过滤器0+标准帧；0x81=过滤器1+扩展帧
@param  FiFOx: 接收FIFO分配（CAN_FILTER_FIFO0 或 CAN_FILTER_FIFO1）
@param  ID: 过滤器基准ID（标准帧：0x000~0x7FF；扩展帧：0x00000000~0x1FFFFFFF）
@param  Mask_ID: 过滤器掩码ID（标准帧：0x000~0x7FF；扩展帧：0x00000000~0x1FFFFFFF）
@note   过滤规则：(接收报文ID & Mask_ID) == (配置ID & Mask_ID) 时接收
        示例1（标准帧：匹配0x120~0x12F）：Object_Para=0x00，ID=0x123，Mask_ID=0x7F0
        示例2（扩展帧：精准匹配0x12345678）：Object_Para=0x80，ID=0x12345678，Mask_ID=0x1FFFFFFF
*/
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, 
                            uint32_t FiFOx, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure={0};
    uint8_t filter_bank = Object_Para & 0x7F;  // 剥离低7位：过滤器编号（0~127，实际CAN1仅0~13有效）
    uint8_t is_extended = (Object_Para & 0x80) ? 1 : 0; // 提取高1位：是否为扩展帧

    // 1. 严格的参数合法性断言（避免配置错误）
    assert_param(hcan != NULL);                                  // 句柄非空
    assert_param((is_extended == 0) || (is_extended == 1));      // 帧格式合法
    assert_param((is_extended == 0 && ID <= 0x7FF && Mask_ID <= 0x7FF) ||  // 标准帧ID/掩码范围
                 (is_extended == 1 && ID <= 0x1FFFFFFF && Mask_ID <= 0x1FFFFFFF)); // 扩展帧范围

    // 2. 过滤器核心配置
    can_filter_init_structure.FilterBank = filter_bank;          // 实际过滤器编号（0~13）
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器尺度
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码匹配模式
    can_filter_init_structure.FilterFIFOAssignment = FiFOx;      // 匹配报文存入指定FIFO
    can_filter_init_structure.FilterActivation = ENABLE;         // 使能当前过滤器组
    // 3. 关键：根据帧格式做ID/掩码位对齐
    uint32_t aligned_ID = 0;
    uint32_t aligned_Mask = 0;
    if (is_extended == 0)  // 标准帧（11位ID）：左移3位对齐（跳过IDE+RTR+保留位）
    {
        aligned_ID = ID << 5;
        aligned_Mask = Mask_ID << 5;
    }
    else  // 扩展帧（29位ID）：无需左移（直接占满32位寄存器）
    {
        aligned_ID = ID;
        aligned_Mask = Mask_ID;
    }

    // 4. 拆分32位对齐后的ID和掩码到高/低16位寄存器
    can_filter_init_structure.FilterIdHigh = aligned_ID >> 16& 0xFFFF;    // 高16位
    can_filter_init_structure.FilterIdLow = aligned_ID       & 0xFFFF;  // 低16位
    can_filter_init_structure.FilterMaskIdHigh = aligned_Mask >> 16& 0xFFFF; // 掩码高16位
    can_filter_init_structure.FilterMaskIdLow = aligned_Mask       & 0xFFFF; // 掩码低16位
		
    // 5. 应用配置并检查结果
		
    if (HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure) != HAL_OK)
    {
        Error_Handler();  // 配置失败进入错误处理（需自行实现）
    }
}
void CAN_FreeRTOS_Init(void)
{
  TaskCANReceiveParam.MsgBuf = xMessageBufferCreateStatic
              (sizeof(canMsgStorage), // 参数1：消息缓冲区总容量（字节）= 数组大小
              canMsgStorage,// 参数2：指向静态存储数组的指针，用于存放消息缓冲区数据
              &sCanMsgBufCtrl// 参数3：指向静态控制块（用于存放消息缓冲区状态信息）的指针
              );//初始化消息缓冲区，用于存储CAN数据
  configASSERT(TaskCANReceiveParam.MsgBuf);//断言消息缓冲区是否初始化成功
  // 2. 创建二值信号量（初始值为 0，中断中释放）
  TaskCANReceiveParam.Semaphore = xSemaphoreCreateBinary();
  configASSERT(TaskCANReceiveParam.Semaphore); // 断言信号量创建成功
							

}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t pxHigherPriorityTaskWoken=pdFALSE;//定义一个标志位，用于判断是否有更高优先级的任务被唤醒

    CAN_RxHeaderTypeDef RxHeader;//定义接收头信息结构体
    CanRxFrame_t CanRxFrame;//定义接收帧结构体
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxFrame.Data) == HAL_OK)
    {
        CanRxFrame.ID=RxHeader.StdId;//将接收到的标准帧ID赋值给结构体中的ID成员
        CanRxFrame.Length=RxHeader.DLC;//将接收到的数据长度赋值给结构体中的Length成员
         xMessageBufferSendFromISR
         (
          TaskCANReceiveParam.MsgBuf, // 参数1：消息缓冲区句柄
          &CanRxFrame, // 参数2：指向要发送数据的指针
          sizeof(CanRxFrame_t), //参数3：要发送的数据大小（字节）
          &pxHigherPriorityTaskWoken//参数4：指向标志位的指针，用于判断是否有更高优先级的任务被唤醒
        );//将接收到的数据发送到消息缓冲区，并更新标志位
        xSemaphoreGiveFromISR(TaskCANReceiveParam.Semaphore, &pxHigherPriorityTaskWoken);//释放信号量，并更新标志位
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);//如果有更高优先级的任务被唤醒，则执行上下文切换

    }
        
}
/* USER CODE END 1 */
