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

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
uint8_t ErrorCode[10] = {0};


/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef can_filter1;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
   /* 配置CAN1过滤器 - 注意：CAN1使用过滤器0~13 CAN2使用过滤器组14-27 */
  can_filter1.FilterIdHigh = 0x0000;
  can_filter1.FilterIdLow = 0x0000;
  can_filter1.FilterMaskIdHigh = 0x0000;
  can_filter1.FilterMaskIdLow = 0x0000;
  can_filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;//CAN_RX_FIFO0;
  can_filter1.FilterBank = 0;                  // CAN2使用过滤器组14开始
  //can_filter1.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter1.FilterMode = CAN_FILTERMODE_IDMASK;//CAN_FILTERMODE_IDLIST;  // 改为列表模式，或者保持掩码模式
  can_filter1.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter1.FilterActivation = ENABLE;        //激活滤波器 0
  can_filter1.SlaveStartFilterBank = 14;        //can1(0-13) can2(14-27) 对于CAN1，这个参数定义CAN2的开始
  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
   // 检查CAN是否在正常模式
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		ErrorCode[0] = 0xF0;  // 启动失败
	}else{
		ErrorCode[0] = 0x00;  // 成功
	}

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	// 检查CAN1状态
	if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY)
	{
		ErrorCode[0] = 0x00;   // 正常
	}else{
		ErrorCode[0] = 0x0F;   // 故障
	}
	
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
  CAN_FilterTypeDef can_filter2;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  //hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  //hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;        
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;// 时间段1
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;// 时间段2
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 配置CAN2过滤器 - 注意：CAN1使用过滤器0~13 CAN2使用过滤器组14-27 */
  can_filter2.FilterBank = 14;                  // CAN2使用过滤器组14开始
  can_filter2.FilterMode = CAN_FILTERMODE_IDMASK;//CAN_FILTERMODE_IDLIST;  // 改为列表模式，或者保持掩码模式
  can_filter2.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter2.FilterIdHigh = 0x0000;
  can_filter2.FilterIdLow = 0x0000;
  can_filter2.FilterMaskIdHigh = 0x0000;
  can_filter2.FilterMaskIdLow = 0x0000;
  //can_filter2.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter2.FilterFIFOAssignment = CAN_FILTER_FIFO0;//CAN_RX_FIFO0;
  can_filter2.FilterActivation = ENABLE;//激活滤波器 0
  can_filter2.SlaveStartFilterBank = 14;        // can1(0-13) can2(14-27) 对于CAN1，这个参数定义CAN2的开始
  
  if (HAL_CAN_ConfigFilter(&hcan2, &can_filter2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN2_Init 2 */
  // 检查CAN是否在正常模式
	if(HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		ErrorCode[1] = 0xF0;//printf("CAN2启动失败\n");
	}else{
	    ErrorCode[1] = 0x00;
	}
	
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);// 激活中断
	// 检查CAN2状态
	if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
	{
		ErrorCode[1] = 0x00;    //CAN2 初始化成功
	}
	else
	{
		ErrorCode[1] |= 0x0F;    //CAN2 初始化失败
	}
	
  /* USER CODE END CAN2_Init 2 */

}


void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    __HAL_RCC_CAN1_CLK_ENABLE();
	  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 10, 1);    //后加
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
//    __HAL_RCC_CAN2_CLK_ENABLE();
//    HAL_RCC_CAN1_CLK_ENABLED++;
//    if(HAL_RCC_CAN1_CLK_ENABLED==1){
//      __HAL_RCC_CAN1_CLK_ENABLE();
//    }
	__HAL_RCC_CAN2_CLK_ENABLE();
    /* CAN2需要CAN1时钟作为基础 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 11, 1);    //后加
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
    if(canHandle->Instance==CAN1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();  // 改为 DISABLE
        
        /**CAN1 GPIO Configuration */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
        
        /* CAN1 interrupt Deinit */
        //HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    }
    else if(canHandle->Instance==CAN2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_CAN2_CLK_DISABLE();  // 改为 DISABLE
        
        /**CAN2 GPIO Configuration */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);
        
        /* CAN2 interrupt Deinit */
        //HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    }
}
/* USER CODE BEGIN 1 */












/* USER CODE END 1 */
