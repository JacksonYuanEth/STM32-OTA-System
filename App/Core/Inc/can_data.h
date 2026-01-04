/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_data.h
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
#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"  // 包含CMSIS-RTOS2头文件
#include "gpio.h"
#include "can.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
// 最大支持 6 个电机 (每个 CAN 总线 3 个电机)
#define MAX_CAN_MOTORS 6
// 单腿最大支持 3 个电机 (每个 CAN 总线 3 个电机)
#define OneLegMAX_CAN_MOTORS 3
//电机ID号定义
//CAN1的电机ID号
#define CAN1Motor1ID       0x01    //CAN1电机1的ID
#define CAN1Motor2ID       0x02    //CAN1电机2的ID
#define CAN1Motor3ID       0x03    //CAN1电机3的ID
//CAN1的电机ID号
#define CAN2Motor1ID       0x01    //CAN2电机1的ID
#define CAN2Motor2ID       0x02    //CAN2电机2的ID
#define CAN2Motor3ID       0x03    //CAN2电机3的ID


//使用DigitalIn接口读取数字输入引脚的值。 逻辑电平为1或0。
#define DigitalIn          HAL_GPIO_ReadPin(MCU_ESTOP_GPIO_Port, MCU_ESTOP_Pin); //estop(PB_15);  //软停止按键？



/* 自定义CAN消息结构体 */
typedef struct {
    uint32_t stdId;     // 标准ID
    uint32_t extId;     // 扩展ID
    uint8_t  ide;       // IDE位：0=标准帧，1=扩展帧
    uint8_t  rtr;       // RTR位：0=数据帧，1=远程帧
    uint8_t  dlc;       // 数据长度(0-8)
    uint8_t  data[8];   // 数据
    uint32_t timestamp; // 时间戳
} CAN_Message_t;


typedef struct
{
	uint32_t StdId;     //指定标准标识符。此参数必须是介于最小值 Min_Data = 0 和最大值 Max_Data = 0x7FF 之间的数字。
	uint32_t ExtId;     //指定扩展标识符。此参数必须是一个介于 Min_Data = 0 和 Max_Data = 0x1FFFFFFF 之间的数值。
	uint32_t IDE;	    //指定将要传输的消息的标识符类型。此参数可以是 @ref CAN_identifier_type 的某个值。
	uint32_t RTR;	    //指定将要传输的消息的帧类型。此参数可以是 @ref CAN_remote_transmission_request 的某个值。
	uint32_t DLC;	    //指定将要传输的帧的长度。此参数必须介于最小数据值 Min_Data = 0 和最大数据值 Max_Data = 8 之间。
	FunctionalState TransmitGlobalTime;     //指定在帧传输开始时捕获的时间戳计数值是否会在 DATA6 和 DATA7 中发送，以取代 pData[6] 和 pData[7]。
											//@注意：必须启用时间触发通信模式。
											//@注意：数据长度（DLC）必须设置为 8 个字节，以便这 2 个字节能够被发送。
											//此参数可设置为 ENABLE 或 DISABLE。
	uint8_t Data[8];    //包含要传输的数据。其范围为 0 到 0xFF 。
} CAN_TxMessage_t;

// 单个 CAN 消息结构 (10 bytes)
typedef struct 
{
    uint16_t id;      // 2 bytes: CAN ID (低字节在前)
    uint8_t data[8];  // 8 bytes: CAN 数据
}CanData_t;


/* 外部变量声明 */
extern osMessageQueueId_t can1QueueHandle;
extern osMessageQueueId_t can2QueueHandle;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
/* 函数声明 */
void CAN_Data_Init(void);
void CAN_Process_Rx_Message(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
void AND_CanSendServe(CAN_HandleTypeDef hcan,CAN_TxMessage_t CAN_TxMessa);//CAN发送
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);//CAN接收
void CANcontrol();    //qr_wl spi2canV3 协议控制函数 (CAN 消息透传) (STM32->CAN->电机)
void CANSend_DataConversion();//CAN发送给电机的数据转换
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DATA_H__ */

