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

/* USER CODE BEGIN Private defines */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 *
 * \enum    CANFormat
 *
 * \brief   Values that represent CAN Format
**/
enum CANFormat {
    CANStandard = 0,
    CANExtended = 1,
    CANAny = 2
};
typedef enum CANFormat CANFormat;

/**
 *
 * \enum    CANType
 *
 * \brief   Values that represent CAN Type
**/
enum CANType {
    CANData   = 0,
    CANRemote = 1
};
typedef enum CANType CANType;

/**
 *
 * \struct  CAN_Message
 *
 * \brief   Holder for single CAN message.
 *
**/
struct CAN_Message {
    unsigned int   id;                 // 29 bit identifier
    unsigned char  data[8];            // Data field
    unsigned char  len;                // Length of data field in bytes
    CANFormat      format;             // Format ::CANFormat
    CANType        type;               // Type ::CANType
};
typedef struct CAN_Message CAN_Message;


//CAN1读取到的数据：
typedef struct{
    unsigned char CAN1_DATA1[10];  //1号电机的数据（包含ID号）
	unsigned char CAN1_DATA2[10];  //2号电机的数据（包含ID号）
	unsigned char CAN1_DATA3[10];  //3号电机的数据（包含ID号）
}CAN1_DATA_Rx;
//CAN2读取到的数据：
typedef struct{
    unsigned char CAN2_DATA1[10];  //1号电机的数据（包含ID号）
	unsigned char CAN2_DATA2[10];  //2号电机的数据（包含ID号）
	unsigned char CAN2_DATA3[10];  //3号电机的数据（包含ID号）
}CAN2_DATA_Rx;




/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

