/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi_slave.h
  * @brief   This file contains all the function prototypes for
  *          the spi_slave.c file
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
#ifndef __SPI_SLAVE_H__
#define __SPI_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define SPI_PACKET_SIZE 68
#define SPI_HEADER 0xAA
#define SPI_FOOTER 0xBB
//接收/发送缓冲区的长度 (qr_wl spi2canV3 协议使用 8-bit 字节)
// 最大帧长度计算:
//   12 电机 (6+6): 3 + 120 + 1(padding) + 4 + 1 = 129 bytes
//   为了安全留有余量，设置为 256 字节
#define RX_LEN 256
#define TX_LEN 256

//传出/传入消息的长度 length of outgoing/incoming messages
#define DATA_LEN 42
#define CMD_LEN 66
// Master CAN ID /// 主设备 canID
#define CAN_ID 0x0
// qr_wl spi2canV3 完整帧结构
// 帧格式: [0xAA][can1_num][can2_num][CAN1 data][CAN2 data][checksum 4B][0xBB]
#define SPI_FRAME_START 0xAA
#define SPI_FRAME_END 0xBB
#define NumberOfMotors 6     //电机总数量





// ============================================
// 动态帧长度计算宏（包含 4 字节对齐）
// ============================================
// qr_wl 协议要求 data 部分（header + CAN data）必须 4 字节对齐
//
// 帧结构:
//   [0xAA][can1_num][can2_num][CAN1 data...][CAN2 data...][padding 0-3B][checksum 4B][0xBB]
//
// 长度计算:
//   1. data_len = 3(header) + 10*can1_num + 10*can2_num
//   2. aligned_len = ((data_len + 3) / 4) * 4  // 向上对齐到 4 字节边界
//   3. total_len = aligned_len + 4(checksum) + 1(end)
//
// 示例（6 电机, 3+3）:
//   data_len = 3 + 30 + 30 = 63
//   aligned_len = ((63 + 3) / 4) * 4 = 64 (padding 1 字节)
//   total_len = 64 + 4 + 1 = 69 字节
// Data 部分长度（不包含 padding）
#define SPI_FRAME_DATA_LEN(can1_num, can2_num) \
    (3 + 10*(can1_num) + 10*(can2_num))

// Data 部分对齐后长度（包含 padding）
#define SPI_FRAME_ALIGNED_LEN(can1_num, can2_num) \
    (((SPI_FRAME_DATA_LEN(can1_num, can2_num) + 3) / 4) * 4)

// 完整帧长度（包含 checksum 和 end）
#define SPI_FRAME_LEN(can1_num, can2_num) \
    (SPI_FRAME_ALIGNED_LEN(can1_num, can2_num) + 5)




// SPI状态枚举
typedef enum {
    SPI_STATE_IDLE,
    SPI_STATE_RECEIVING,
    SPI_STATE_TRANSMITTING,
    SPI_STATE_COMPLETE
} SPI_State_t;

// SPI句柄结构体
typedef struct {
    uint8_t rx_buffer[SPI_PACKET_SIZE];
    uint8_t tx_buffer[SPI_PACKET_SIZE];
    osMessageQueueId_t rx_queue;
    osMessageQueueId_t tx_queue;
    osEventFlagsId_t spi_event;
    volatile SPI_State_t spi_state;
    volatile uint8_t packet_received;
    volatile uint8_t packet_to_send;
} SPI_Slave_Handle_t;


/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

// 函数声明
void SPI_Slave_Init(void);
void SPI_Slave_Start_Transfer(void);
void SPI_Slave_Handle_Transfer(void);
SPI_Slave_Handle_t* Get_SPI_Slave_Handle(void);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// qr_wl spi2canV3 协议校验和计算
uint32_t xor_checksum_qrwl(uint8_t* data, size_t len);
//验证校验码
void VerificationCheckCode();
//构建 SPI 返回帧 (电机反馈数据)
void build_spi_response();
//生成数据（测试）
void CAN_GeneratedData();


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__SPI_SLAVE_H__ */

