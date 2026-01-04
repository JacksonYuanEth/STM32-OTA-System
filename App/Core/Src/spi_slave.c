/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi_slave.c
  * @brief   This file provides code for the configuration
  *          of the spi_slave instances.
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
#include "spi_slave.h"
#include "main.h"
#include "spi.h"
#include "string.h"
#include "can.h"
#include "can_data.h"



/* USER CODE BEGIN 0 */
SPI_Slave_Handle_t spi_slave;
extern SPI_HandleTypeDef hspi1;
CAN1_DATA_Rx Can1_RxDATA;
CAN2_DATA_Rx Can2_RxDATA;

// spi buffers (修改为 8-bit 字节缓冲区，匹配 qr_wl spi2canV3 协议)
uint8_t rx_buff[RX_LEN];  //接收的SPI数据缓冲区 (8-bit 字节)
uint8_t tx_buff[RX_LEN];  //发送的SPI数据缓冲区 (8-bit 字节)

uint8_t can1_tx[30];
uint8_t can2_tx[30];

extern uint8_t can1_rx_count;         // CAN1 接收消息数量
extern uint8_t can2_rx_count;         // CAN2 接收消息数量

////////////////////////////////////////////////////////////////////////////////////////////
//电机的ID号：测试
uint16_t RxID_Number[NumberOfMotors] = {0};              //SPI读取到的电机ID编号
uint16_t TxID_Number[NumberOfMotors] = {1,2,3,4,5,6};    //SPI要发送的电机ID编号
//SPI发送协议：  (STM32 -> SPI -> 上位机)
// *     * Byte 0-1: 位置 (16-bit)
// *     * Byte 2-3: 速度 (12-bit)
// *     * Byte 4-5: 力矩/电流 (12-bit)
// *     * Byte 6-7: 补 0x00 (MIT 电机不使用)
//SPI接收协议    (上位机 ->SPI-> STM32)
// * MIT 协议命令格式 (8 bytes):
// *   - Byte 0-1: 位置 (16-bit)
// *   - Byte 2-3: 速度 (12-bit) + Kp (12-bit 高 4 位)
// *   - Byte 4: Kp (低 8 位)
// *   - Byte 5-6: Kd (12-bit) + 力矩 (12-bit 高 4 位)
// *   - Byte 7: 力矩 (低 8 位) 或特殊命令 (0xFC/0xFD/0xFE)
//位置：
uint16_t RxMotorPosition[NumberOfMotors] = {0x32,0x3c,0x46,0x50,0x5A,0x78};                                  //SPI读取到的6个电机位置
uint16_t TxMotorPosition[NumberOfMotors] = {0x32,0x3c,0x46,0x50,0x5A,0x78};      //SPI要发送的6个电机位置
//速度：
uint16_t RxMotorSpeed[NumberOfMotors] = {0x14,0x19,0x1e,0x23,0x28,0x32};                                     //SPI读取到的6个电机的速度+Kp高4位
uint16_t TxMotorSpeed[NumberOfMotors] = {0x14,0x19,0x1e,0x23,0x28,0x32};         //SPI要发送的6个电机的速度
//Kp：
uint16_t RxMotorKp[NumberOfMotors] = {0xfe,0xee,0xaa,0xbb,0xcc,0xfe};                                        //SPI读取到的6个电机的Kp
//Kd
uint16_t RxMotorKd[NumberOfMotors] = {0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};                                        //SPI读取到的6个电机的Kd
//力矩:
uint16_t RxMotorTorque[NumberOfMotors] = {0x1e,0x20,0x1d,0x1f,0x23,0x1c};                                    //SPI读取到的6个电机的力矩/电流
uint16_t TxMotorTorque[NumberOfMotors] = {0x1e,0x20,0x1d,0x1f,0x23,0x1c};        //SPI要发送的6个电机的力矩/电流
//其他
uint16_t RxMotorTorqueL[NumberOfMotors] = {0};                                   //SPI读取到的6个电机的力矩低
uint16_t TxMotorOffsetValue[NumberOfMotors] = {0};                               //SPI要发送的6个电机的补偿值
//////////////////////////////////////////////////////////////////////////////////////////////




//计算预期帧长度
//uint32_t expected_len = SPI_FRAME_LEN(can1_rx_count, can2_rx_count)-1;
/* USER CODE END 0 */



/* SPI slave init function */

// 获取SPI句柄
SPI_Slave_Handle_t* Get_SPI_Slave_Handle(void)
{
    return &spi_slave;
}

// NSS引脚外部中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_4) // SPI1_NSS
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            // NSS拉低，开始传输
            SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
            if(spi->spi_state == SPI_STATE_IDLE) //&& spi->packet_to_send)     //&& spi->packet_to_send为后加的
            {
                spi->spi_state = SPI_STATE_TRANSMITTING;
                //SPI_Slave_Start_Transfer();     // 开始SPI传输
            }
        }
        else
        {
            // NSS拉高，传输结束
            SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
            if(spi->spi_state == SPI_STATE_TRANSMITTING)
            {
                spi->spi_state = SPI_STATE_IDLE;
            }
        }
    }
}

// SPI传输完成回调（被两个函数调用：SPI_CloseRxTx_ISR() 、 SPI_DMATransmitReceiveCplt()）
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance != SPI1) return;

    SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();

    /* 处理刚收到的帧 */
    if(spi->rx_buffer[0] == SPI_HEADER &&
       spi->rx_buffer[SPI_PACKET_SIZE-1] == SPI_FOOTER)
    {
        if(spi->rx_queue)
            osMessageQueuePut(spi->rx_queue, spi->rx_buffer, 0, 0);
    }

    /* 原地更新下一帧要发的数据 */
    static uint8_t cnt = 0;
    cnt++;
    spi->tx_buffer[0]               = SPI_HEADER;
    spi->tx_buffer[SPI_PACKET_SIZE-1] = SPI_FOOTER;
    //for(uint8_t i = 1; i < SPI_PACKET_SIZE-1; i++)
        //spi->tx_buffer[i] = cnt + i;

    /* 不需要重新启动 DMA，CIRCULAR 会自动继续 */
}

// SPI错误回调
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1)
    {
        SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
        spi->spi_state = SPI_STATE_IDLE;
        spi->packet_received = 0;
        
        // 重新初始化SPI
        HAL_SPI_DeInit(hspi);
        MX_SPI1_Init();
    }
}

// SPI从机初始化
void SPI_Slave_Init(void)
{
    SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
    memset(spi, 0, sizeof(*spi));

    /* 创建队列/事件（保持原样） */
    spi->rx_queue = osMessageQueueNew(4, SPI_PACKET_SIZE, NULL);
    spi->tx_queue = osMessageQueueNew(4, SPI_PACKET_SIZE, NULL);
    spi->spi_event = osEventFlagsNew(NULL);

    /* 把默认帧填好（测试数据已经在任务函数中生成了，初始化中可以不需要生成数据） */
    //for(uint8_t i = 0; i < SPI_PACKET_SIZE; i++)
    //    spi->tx_buffer[i] = i;
    //spi->tx_buffer[0]               = SPI_HEADER;
    //spi->tx_buffer[SPI_PACKET_SIZE-1] = SPI_FOOTER;

    /* 只跑一次，永不停止 */
    HAL_SPI_TransmitReceive_DMA(&hspi1,
                                spi->tx_buffer,
                                spi->rx_buffer,
                                SPI_PACKET_SIZE);
}

// 开始SPI传输
void SPI_Slave_Start_Transfer(void)
{
    SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
    
    if(spi->packet_to_send)
    {
        // 启动SPI全双工DMA传输
        HAL_SPI_TransmitReceive_DMA(&hspi1, 
                                   spi->tx_buffer,
                                   spi->rx_buffer,
                                   SPI_PACKET_SIZE);
        spi->packet_to_send = 0;
    }
}

// 处理SPI传输
void SPI_Slave_Handle_Transfer(void)
{
    SPI_Slave_Handle_t *spi = Get_SPI_Slave_Handle();
    
    if(spi->spi_state == SPI_STATE_COMPLETE)
    {
        // 传输完成，准备下一次发送数据
        if(spi->packet_received)
        {
            // 可以处理接收到的数据
            // 这里可以根据接收数据生成新的发送数据
            
            // 生成新的发送数据
            static uint8_t counter = 0;
            counter++;
            
            memset(spi->tx_buffer, 0, SPI_PACKET_SIZE);
            spi->tx_buffer[0] = SPI_HEADER;
            spi->tx_buffer[SPI_PACKET_SIZE-1] = SPI_FOOTER;
            
            // 填充数据（示例：递增数据）
            for(uint8_t i = 1; i < SPI_PACKET_SIZE-1; i++)
            {
                spi->tx_buffer[i] = counter + i;
            }
            
            spi->packet_to_send = 1;
            spi->packet_received = 0;
        }
        
        spi->spi_state = SPI_STATE_IDLE;
    }
}
/* SPI slave init function */




/* USER CODE BEGIN 1 */


/**
 * @brief qr_wl spi2canV3 协议校验和计算
 *
 * @details
 * 校验和算法: XOR + 0xAA
 *   1. 将数据按 32-bit 字 (4 字节) 分组
 *   2. 对所有 32-bit 字进行 XOR 运算
 *   3. 结果加上 0xAA (qr_wl 协议特征值)
 *
 * 示例计算:
 *   数据: [0xAA, 0x03, 0x03, 0x01, 0x00, 0xFC, 0xFF, ...]
 *   1. 转换为 uint32_t: [0x030303AA, 0xFFFC0001, ...]
 *   2. XOR: 0x030303AA ^ 0xFFFC0001 ^ ... = 0x898889AA
 *   3. +0xAA: 0x898889AA + 0xAA = 0x89888A54
 *
 * @param data 字节数据指针 (包含 start, header, CAN data, padding)
 * @param len  字节长度 (必须是 4 的倍数，不包括校验和本身的 4 字节)
 * @return uint32_t 32-bit 校验和
 *
 * @note 调用前确保 len 是 4 字节对齐的 (使用 SPI_FRAME_ALIGNED_LEN 宏)
 * @note 校验范围: 从 0xAA 起始到 padding 末尾 (包含 padding)
 */
 
uint32_t xor_checksum_qrwl(uint8_t* data, size_t len)
{
    uint32_t checksum = 0;
    uint32_t* doubleWordData = (uint32_t*)data;
    uint32_t doubleWordLen = len / 4;  // 按 32-bit 字计算

    for (size_t i = 0; i < doubleWordLen; i++) {
        checksum = checksum ^ doubleWordData[i];  // 逐个 32-bit 字 XOR
			//printf("checksum 0x%08X \n", checksum);
    }

    checksum += 0xAA;  // qr_wl 协议特征：XOR 结果 + 0xAA
		//printf("result checksum 0x%08X \n", checksum);
    return checksum;
}


//验证校验码
void VerificationCheckCode()
{
    // 4. 验证校验和 (校验范围: start ~ padding 末尾，对齐到 4 字节)
    // 注意: 校验和必须包含 padding 字节，因为 qr_wl 协议要求 4 字节对齐
    uint32_t aligned_len = SPI_FRAME_ALIGNED_LEN(can1_rx_count, can2_rx_count)-1;
    uint32_t received_checksum = *((uint32_t*)&rx_buff[aligned_len]);
    uint32_t calc_checksum = xor_checksum_qrwl(rx_buff, aligned_len);
    //printf("Checksum mismatch 0x%08X != 0x%08X\n", received_checksum, calc_checksum);
    if (received_checksum != calc_checksum) {
         //printf("ERR: Checksum mismatch 0x%08X != 0x%08X\n", received_checksum, calc_checksum);
        return;
    }
   
    // 5. 检查帧结束标志
//    uint8_t end_marker = rx_buff[expected_len - 1];
//    if (end_marker != SPI_FRAME_END) {
//         printf("ERR: Invalid end marker 0x%02X expected_len%d\n", end_marker,expected_len);
//        return;
//    }
}






/**
 * @brief 构建 SPI 返回帧 (电机反馈数据)
 *
 * @details
 * 从 CAN1/CAN2 总线读取电机反馈 (位置、速度、力矩)，构建符合 qr_wl spi2canV3 协议的返回帧。
 *
 * 返回帧格式:
 *   [0xAA][can1_num][can2_num][CAN1 反馈...][CAN2 反馈...][padding][checksum 4B][0xBB]
 *
 * 每个 CAN 反馈 (10 bytes):
 *   - ID (2B, 小端序): 响应的电机 ID
 *   - Data (8B): MIT 协议编码的电机状态
 *     * MIT 电机实际返回 6 字节: [ID, 位置 2B, 速度 1.5B, 电流 1.5B]
 *     * 本函数根据 rxMsg.len 动态拷贝，不足 8 字节补 0x00
 *     * Byte 0-1: 位置 (16-bit)
 *     * Byte 2-3: 速度 (12-bit)
 *     * Byte 4-5: 力矩/电流 (12-bit)
 *     * Byte 6-7: 补 0x00 (MIT 电机不使用)
 *
 * CAN 反馈长度处理:
 *   - MIT 电机: rxMsg.len = 6 → 拷贝 6 字节 + 补 2 字节 0x00
 *   - 标准 CAN: rxMsg.len = 8 → 拷贝 8 字节
 *   - 超长数据: rxMsg.len > 8 → 仅拷贝前 8 字节
 *
 * 无反馈处理:
 *   如果 CAN 总线无数据 (电机离线/超时)，则填充 10 个 0xFF
 *
 * 填充规则:
 *   Data 部分必须 4 字节对齐，不足部分用 0x00 填充
 *
 * @note 调用前必须先设置 can1_rx_count 和 can2_rx_count
 * @note 结果存储在全局变量 tx_buff 中，供下次 SPI 传输使用
 * @note CAN 读取是非阻塞的 (can.read() 返回 false 表示无数据)
 * @note MIT 电机固件: /home/wl/STM32/MIT_Driver MBED/main.c (txMsg.len = 6)
 */
void build_spi_response()
{
    uint32_t tx_offset = 0;

    // 1. 帧头
	spi_slave.tx_buffer[0] = SPI_FRAME_START;  // 0xAA
    spi_slave.tx_buffer[1] = 3;//can1_rx_count;
    spi_slave.tx_buffer[2] = 3;//can2_rx_count;

	//将CAN1读取到的数据填充进SPI发送BUFF
	for(int i=0;i<10;i++)
	   spi_slave.tx_buffer[i+3]  = Can1_RxDATA.CAN1_DATA1[i];
	for(int i=0;i<10;i++)
	   spi_slave.tx_buffer[i+13] = Can1_RxDATA.CAN1_DATA2[i];
	for(int i=0;i<10;i++)
	   spi_slave.tx_buffer[i+23] = Can1_RxDATA.CAN1_DATA3[i];
	
    //将CAN2读取到的数据填充进SPI发送BUFF
	for(int i=0;i<10;i++)
       spi_slave.tx_buffer[i+33] = Can2_RxDATA.CAN2_DATA1[i];
    for(int i=0;i<10;i++)
       spi_slave.tx_buffer[i+43] = Can2_RxDATA.CAN2_DATA2[i];
	for(int i=0;i<10;i++)
       spi_slave.tx_buffer[i+53] = Can2_RxDATA.CAN2_DATA3[i];


    // 4. Padding 到 4 字节对齐
    uint32_t aligned_len = SPI_FRAME_ALIGNED_LEN(3,3)-1;   //can1_rx_count, can2_rx_count)-1;
    
		//printf("tx_offset = %d aligned_len = %d",tx_offset,aligned_len);
    //while (tx_offset < aligned_len) {
    //    tx_buff[tx_offset++] = 0x00;  // Padding 字节
    //}

    // 5. 计算并填充校验和
    uint32_t tx_checksum = xor_checksum_qrwl(spi_slave.tx_buffer, 63);
    *((uint32_t*)&spi_slave.tx_buffer[63]) = tx_checksum;
    //tx_offset += 4;

    // 6. 帧尾
    spi_slave.tx_buffer[67] = SPI_FRAME_END;  // 0xBB
}

//生成数据（测试）
/* 从 CAN1/CAN2 总线读取电机反馈 (位置、速度、力矩)，构建符合 qr_wl spi2canV3 协议的返回帧。
 *
 * 返回帧格式:
 *   [0xAA][can1_num][can2_num][CAN1 反馈...][CAN2 反馈...][padding][checksum 4B][0xBB]
 *
 * 每个 CAN 反馈 (10 bytes):
 *   - ID (2B, 小端序): 响应的电机 ID
 *   - Data (8B): MIT 协议编码的电机状态
 *     * MIT 电机实际返回 6 字节: [ID, 位置 2B, 速度 1.5B, 电流 1.5B]
 *     * 本函数根据 rxMsg.len 动态拷贝，不足 8 字节补 0x00
 *     * Byte 0-1: 位置 (16-bit)
 *     * Byte 2-3: 速度 (12-bit)
 *     * Byte 4-5: 力矩/电流 (12-bit)
 *     * Byte 6-7: 补 0x00 (MIT 电机不使用)
 *
 * CAN 反馈长度处理:
 *   - MIT 电机: rxMsg.len = 6 → 拷贝 6 字节 + 补 2 字节 0x00
 *   - 标准 CAN: rxMsg.len = 8 → 拷贝 8 字节
 *   - 超长数据: rxMsg.len > 8 → 仅拷贝前 8 字节
 *
 * 无反馈处理:
 *   如果 CAN 总线无数据 (电机离线/超时)，则填充 10 个 0xFF
 *
 * 填充规则:
 *   Data 部分必须 4 字节对齐，不足部分用 0x00 填充*/
void CAN_GeneratedData()
{
	//1号电机
	Can1_RxDATA.CAN1_DATA1[0] = TxID_Number[0]&0xff;               //SPI要发送的1号电机编号
	Can1_RxDATA.CAN1_DATA1[1] = (TxID_Number[0]&0xff00) >> 8;      //SPI要发送的1号电机编号高位
	Can1_RxDATA.CAN1_DATA1[2] =  TxMotorPosition[0]&0xff;          //SPI要发送的1号电机位置数据低
	Can1_RxDATA.CAN1_DATA1[3] = (TxMotorPosition[0]&0xff00) >> 8;  //SPI要发送的1号电机位置数据高
	Can1_RxDATA.CAN1_DATA1[4] = TxMotorSpeed[0]&0xff;              //SPI要发送的1号电机速度L        
	Can1_RxDATA.CAN1_DATA1[5] = (TxMotorSpeed[0]&0xff00) >> 8;     //SPI要发送的1号电机速度H
	Can1_RxDATA.CAN1_DATA1[6] = TxMotorTorque[0]&0xff;             //SPI要发送的1号电机的力矩/电流L
	Can1_RxDATA.CAN1_DATA1[7] = (TxMotorTorque[0]&0xff00) >> 8;    //SPI要发送的1号电机的力矩/电流H
	Can1_RxDATA.CAN1_DATA1[8] = TxMotorOffsetValue[0]&0xff;        //SPI要发送的1号电机的补偿值L
	Can1_RxDATA.CAN1_DATA1[9] = (TxMotorOffsetValue[0]&0xff00) >> 8;  //SPI要发送的1号电机的补偿值H
	//2号电机：
    Can1_RxDATA.CAN1_DATA2[0] = TxID_Number[1]&0xff;               //SPI要发送的2号电机编号
	Can1_RxDATA.CAN1_DATA2[1] = (TxID_Number[1]&0xff00) >> 8;      //SPI要发送的2号电机编号高位
	Can1_RxDATA.CAN1_DATA2[2] =  TxMotorPosition[1]&0xff;          //SPI要发送的2号电机位置数据低
	Can1_RxDATA.CAN1_DATA2[3] = (TxMotorPosition[1]&0xff00) >> 8;  //SPI要发送的2号电机位置数据高
	Can1_RxDATA.CAN1_DATA2[4] = TxMotorSpeed[1]&0xff;              //SPI要发送的2号电机速度L        
	Can1_RxDATA.CAN1_DATA2[5] = (TxMotorSpeed[1]&0xff00) >> 8;     //SPI要发送的2号电机速度H
	Can1_RxDATA.CAN1_DATA2[6] = TxMotorTorque[1]&0xff;             //SPI要发送的2号电机的力矩/电流L
	Can1_RxDATA.CAN1_DATA2[7] = (TxMotorTorque[1]&0xff00) >> 8;    //SPI要发送的2号电机的力矩/电流H
	Can1_RxDATA.CAN1_DATA2[8] = TxMotorOffsetValue[1]&0xff;        //SPI要发送的2号电机的补偿值L
	Can1_RxDATA.CAN1_DATA2[9] = (TxMotorOffsetValue[1]&0xff00) >> 8;  //SPI要发送的2号电机的补偿值H
	//3号电机：
	Can1_RxDATA.CAN1_DATA3[0] = TxID_Number[2]&0xff;               //SPI要发送的3号电机编号
	Can1_RxDATA.CAN1_DATA3[1] = (TxID_Number[2]&0xff00) >> 8;      //SPI要发送的3号电机编号高位
	Can1_RxDATA.CAN1_DATA3[2] =  TxMotorPosition[2]&0xff;          //SPI要发送的3号电机位置数据低
	Can1_RxDATA.CAN1_DATA3[3] = (TxMotorPosition[2]&0xff00) >> 8;  //SPI要发送的3号电机位置数据高
	Can1_RxDATA.CAN1_DATA3[4] = TxMotorSpeed[2]&0xff;              //SPI要发送的3号电机速度L        
	Can1_RxDATA.CAN1_DATA3[5] = (TxMotorSpeed[2]&0xff00) >> 8;     //SPI要发送的3号电机速度H
	Can1_RxDATA.CAN1_DATA3[6] = TxMotorTorque[2]&0xff;             //SPI要发送的3号电机的力矩/电流L
	Can1_RxDATA.CAN1_DATA3[7] = (TxMotorTorque[2]&0xff00) >> 8;    //SPI要发送的3号电机的力矩/电流H
	Can1_RxDATA.CAN1_DATA3[8] = TxMotorOffsetValue[2]&0xff;        //SPI要发送的3号电机的补偿值L
	Can1_RxDATA.CAN1_DATA3[9] = (TxMotorOffsetValue[2]&0xff00) >> 8;  //SPI要发送的3号电机的补偿值H
    //4号电机：
	Can2_RxDATA.CAN2_DATA1[0] = TxID_Number[3]&0xff;               //SPI要发送的4号电机编号
	Can2_RxDATA.CAN2_DATA1[1] = (TxID_Number[3]&0xff00) >> 8;      //SPI要发送的4号电机编号高位
	Can2_RxDATA.CAN2_DATA1[2] =  TxMotorPosition[3]&0xff;          //SPI要发送的4号电机位置数据低
	Can2_RxDATA.CAN2_DATA1[3] = (TxMotorPosition[3]&0xff00) >> 8;  //SPI要发送的4号电机位置数据高
	Can2_RxDATA.CAN2_DATA1[4] = TxMotorSpeed[3]&0xff;              //SPI要发送的4号电机速度L        
	Can2_RxDATA.CAN2_DATA1[5] = (TxMotorSpeed[3]&0xff00) >> 8;     //SPI要发送的4号电机速度H
	Can2_RxDATA.CAN2_DATA1[6] = TxMotorTorque[3]&0xff;             //SPI要发送的4号电机的力矩/电流L
	Can2_RxDATA.CAN2_DATA1[7] = (TxMotorTorque[3]&0xff00) >> 8;    //SPI要发送的4号电机的力矩/电流H
	Can2_RxDATA.CAN2_DATA1[8] = TxMotorOffsetValue[3]&0xff;        //SPI要发送的4号电机的补偿值L
	Can2_RxDATA.CAN2_DATA1[9] = (TxMotorOffsetValue[3]&0xff00) >> 8;  //SPI要发送的4号电机的补偿值H
	//5号电机：
	Can2_RxDATA.CAN2_DATA2[0] = TxID_Number[4]&0xff;               //SPI要发送的5号电机编号
	Can2_RxDATA.CAN2_DATA2[1] = (TxID_Number[4]&0xff00) >> 8;      //SPI要发送的5号电机编号高位
	Can2_RxDATA.CAN2_DATA2[2] =  TxMotorPosition[4]&0xff;          //SPI要发送的5号电机位置数据低
	Can2_RxDATA.CAN2_DATA2[3] = (TxMotorPosition[4]&0xff00) >> 8;  //SPI要发送的5号电机位置数据高
	Can2_RxDATA.CAN2_DATA2[4] = TxMotorSpeed[4]&0xff;              //SPI要发送的5号电机速度L        
	Can2_RxDATA.CAN2_DATA2[5] = (TxMotorSpeed[4]&0xff00) >> 8;     //SPI要发送的5号电机速度H
	Can2_RxDATA.CAN2_DATA2[6] = TxMotorTorque[4]&0xff;             //SPI要发送的5号电机的力矩/电流L
	Can2_RxDATA.CAN2_DATA2[7] = (TxMotorTorque[4]&0xff00) >> 8;    //SPI要发送的5号电机的力矩/电流H
	Can2_RxDATA.CAN2_DATA2[8] = TxMotorOffsetValue[4]&0xff;        //SPI要发送的5号电机的补偿值L
	Can2_RxDATA.CAN2_DATA2[9] = (TxMotorOffsetValue[4]&0xff00) >> 8;  //SPI要发送的5号电机的补偿值H
	//6号电机：
	Can2_RxDATA.CAN2_DATA3[0] = TxID_Number[5]&0xff;               //SPI要发送的6号电机编号
	Can2_RxDATA.CAN2_DATA3[1] = (TxID_Number[5]&0xff00) >> 8;      //SPI要发送的6号电机编号高位
	Can2_RxDATA.CAN2_DATA3[2] =  TxMotorPosition[5]&0xff;          //SPI要发送的6号电机位置数据低
	Can2_RxDATA.CAN2_DATA3[3] = (TxMotorPosition[5]&0xff00) >> 8;  //SPI要发送的6号电机位置数据高
	Can2_RxDATA.CAN2_DATA3[4] = TxMotorSpeed[5]&0xff;              //SPI要发送的6号电机速度L        
	Can2_RxDATA.CAN2_DATA3[5] = (TxMotorSpeed[5]&0xff00) >> 8;     //SPI要发送的6号电机速度H
	Can2_RxDATA.CAN2_DATA3[6] = TxMotorTorque[5]&0xff;             //SPI要发送的6号电机的力矩/电流L
	Can2_RxDATA.CAN2_DATA3[7] = (TxMotorTorque[5]&0xff00) >> 8;    //SPI要发送的6号电机的力矩/电流H
	Can2_RxDATA.CAN2_DATA3[8] = TxMotorOffsetValue[5]&0xff;        //SPI要发送的6号电机的补偿值L
	Can2_RxDATA.CAN2_DATA3[9] = (TxMotorOffsetValue[5]&0xff00) >> 8;  //SPI要发送的6号电机的补偿值H
	
}


/* USER CODE END 1 */
