#include "can_data.h"
#include <string.h>
#include "main.h"
#include "spi_slave.h"
#include "gpio.h"


/* USER CODE BEGIN 1 */
uint8_t CAN1RxData[8];
uint8_t CAN2RxData[8];
uint8_t CAN1_TX_Data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};     //测试
uint8_t CAN2_TX_Data[8] = {0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10};     //测试
CanData_t can1_rx_msgs[OneLegMAX_CAN_MOTORS];  // 从上位机接收的 CAN1 消息；上位机->STM32->CAN1->电机的CAN1数据
CanData_t can2_rx_msgs[OneLegMAX_CAN_MOTORS];  // 从上位机接收的 CAN2 消息；上位机->STM32->CAN2->电机的CAN2数据

// spi buffers (修改为 8-bit 字节缓冲区，匹配 qr_wl spi2canV3 协议)
extern uint8_t rx_buff[RX_LEN];  //接收的SPI数据缓冲区 (8-bit 字节) ；上位机->SPI->STM32的SPI数据
extern uint8_t tx_buff[RX_LEN];  //发送的SPI数据缓冲区 (8-bit 字节) ；STM32->SPI->上位机的SPI数据
uint8_t can1_rx_count = 3;         // CAN1 接收消息数量
uint8_t can2_rx_count = 3;         // CAN2 接收消息数量
uint8_t PB15 = 0;


/* 队列句柄定义 */
// 全局变量用于任务间通信
osMessageQueueId_t can1QueueHandle = NULL;  // 需要在FreeRTOS中创建
osMessageQueueId_t can2QueueHandle = NULL;  
uint8_t data[8];
CAN_RxHeaderTypeDef CAN1RxHeader;
// 创建消息结构体（接收）
CAN_Message_t Can1Msg[OneLegMAX_CAN_MOTORS];  
CAN_Message_t Can2Msg[OneLegMAX_CAN_MOTORS];
uint8_t CAN1RxPacketNumber ;   //CAN1接收数据包的数量
uint8_t CAN2RxPacketNumber ;   //CAN2接收数据包的数量
//创建消息结构体 （发送）
CAN_TxMessage_t                 CAN1_TxMessa;
CAN_TxMessage_t                 CAN2_TxMessa;
uint64_t u8SendCount = 0;


////////////////////////////////////////////////////////////////////////////////////////////
//电机的ID号：测试
extern uint16_t RxID_Number[NumberOfMotors];              //SPI读取到的电机ID编号
extern uint16_t TxID_Number[NumberOfMotors];              //SPI要发送的电机ID编号
//SPI发送协议：
// *     * Byte 0-1: 位置 (16-bit)
// *     * Byte 2-3: 速度 (12-bit)
// *     * Byte 4-5: 力矩/电流 (12-bit)
// *     * Byte 6-7: 补 0x00 (MIT 电机不使用)
//SPI接收协议
// * MIT 协议命令格式 (8 bytes):
// *   - Byte 0-1: 位置 (16-bit)
// *   - Byte 2-3: 速度 (12-bit) + Kp (12-bit 高 4 位)
// *   - Byte 4: Kp (低 8 位)
// *   - Byte 5-6: Kd (12-bit) + 力矩 (12-bit 高 4 位)
// *   - Byte 7: 力矩 (低 8 位) 或特殊命令 (0xFC/0xFD/0xFE)
//位置：
extern uint16_t RxMotorPosition[NumberOfMotors];          //SPI读取到的6个电机位置
extern uint16_t TxMotorPosition[NumberOfMotors];          //SPI要发送的6个电机位置
//速度：
extern uint16_t RxMotorSpeed[NumberOfMotors];             //SPI读取到的6个电机的速度+Kp高4位
extern uint16_t TxMotorSpeed[NumberOfMotors];             //SPI要发送的6个电机的速度
//Kp：
extern uint16_t RxMotorKp[NumberOfMotors];                //SPI读取到的6个电机的Kp
//Kd
extern uint16_t RxMotorKd[NumberOfMotors];                //SPI读取到的6个电机的Kd
//力矩:
extern uint16_t RxMotorTorque[NumberOfMotors];            //SPI读取到的6个电机的力矩/电流
extern uint16_t TxMotorTorque[NumberOfMotors];            //SPI要发送的6个电机的力矩/电流
//其他
extern uint16_t RxMotorTorqueL[NumberOfMotors];           //SPI读取到的6个电机的力矩低
extern uint16_t TxMotorOffsetValue[NumberOfMotors];       //SPI要发送的6个电机的补偿值
//////////////////////////////////////////////////////////////////////////////////////////////







/* 初始化函数 */
void CAN_Data_Init(void)
{
    /* 创建CAN消息队列 */
    can1QueueHandle = osMessageQueueNew(16, sizeof(CAN_Message_t), NULL);
    can2QueueHandle = osMessageQueueNew(16, sizeof(CAN_Message_t), NULL);
    
    if (can1QueueHandle == NULL || can2QueueHandle == NULL)
    {
        Error_Handler();
    }
}



/* 处理接收到的CAN消息 */
void CAN_Process_Rx_Message(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    CAN_Message_t canMsg;
    
    /* 填充消息结构体 */
    canMsg.stdId = RxHeader->StdId;
    canMsg.extId = RxHeader->ExtId;
    canMsg.ide = (RxHeader->IDE == CAN_ID_STD) ? 0 : 1;
    canMsg.rtr = (RxHeader->RTR == CAN_RTR_DATA) ? 0 : 1;
    canMsg.dlc = RxHeader->DLC;
    canMsg.timestamp = RxHeader->Timestamp;
    memcpy(canMsg.data, RxData, RxHeader->DLC);
    
    /* 根据CAN实例发送到不同队列 */
    if (hcan->Instance == CAN1 && can1QueueHandle != NULL)
    {
        osMessageQueuePut(can1QueueHandle, &canMsg, 0, 0);
    }
    else if (hcan->Instance == CAN2 && can2QueueHandle != NULL)
    {
        osMessageQueuePut(can2QueueHandle, &canMsg, 0, 0);
    }
}


//CAN发送
void AND_CanSendServe(CAN_HandleTypeDef hcan,CAN_TxMessage_t CAN_TxMessa)
{
    uint32_t TxMailbox = 0;
    //CAN1发送数据
	if(hcan.Instance == CAN1)     //如果是CAN1
	{
		if ( HAL_OK ==  HAL_CAN_AddTxMessage ( &hcan, ( CAN_TxHeaderTypeDef * ) &CAN_TxMessa, CAN_TxMessa.Data, &TxMailbox ) )   //向第一个空闲Tx邮箱添加一条消息，并激活相应的传输请求。
		{
			u8SendCount++;
		}
	}
	//CAN2发送数据
	if(hcan.Instance == CAN2)     //如果是CAN2
	{
		if ( HAL_OK ==  HAL_CAN_AddTxMessage ( &hcan, ( CAN_TxHeaderTypeDef * ) &CAN_TxMessa, CAN_TxMessa.Data, &TxMailbox ) )   //向第一个空闲Tx邮箱添加一条消息，并激活相应的传输请求。
		{
			u8SendCount++;
		}
	}
}


//CAN接收数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    //uint8_t RxData[8];
    if(hcan->Instance ==CAN1)
	{
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN1RxData) == HAL_OK)
		{
			Can1Msg[CAN1RxPacketNumber].stdId = RxHeader.StdId;     // 标准ID
			Can1Msg[CAN1RxPacketNumber].extId = RxHeader.ExtId;     // 扩展ID
			Can1Msg[CAN1RxPacketNumber].rtr = RxHeader.RTR;         // RTR位：0=数据帧，1=远程帧
			Can1Msg[CAN1RxPacketNumber].ide = RxHeader.IDE;         // IDE位：0=标准帧，1=扩展帧
			Can1Msg[CAN1RxPacketNumber].dlc = RxHeader.DLC;         // 数据长度(0-8)
			Can1Msg[CAN1RxPacketNumber].timestamp = RxHeader.Timestamp;    // 时间戳
			memcpy(Can1Msg[CAN1RxPacketNumber].data, CAN1RxData, 8);       //
			CAN1RxPacketNumber ++;  //接收句柄下标自加
			if(CAN1RxPacketNumber >= OneLegMAX_CAN_MOTORS)    //当大于等于单腿最大电机数量OneLegMAX_CAN_MOTORS时，则清零
				CAN1RxPacketNumber = 0;
		}
	}
	if(hcan->Instance ==CAN2)
	{
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN2RxData) == HAL_OK)
		{
			Can2Msg[CAN2RxPacketNumber].stdId = RxHeader.StdId;     // 标准ID
			Can2Msg[CAN2RxPacketNumber].extId = RxHeader.ExtId;     // 扩展ID
			Can2Msg[CAN2RxPacketNumber].rtr = RxHeader.RTR;         // RTR位：0=数据帧，1=远程帧
			Can2Msg[CAN2RxPacketNumber].ide = RxHeader.IDE;         // IDE位：0=标准帧，1=扩展帧
			Can2Msg[CAN2RxPacketNumber].dlc = RxHeader.DLC;         // 数据长度(0-8)
			Can2Msg[CAN2RxPacketNumber].timestamp = RxHeader.Timestamp;    // 时间戳
			memcpy(Can2Msg[CAN2RxPacketNumber].data, CAN2RxData, 8);       
			CAN2RxPacketNumber ++;  //接收句柄下标自加
			if(CAN2RxPacketNumber >= OneLegMAX_CAN_MOTORS)    //当大于等于单腿最大电机数量OneLegMAX_CAN_MOTORS时，则清零
				CAN2RxPacketNumber = 0;
		}
	}
}





///**
// * @brief 构建 SPI 返回帧 (电机反馈数据)
// *
// * @details
// * 从 CAN1/CAN2 总线读取电机反馈 (位置、速度、力矩)，构建符合 qr_wl spi2canV3 协议的返回帧。
// *
// * 返回帧格式:
// *   [0xAA][can1_num][can2_num][CAN1 反馈...][CAN2 反馈...][padding][checksum 4B][0xBB]
// *
// * 每个 CAN 反馈 (10 bytes):
// *   - ID (2B, 小端序): 响应的电机 ID
// *   - Data (8B): MIT 协议编码的电机状态
// *     * MIT 电机实际返回 6 字节: [ID, 位置 2B, 速度 1.5B, 电流 1.5B]
// *     * 本函数根据 rxMsg.len 动态拷贝，不足 8 字节补 0x00
// *     * Byte 0-1: 位置 (16-bit)
// *     * Byte 2-3: 速度 (12-bit)
// *     * Byte 4-5: 力矩/电流 (12-bit)
// *     * Byte 6-7: 补 0x00 (MIT 电机不使用)
// *
// * CAN 反馈长度处理:
// *   - MIT 电机: rxMsg.len = 6 → 拷贝 6 字节 + 补 2 字节 0x00
// *   - 标准 CAN: rxMsg.len = 8 → 拷贝 8 字节
// *   - 超长数据: rxMsg.len > 8 → 仅拷贝前 8 字节
// *
// * 无反馈处理:
// *   如果 CAN 总线无数据 (电机离线/超时)，则填充 10 个 0xFF
// *
// * 填充规则:
// *   Data 部分必须 4 字节对齐，不足部分用 0x00 填充
// *
// * @note 调用前必须先设置 can1_rx_count 和 can2_rx_count
// * @note 结果存储在全局变量 tx_buff 中，供下次 SPI 传输使用
// * @note CAN 读取是非阻塞的 (can.read() 返回 false 表示无数据)
// * @note MIT 电机固件: /home/wl/STM32/MIT_Driver MBED/main.c (txMsg.len = 6)
// */
//void build_spi_response()
//{
//    uint32_t tx_offset = 0;

//    // 1. 帧头
//    tx_buff[tx_offset++] = SPI_HEADER;  // 0xAA
//    tx_buff[tx_offset++] = can1_rx_count;
//    tx_buff[tx_offset++] = can2_rx_count;

//    // 2. 读取 CAN1 反馈并填充
//    for (uint8_t i = 0; i < can1_rx_count; i++) {
//        CANMessage rxMsg;
//        //printf("readyReadCan1 \r\n");
//        if (can1.read(rxMsg)) {
//					//for(int j=0;j<8;j++)
//					// printf("can1.read = %x ",rxMsg.data[j]);
//					 //printf("\r\n");
//            // 有反馈: 填充 ID (小端序) + data
//            tx_buff[tx_offset++] = rxMsg.d & 0xFF;        // ID 低字节
//            tx_buff[tx_offset++] = (rxMsg.id >> 8) & 0xFF; // ID 高字节

//            // 动态拷贝: MIT 电机返回 6 字节，标准 CAN 最多 8 字节
//            uint8_t actual_len = (rxMsg.len > 8) ? 8 : rxMsg.len;
//            memcpy(&tx_buff[tx_offset], rxMsg.data, actual_len);
//            tx_offset += actual_len;

//            // 不足 8 字节补 0x00 (保持 qr_wl 协议兼容性)
//            if (actual_len < 8) {
//                memset(&tx_buff[tx_offset], 0x00, 8 - actual_len);
//                tx_offset += (8 - actual_len);
//            }
//        } else {
//            // 无反馈: 填充 0xFF
//            memset(&tx_buff[tx_offset], 0xFF, 10);
//            tx_offset += 10;
//        }
//    }

//    // 3. 读取 CAN2 反馈并填充
//    for (uint8_t i = 0; i < can2_rx_count; i++) {
//        CANMessage rxMsg;

//        if (can2.read(rxMsg)) {
//					//for(int j=0;j<8;j++)
//					 //printf("can2.read = %x ",rxMsg.data[j]);
//					 //printf("\r\n");
//            tx_buff[tx_offset++] = rxMsg.id & 0xFF;
//            tx_buff[tx_offset++] = (rxMsg.id >> 8) & 0xFF;

//            // 动态拷贝: MIT 电机返回 6 字节，标准 CAN 最多 8 字节
//            uint8_t actual_len = (rxMsg.len > 8) ? 8 : rxMsg.len;
//            memcpy(&tx_buff[tx_offset], rxMsg.data, actual_len);
//            tx_offset += actual_len;

//            // 不足 8 字节补 0x00 (保持 qr_wl 协议兼容性)
//            if (actual_len < 8) {
//                memset(&tx_buff[tx_offset], 0x00, 8 - actual_len);
//                tx_offset += (8 - actual_len);
//            }
//        } else {
//            memset(&tx_buff[tx_offset], 0xFF, 10);
//            tx_offset += 10;
//        }
//    }

//    // 4. Padding 到 4 字节对齐
//    uint32_t aligned_len = SPI_FRAME_ALIGNED_LEN(can1_rx_count, can2_rx_count)-1;
//    
//		//printf("tx_offset = %d aligned_len = %d",tx_offset,aligned_len);
//    //while (tx_offset < aligned_len) {
//    //    tx_buff[tx_offset++] = 0x00;  // Padding 字节
//    //}

//    // 5. 计算并填充校验和
//    uint32_t tx_checksum = xor_checksum_qrwl(tx_buff, 63);
//    *((uint32_t*)&tx_buff[tx_offset]) = tx_checksum;
//    tx_offset += 4;

//    // 6. 帧尾
//    tx_buff[tx_offset++] = SPI_FRAME_END;  // 0xBB
//		 
//		//for(int i=0;i<69;i++)
//		//printf(" %X ",tx_buff[i]);
//		//printf("\r\n");
//	
//}






/**
* @brief qr_wl spi2canV3 协议控制函数 (CAN 消息透传) (STM32->CAN->电机)
 *
 * @details
 * 将从 SPI 接收的 CAN 消息透传到 CAN1/CAN2 总线，发送 MIT 协议命令到电机。
 *
 * 工作流程:
 *   1. 发送 CAN1 消息:
 *      - 遍历 can1_rx_msgs 数组 (最多 6 个消息)
 *      - 构建 CANMessage 并发送到 can1 总线
 *      - 每个消息间隔 20us (避免 CAN 总线冲突)
 *   2. 发送 CAN2 消息:
 *      - 遍历 can2_rx_msgs 数组
 *      - 构建 CANMessage 并发送到 can2 总线
 *   3. E-stop 处理:
 *      - 如果急停按钮触发 (estop == 0)
 *      - 发送失能命令 (0xFD) 到所有活跃电机
 *      - 清空消息计数和缓冲区
 *
 * MIT 协议命令格式 (8 bytes):
 *   - Byte 0-1: 位置 (16-bit)
 *   - Byte 2-3: 速度 (12-bit) + Kp (12-bit 高 4 位)
 *   - Byte 4: Kp (低 8 位)
 *   - Byte 5-6: Kd (12-bit) + 力矩 (12-bit 高 4 位)
 *   - Byte 7: 力矩 (低 8 位) 或特殊命令 (0xFC/0xFD/0xFE)
 *
 * 特殊命令:
 *   - 0xFF FF FF FF FF FF FF FC: 零位标定
 *   - 0xFF FF FF FF FF FF FF FD: 失能
 *   - 0xFF FF FF FF FF FF FF FE: 使能
 *
 * 安全特性:
 *   - E-stop 自动失能所有电机
 *   - CAN 发送间隔 20us (防止总线冲突)
 *   - 电机数量限制 (MAX_CAN_MOTORS = 6)
 *
 * @note 本函数在 spi_isr() 中断上下文中调用
 * @note 全局变量: can1_rx_msgs, can2_rx_msgs, can1_rx_count, can2_rx_count, estop
 * @note 电机反馈读取在 build_spi_response() 中完成
 */
void CANcontrol()
{
    // ================ 发送 CAN1 消息到 CAN1 总线 ================
    for (uint8_t i = 0; i < can1_rx_count && i < MAX_CAN_MOTORS; i++) {
        //CANMessage msg;
        CAN1_TxMessa.StdId = can1_rx_msgs[i].id;
		CAN1_TxMessa.DLC = 8;
        memcpy(CAN1_TxMessa.Data, can1_rx_msgs[i].data, 8);
        AND_CanSendServe(hcan1,CAN1_TxMessa); //CAN发送
		osDelay(1);// >20us 延时，避免 CAN 总线冲突
    }
    // ================ 发送 CAN2 消息到 CAN2 总线 ================
    for (uint8_t i = 0; i < can2_rx_count && i < MAX_CAN_MOTORS; i++) {
        //CANMessage msg;
        CAN2_TxMessa.StdId = can2_rx_msgs[i].id;
		CAN2_TxMessa.DLC = 8;
        memcpy(CAN2_TxMessa.Data , can2_rx_msgs[i].data, 8);
        AND_CanSendServe(hcan2,CAN2_TxMessa); //CAN发送
		osDelay(1);// >20us 延时，避免 CAN 总线冲突
    }

    // ================ 读取电机反馈 (可选) ================
    // 注意: 原 SPIne 协议会立即读取反馈并填充 spi_data
    // qr_wl 协议暂时不需要立即反馈，可在后续迭代中添加

    // 方案1: 不读取反馈，直接返回 (最简单)
    // 方案2: 读取 CAN 反馈，填充 spine2upboard 结构 (需要额外实现)

    // 当前实现方案1: 不读取反馈
    // 如需实现方案2，可参考以下伪代码:
    /*
    CANMessage rxMsg;
    if (can1.read(rxMsg)) {
        // 解析 MIT 协议反馈，填充 spine2upboard
    }
    if (can2.read(rxMsg)) {
        // 解析 MIT 协议反馈，填充 spine2upboard
    }
    */
	
    // ================ E-stop 处理 (增强版) ================
	PB15 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
    if(0 == PB15)               //如果停止按键触发
	{
        // 紧急停止：发送失能命令到所有电机
        //CANMessage disable_msg;
		CAN_TxMessage_t CAN_TxMessage;    
        //disable_msg.len = 8;
		CAN_TxMessage.DLC = 8;
        for (int j = 0; j < 8; j++) 
		   CAN_TxMessage.Data[j] = 0xFF;
        CAN_TxMessage.Data[7] = 0xFD;  // 失能命令

        // 向之前活跃的电机发送失能命令
        for (uint8_t i = 0; i < can1_rx_count; i++) {           //向CAN1的多个电机发送停止指令
            CAN_TxMessage.ExtId = can1_rx_msgs[i].id;           //填装ID号
			AND_CanSendServe(hcan1,CAN_TxMessage);              //CAN1发送
			osDelay(1);                                         // >20us 延时，避免 CAN 总线冲突
        }
        for (uint8_t i = 0; i < can2_rx_count; i++) {           //向CAN2的多个电机发送停止指令
            CAN_TxMessage.ExtId = can2_rx_msgs[i].id;           //填装ID号
            AND_CanSendServe(hcan2,CAN_TxMessage);              //CAN2发送   
            osDelay(1);                                         // >20us 延时，避免 CAN 总线冲突
        }
        // 清空所有消息计数
        can1_rx_count = 0;
        can2_rx_count = 0;
        //led = 1;
    } else {
        //led = 0;
    }
}

//CAN发送给电机的数据转换(测试用，也可以正常使用)
/* 工作流程:
*   1. 发送 CAN1 消息:
*      - 遍历 can1_rx_msgs 数组 (最多 6 个消息)
*      - 构建 CANMessage 并发送到 can1 总线
*      - 每个消息间隔 20us (避免 CAN 总线冲突)
*   2. 发送 CAN2 消息:
*      - 遍历 can2_rx_msgs 数组
*      - 构建 CANMessage 并发送到 can2 总线
*   3. E-stop 处理:
*      - 如果急停按钮触发 (estop == 0)
*      - 发送失能命令 (0xFD) 到所有活跃电机
*      - 清空消息计数和缓冲区
*
* MIT 协议命令格式 (8 bytes):
*   - Byte 0-1: 位置 (16-bit)
*   - Byte 2-3: 速度 (12-bit) + Kp (12-bit 高 4 位)
*   - Byte 4: Kp (低 8 位)
*   - Byte 5-6: Kd (12-bit) + 力矩 (12-bit 高 4 位)
*   - Byte 7: 力矩 (低 8 位) 或特殊命令 (0xFC/0xFD/0xFE)
*
* MIT 协议命令格式（详细） (8 bit):
*   - Byte 0: 位置数据高8位
    - Byte 1: 位置数据低8位 
*   - Byte 2  12位的速度数据高8位
    - Byte 3: 12位的速度数据低4位放在Byte 3空间的高4位位置 + 12位Kp数据的高4位放在Byte 3空间的低4位位置
*   - Byte 4: 12位的Kp数据低8位
*   - Byte 5：12位的Kd数据高8位
    - Byte 6: 12位的Kd数据低4位放在Byte 6空间的高4位位置 + 12位力矩数据的高4位放在Byte 6空间的低4位位置
*   - Byte 7: 力矩的(低 8 位) 或特殊命令 (0xFC/0xFD/0xFE)
*
* 特殊命令:
*   - 0xFF FF FF FF FF FF FF FC: 零位标定
*   - 0xFF FF FF FF FF FF FF FD: 失能
*   - 0xFF FF FF FF FF FF FF FE: 使能
*
* 安全特性:
*   - E-stop 自动失能所有电机
*   - CAN 发送间隔 20us (防止总线冲突)
*   - 电机数量限制 (MAX_CAN_MOTORS = 6)*/
void CANSend_DataConversion()
{
	//CAN1发送的数据：
	//电机1数据：
    can1_rx_msgs[0].id = CAN1Motor1ID;   //CAN1电机1
	can1_rx_msgs[0].data[0] = (RxMotorPosition[0]&0xff00)>>8;   //位置数据高8位
    can1_rx_msgs[0].data[1] = RxMotorPosition[0]&0xff;          //位置数据低8位
	can1_rx_msgs[0].data[2] = (RxMotorSpeed[0]&0x0ff0)>>4;      //速度数据高8位
    can1_rx_msgs[0].data[3] = (RxMotorSpeed[0]&0x000f)<<4 | (RxMotorKp[0]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can1_rx_msgs[0].data[4] = RxMotorKp[0]&0x00ff;              //Kp低8位
	can1_rx_msgs[0].data[5] = (RxMotorKd[0]&0x0ff0)>>4;         //Kd高8位
	can1_rx_msgs[0].data[6] = (RxMotorKd[0]&0x000f)<<4 | (RxMotorTorque[0]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can1_rx_msgs[0].data[7] = RxMotorTorque[0]&0x00ff;          //力矩 (低 8 位)
	//电机1数据：
    can1_rx_msgs[1].id = CAN1Motor2ID;   //CAN1电机2
	can1_rx_msgs[1].data[0] = (RxMotorPosition[1]&0xff00)>>8;   //位置数据高8位
    can1_rx_msgs[1].data[1] = RxMotorPosition[1]&0xff;          //位置数据低8位
	can1_rx_msgs[1].data[2] = (RxMotorSpeed[1]&0x0ff0)>>4;      //速度数据高8位
    can1_rx_msgs[1].data[3] = (RxMotorSpeed[1]&0x000f)<<4 | (RxMotorKp[1]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can1_rx_msgs[1].data[4] = RxMotorKp[1]&0x00ff;              //Kp低8位
	can1_rx_msgs[1].data[5] = (RxMotorKd[1]&0x0ff0)>>4;         //Kd高8位
	can1_rx_msgs[1].data[6] = (RxMotorKd[1]&0x000f)<<4 | (RxMotorTorque[1]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can1_rx_msgs[1].data[7] = RxMotorTorque[1]&0x00ff;          //力矩 (低 8 位)
	//电机1数据：
    can1_rx_msgs[2].id = CAN1Motor3ID;   //CAN1电机3
	can1_rx_msgs[2].data[0] = (RxMotorPosition[2]&0xff00)>>8;   //位置数据高8位
    can1_rx_msgs[2].data[1] = RxMotorPosition[2]&0xff;          //位置数据低8位
	can1_rx_msgs[2].data[2] = (RxMotorSpeed[2]&0x0ff0)>>4;      //速度数据高8位
    can1_rx_msgs[2].data[3] = (RxMotorSpeed[2]&0x000f)<<4 | (RxMotorKp[2]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can1_rx_msgs[2].data[4] = RxMotorKp[2]&0x00ff;              //Kp低8位
	can1_rx_msgs[2].data[5] = (RxMotorKd[2]&0x0ff0)>>4;         //Kd高8位
	can1_rx_msgs[2].data[6] = (RxMotorKd[2]&0x000f)<<4 | (RxMotorTorque[2]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can1_rx_msgs[2].data[7] = RxMotorTorque[2]&0x00ff;          //力矩 (低 8 位)
	//CAN2发送的数据：
	//电机1数据：
    can2_rx_msgs[0].id = CAN2Motor1ID;   //CAN2电机1
	can2_rx_msgs[0].data[0] = (RxMotorPosition[3]&0xff00)>>8;   //位置数据高8位
    can2_rx_msgs[0].data[1] = RxMotorPosition[3]&0xff;          //位置数据低8位
	can2_rx_msgs[0].data[2] = (RxMotorSpeed[3]&0x0ff0)>>4;      //速度数据高8位
    can2_rx_msgs[0].data[3] = (RxMotorSpeed[3]&0x000f)<<4 | (RxMotorKp[3]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can2_rx_msgs[0].data[4] = RxMotorKp[3]&0x00ff;              //Kp低8位
	can2_rx_msgs[0].data[5] = (RxMotorKd[3]&0x0ff0)>>4;         //Kd高8位
	can2_rx_msgs[0].data[6] = (RxMotorKd[3]&0x000f)<<4 | (RxMotorTorque[3]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can2_rx_msgs[0].data[7] = RxMotorTorque[3]&0x00ff;          //力矩 (低 8 位)
	//电机2数据：
    can2_rx_msgs[1].id = CAN2Motor2ID;   //CAN2电机2
	can2_rx_msgs[1].data[0] = (RxMotorPosition[4]&0xff00)>>8;   //位置数据高8位
    can2_rx_msgs[1].data[1] = RxMotorPosition[4]&0xff;          //位置数据低8位
	can2_rx_msgs[1].data[2] = (RxMotorSpeed[4]&0x0ff0)>>4;      //速度数据高8位
    can2_rx_msgs[1].data[3] = (RxMotorSpeed[4]&0x000f)<<4 | (RxMotorKp[4]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can2_rx_msgs[1].data[4] = RxMotorKp[4]&0x00ff;              //Kp低8位
	can2_rx_msgs[1].data[5] = (RxMotorKd[4]&0x0ff0)>>4;         //Kd高8位
	can2_rx_msgs[1].data[6] = (RxMotorKd[4]&0x000f)<<4 | (RxMotorTorque[4]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can2_rx_msgs[1].data[7] = RxMotorTorque[4]&0x00ff;          //力矩 (低 8 位)
	//电机3数据：
    can2_rx_msgs[2].id = CAN2Motor3ID;   //CAN2电机3
	can2_rx_msgs[2].data[0] = (RxMotorPosition[5]&0xff00)>>8;   //位置数据高8位
    can2_rx_msgs[2].data[1] = RxMotorPosition[5]&0xff;          //位置数据低8位
	can2_rx_msgs[2].data[2] = (RxMotorSpeed[5]&0x0ff0)>>4;      //速度数据高8位
    can2_rx_msgs[2].data[3] = (RxMotorSpeed[5]&0x000f)<<4 | (RxMotorKp[5]&0x0f00)>>8;    //速度数据低4位 + Kp (12-bit 高 4 位)
	can2_rx_msgs[2].data[4] = RxMotorKp[5]&0x00ff;              //Kp低8位
	can2_rx_msgs[2].data[5] = (RxMotorKd[5]&0x0ff0)>>4;         //Kd高8位
	can2_rx_msgs[2].data[6] = (RxMotorKd[5]&0x000f)<<4 | (RxMotorTorque[5]&0x0f00)>>8;   //Kd低4位+力矩 (12-bit 高 4 位)
    can2_rx_msgs[2].data[7] = RxMotorTorque[5]&0x00ff;          //力矩 (低 8 位)
}





/* USER CODE END 1 */











