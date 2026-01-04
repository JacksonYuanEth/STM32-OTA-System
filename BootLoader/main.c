#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include "stm32f4xx_hal.h"

// ================= 配置区 =================
// 1. APP 起始地址 (32KB Bootloader 之后，即 Sector 2)
#define APP_ADDR  0x08008000

// 2. 标志位地址 (SRAM 末尾)
#define BOOT_FLAG_ADDR  0x2001FFF0 
#define BOOT_FLAG_MAGIC 0xDEADBEEF

// 3. 串口接收缓冲区
#define RX_BUFF_SIZE 2048
uint8_t RxBuff[RX_BUFF_SIZE];
// ==========================================

// 函数指针定义
typedef void (*pFunction)(void);

// 函数声明
void Jump_To_App(void);
void Process_Command(void);
void Send_Ack(uint8_t cmd, uint8_t status);
uint8_t Flash_Erase_App(void);
uint8_t Flash_Write_App(uint32_t offset, uint8_t *data, uint16_t len);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // ===========================================================
    // 阶段一：启动检查与救砖逻辑
    // ===========================================================
    
    // 1. 给树莓派留出 "抢占窗口"
    // 如果 APP 彻底死机，树莓派会在上电瞬间疯狂发指令，这里延时让串口有机会接收
    HAL_Delay(200); 

    // 2. 检查标志位 (看看是不是 APP 主动请求升级)
    uint32_t flag = *(__IO uint32_t *)BOOT_FLAG_ADDR;
    
    // 3. 检查 APP 是否有效 (救砖核心)
    // 读取 APP 地址的第一个字 (栈顶指针 MSP)
    // STM32F4 的 SRAM 范围通常在 0x20000000 附近
    uint32_t app_stack = *(__IO uint32_t*)APP_ADDR;
    uint8_t is_app_valid = 0;
    
    // 简单的合法性检查：栈顶指针必须在 SRAM 范围内
    // 0x2FFE0000 是掩码，确保高位是 0x200xxxxx
    if ((app_stack & 0x2FFE0000) == 0x20000000) {
        is_app_valid = 1;
    }

    // 4. 决策跳转逻辑
    if (flag == BOOT_FLAG_MAGIC)
    {
        // 情况 A: 收到 OTA 请求 -> 清除标志，留在 Bootloader 升级
        *(__IO uint32_t *)BOOT_FLAG_ADDR = 0;
    }
    else if (is_app_valid == 0)
    {
        // 情况 B: APP 损坏 (可能是上次擦除中断了) -> 强制留在 Bootloader
        // 可以加个 LED 快闪提示
    }
    else
    {
        // 情况 C: 正常启动且 APP 完好 -> 跳转运行 APP
        Jump_To_App();
    }

    // ===========================================================
    // 阶段二：进入升级模式 (循环接收指令)
    // ===========================================================
    
    // 发送 Ready 信号 (0x55) 告诉树莓派：Bootloader 已经就绪
    uint8_t ready = 0x55;
    HAL_UART_Transmit(&huart1, &ready, 1, 100);

    while (1)
    {
        // 阻塞接收帧头 [AA 55 LEN CMD]
        if (HAL_UART_Receive(&huart1, RxBuff, 4, 1000) == HAL_OK) 
        {
            if (RxBuff[0] == 0xAA && RxBuff[1] == 0x55)
            {
                uint8_t len = RxBuff[2];
                // 接收剩余数据: Payload + Checksum
                // 注意：len 包含了 cmd 的 1 个字节，所以剩余需读取 len (payload+cmd) - 1 (cmd已读) + 1 (checksum) = len
                // 修正逻辑：RxBuff[3] 已经是 CMD 了，上一句读了 4 个字节 (0,1,2,3)。
                // 协议定义 LEN = CMD(1) + PAYLOAD_LEN。
                // 所以剩余数据长度 = (LEN - 1) + 1(Checksum) = LEN。
                // 也就是还要读 RxBuff[4] 开始的 len 个字节
                
                if (HAL_UART_Receive(&huart1, &RxBuff[4], len, 1000) == HAL_OK)
                {
                    Process_Command();
                }
            }
        }
    }
}

// 跳转到 APP 核心函数 (已修复)
void Jump_To_App(void)
{
    uint32_t JumpAddress;
    pFunction JumpToApplication;

    // 1. 关闭所有外设 (非常重要，否则 APP 初始化时可能冲突死机)
    HAL_UART_DeInit(&huart1);
    HAL_RCC_DeInit();
    
    // 2. 关闭全局中断
    __disable_irq();
    
    // 3. 关闭并清除 SysTick (防止跳转后 SysTick 中断还在跑)
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // 4. 获取跳转地址 (Reset_Handler 的地址存放在 APP 基地址 + 4 的位置)
    JumpAddress = *(__IO uint32_t*) (APP_ADDR + 4);
    JumpToApplication = (pFunction) JumpAddress;

    // 5. 设置主堆栈指针 (MSP)
    __set_MSP(*(__IO uint32_t*) APP_ADDR);

    // 6. 跳转
    JumpToApplication();
}

void Process_Command(void)
{
    uint8_t len = RxBuff[2]; // 这个 len 是 CMD(1) + Payload
    uint8_t cmd = RxBuff[3];
    uint8_t *data = &RxBuff[4];
    uint8_t checksum_recv = RxBuff[4 + len - 1]; 
    
    // 1. 计算校验和
    uint8_t sum = 0;
    // 校验范围：[AA 55 LEN CMD ... PAYLOAD]
    // 现在的 RxBuff 包含 [AA 55 LEN CMD(下标3) DATA(下标4...)]
    for(int i=0; i < 4 + len - 1; i++) sum += RxBuff[i];
    
    if (sum != checksum_recv) {
        Send_Ack(cmd, 0xFF); // 校验错误
        return;
    }

    // 2. 执行命令
    switch (cmd)
    {
        case 0x10: // Jump to Bootloader (树莓派握手用)
            // 已经在 Bootloader 了，直接回 ACK
            Send_Ack(cmd, 0x00);
            break;

        case 0x20: // 擦除
            if (Flash_Erase_App()) Send_Ack(cmd, 0x00);
            else Send_Ack(cmd, 0x01);
            break;

        case 0x21: // 写入
            {
                // 解析 Offset (大端/小端要看 Python 发送顺序，这里假设 Python 代码是 Big-Endian)
                uint32_t offset = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
                uint16_t data_len = len - 1 - 4; // 总长 - CMD(1) - Offset(4)
                
                if (Flash_Write_App(offset, &data[4], data_len)) Send_Ack(cmd, 0x00);
                else Send_Ack(cmd, 0x01);
            }
            break;

        case 0x30: // 重启 (跳转到 APP)
            Send_Ack(cmd, 0x00);
            HAL_Delay(10); // 等数据发完
            NVIC_SystemReset(); // 软重启，让 Bootloader 重新判断是否跳转
            break;
            
        default:
            Send_Ack(cmd, 0xFE); 
            break;
    }
}

void Send_Ack(uint8_t cmd, uint8_t status)
{
    uint8_t packet[5] = {0xAA, 0x55, 0x02, cmd, status};
    HAL_UART_Transmit(&huart1, packet, 5, 100);
}

// 擦除函数 (已修正扇区范围)
uint8_t Flash_Erase_App(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;
    
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    
    // === 关键修改 ===
    // APP 从 Sector 2 (0x08008000) 开始
    EraseInitStruct.Sector        = FLASH_SECTOR_2; 
    
    // 擦除 Sector 2, 3, 4, 5, 6 (共5个扇区)
    // 绝对不要擦除 Sector 7 (参数区) !!
    EraseInitStruct.NbSectors     = 5;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0; // 失败
    }

    HAL_FLASH_Lock();
    return 1; // 成功
}

// 写入函数 (已修复指针对齐问题)
uint8_t Flash_Write_App(uint32_t offset, uint8_t *data, uint16_t len)
{
    uint32_t start_addr = APP_ADDR + offset;
    HAL_FLASH_Unlock();

    for (uint16_t i = 0; i < len; i += 4)
    {
        // === 关键修改：安全地构建 32 位字 ===
        // 避免直接强制转换 *(uint32_t*)&data[i] 导致的 HardFault (非对齐访问)
        uint32_t word = 0;
        
        // 假设是小端模式写入 (Little Endian)，与 Python 脚本发的数据顺序对应
        word |= (uint32_t)data[i];
        if (i+1 < len) word |= (uint32_t)data[i+1] << 8;
        if (i+2 < len) word |= (uint32_t)data[i+2] << 16;
        if (i+3 < len) word |= (uint32_t)data[i+3] << 24;
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0; // 失败
        }
    }

    HAL_FLASH_Lock();
    return 1; // 成功
}