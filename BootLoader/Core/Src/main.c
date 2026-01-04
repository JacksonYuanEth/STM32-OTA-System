/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>  // 必须包含，用于内存操作
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 函数指针定义，用于跳转
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ================= 配置区 =================
// 1. APP 起始地址 (32KB Bootloader 之后，即 Sector 2)
#define APP_ADDR  0x08008000

// 2. 标志位地址 (SRAM 末尾)
#define BOOT_FLAG_ADDR  0x2001FFF0 
#define BOOT_FLAG_MAGIC 0xDEADBEEF

// 3. 串口接收缓冲区大小
#define RX_BUFF_SIZE 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 定义接收缓冲区
uint8_t RxBuff[RX_BUFF_SIZE];
IWDG_HandleTypeDef hiwdg; // 看门狗句柄
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 声明自定义函数
void Jump_To_App(void);
void Process_Command(void);
void Send_Ack(uint8_t cmd, uint8_t status);
uint8_t Flash_Erase_App(void);
uint8_t Flash_Write_App(uint32_t offset, uint8_t *data, uint16_t len);
void MX_IWDG_Init(void); //看门狗初始化函数声明
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // ===========================================================
  // 阶段一：启动检查与救砖逻辑
  // ===========================================================
  
  // 1. 给树莓派留出 "抢占窗口"
  // 延时 200ms，防止 APP 死机后无法进入 Bootloader
  HAL_Delay(200); 

  // 2. 检查标志位 (看看是不是 APP 主动请求升级)
  uint32_t flag = *(__IO uint32_t *)BOOT_FLAG_ADDR;
  
  // 3. 检查 APP 是否有效 (救砖核心)
  // 读取 APP 地址的第一个字 (栈顶指针 MSP)
  uint32_t app_stack = *(__IO uint32_t*)APP_ADDR;
  uint8_t is_app_valid = 0;
  
  // 简单的合法性检查：栈顶指针必须在 SRAM 范围内 (0x2000xxxx)
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
      // 情况 B: APP 损坏 -> 强制留在 Bootloader
  }
  else
  {
      // 情况 C: 正常启动且 APP 完好 -> 跳转运行 APP
      Jump_To_App();
  }

  // ===========================================================
  // 阶段二：进入升级模式
  // ===========================================================
  
  // 发送 Ready 信号 (0x55) 告诉树莓派：Bootloader 已经就绪
  // 注意：这里已经改为 &huart2
  uint8_t ready = 0x55;
  HAL_UART_Transmit(&huart2, &ready, 1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // 喂狗
    HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE BEGIN 3 */
    // 阻塞接收帧头 [AA 55 LEN CMD]
    // 注意：这里已经改为 &huart2
    if (HAL_UART_Receive(&huart2, RxBuff, 4, 1000) == HAL_OK) 
    {
        if (RxBuff[0] == 0xAA && RxBuff[1] == 0x55)
        {
            uint8_t len = RxBuff[2];
            // 接收剩余数据: Payload + Checksum
            // 这里的 len = CMD(1) + Payload长度
            if (HAL_UART_Receive(&huart2, &RxBuff[4], len, 1000) == HAL_OK)
            {
                Process_Command();
            }
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // 注意：此处为您生成的16MHz配置
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 跳转到 APP 核心函数
void Jump_To_App(void)
{
    uint32_t JumpAddress;
    pFunction JumpToApplication;

    // 1. 关闭所有外设 (注意：改为 huart2)
    HAL_UART_DeInit(&huart2); 
    HAL_RCC_DeInit();
    
    // 2. 关闭全局中断
    __disable_irq();
    
    // 3. 关闭并清除 SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // 4. 获取跳转地址 (Reset_Handler)
    JumpAddress = *(__IO uint32_t*) (APP_ADDR + 4);
    JumpToApplication = (pFunction) JumpAddress;

    // 5. 设置主堆栈指针 (MSP)
    __set_MSP(*(__IO uint32_t*) APP_ADDR);

    // 6. 跳转
    JumpToApplication();
}

void Process_Command(void)
{
    uint8_t len = RxBuff[2]; 
    uint8_t cmd = RxBuff[3];
    uint8_t *data = &RxBuff[4];
    uint8_t checksum_recv = RxBuff[4 + len - 1]; 
    
    // 1. 计算校验和
    uint8_t sum = 0;
    for(int i=0; i < 4 + len - 1; i++) sum += RxBuff[i];
    
    if (sum != checksum_recv) {
        Send_Ack(cmd, 0xFF); // 校验错误
        return;
    }

    // 2. 执行命令
    switch (cmd)
    {
        case 0x10: // Jump to Bootloader
            Send_Ack(cmd, 0x00);
            break;

        case 0x20: // 擦除
            if (Flash_Erase_App()) Send_Ack(cmd, 0x00);
            else Send_Ack(cmd, 0x01);
            break;

        case 0x21: // 写入
            {
                // 解析 Offset (大端接收)
                uint32_t offset = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
                uint16_t data_len = len - 1 - 4; 
                
                if (Flash_Write_App(offset, &data[4], data_len)) Send_Ack(cmd, 0x00);
                else Send_Ack(cmd, 0x01);
            }
            break;

        case 0x30: // 重启
            Send_Ack(cmd, 0x00);
            HAL_Delay(10); 
            NVIC_SystemReset();
            break;  
            
        default:
            Send_Ack(cmd, 0xFE); 
            break;
    }
}

void Send_Ack(uint8_t cmd, uint8_t status)
{
    uint8_t packet[5] = {0xAA, 0x55, 0x02, cmd, status};
    // 注意：改为 huart2
    HAL_UART_Transmit(&huart2, packet, 5, 100);
}

// ==========================================================
// 最终修正版：写入函数 (带错误标志位清除)
// ==========================================================
uint8_t Flash_Write_App(uint32_t offset, uint8_t *data, uint16_t len)
{
    uint32_t start_addr = APP_ADDR + offset;
    
    HAL_FLASH_Unlock();

    // 1. 【核心修复】写入前必须清除所有潜在的错误标志
    // 如果不加这一行，只要之前发生过任何错误，写入就会立刻失败
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (uint16_t i = 0; i < len; i += 4)
    {
        // 2. 组合 4 字节数据 (Little Endian)
        uint32_t word = 0;
        if (i < len) word |= (uint32_t)data[i];
        if (i+1 < len) word |= (uint32_t)data[i+1] << 8;
        if (i+2 < len) word |= (uint32_t)data[i+2] << 16;
        if (i+3 < len) word |= (uint32_t)data[i+3] << 24;
        
        // 3. 执行写入 (以 Word 为单位)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, word) != HAL_OK)
        {
            // 如果失败，尝试再次清除标志位并上锁
             __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
            HAL_FLASH_Lock();
            return 0; // 返回失败给 Python
        }
    }

    HAL_FLASH_Lock();
    return 1; // 成功
}


// 擦除函数 (已添加抗干扰处理)
uint8_t Flash_Erase_App(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;
    
    HAL_FLASH_Unlock();

    // ==========================================================
    // 【核心修复】擦除前必须清除所有潜在的错误标志
    // 否则如果上一次操作失败，这次会直接卡死或报错
    // ==========================================================
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = FLASH_SECTOR_2; // 起始扇区
    EraseInitStruct.NbSectors     = 5;              // 扇区数量 (2,3,4,5,6)

    // 喂狗，防止擦除过程中复位
    HAL_IWDG_Refresh(&hiwdg);

    // 执行擦除 (STM32F4擦除这几个扇区可能需要 1~3 秒，请确保Python没有超时)
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        // 再次尝试清除标志位，防止锁死
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        HAL_FLASH_Lock();
        return 0; // 返回失败
    }

    HAL_FLASH_Lock();
    return 1; // 成功
}

//STM32F446的LSI频率是17KHz->47KHz
//设置看门狗时间为4.08s->11.29s,最差情况的4s也远大于Flash烧写的2s
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;  // 64分频
  hiwdg.Init.Reload = 3000;                  // 重装载值
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    // 如果初始化失败，说明硬件有问题，死循环报错
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
