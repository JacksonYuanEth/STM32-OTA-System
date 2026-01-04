/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "spi_slave.h"
#include "can_data.h"
#include "string.h" 
#include "usart.h"  


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 外部变量声明 */
extern osMessageQueueId_t can1QueueHandle;
extern osMessageQueueId_t can2QueueHandle;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_TxMessage_t   CAN1_TxMessa;
extern CAN_TxMessage_t   CAN2_TxMessa;
extern UART_HandleTypeDef huart2; // 引用 usart.c 里的句柄
CAN_Message_t can2_msg;
uint32_t TaskThreadHeartbeat[8] = {0};
uint8_t RxBuffer_OTA[300];  // 定义足够大的缓冲区，覆盖 265 字节的最大包长

// 定义一个 128 字节的接收缓冲区
uint8_t RxBuffer_USART2[128];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern  uint16_t SPI1_CsEnabled;   //SPI1的使能开关，1为接受到新数据，0为未读到新数据
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPI1Task02 */
osThreadId_t SPI1Task02Handle;
const osThreadAttr_t SPI1Task02_attributes = {
  .name = "SPI1Task02",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Can1Task03 */
osThreadId_t Can1Task03Handle;
const osThreadAttr_t Can1Task03_attributes = {
  .name = "Can1Task03",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Can2Task04 */
osThreadId_t Can2Task04Handle;
const osThreadAttr_t Can2Task04_attributes = {
  .name = "Can2Task04",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Usart2Task05 */
osThreadId_t Usart2Task05Handle;
const osThreadAttr_t Usart2Task05_attributes = {
  .name = "Usart2Task05",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DataTask06 */
osThreadId_t DataTask06Handle;
const osThreadAttr_t DataTask06_attributes = {
  .name = "DataTask06",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FlowTask07 */
osThreadId_t FlowTask07Handle;
const osThreadAttr_t FlowTask07_attributes = {
  .name = "FlowTask07",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void Check_OTA_Request(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SPI1Task02 */
  SPI1Task02Handle = osThreadNew(StartTask02, NULL, &SPI1Task02_attributes);

  /* creation of Can1Task03 */
  Can1Task03Handle = osThreadNew(StartTask03, NULL, &Can1Task03_attributes);

  /* creation of Can2Task04 */
  Can2Task04Handle = osThreadNew(StartTask04, NULL, &Can2Task04_attributes);

  /* creation of Usart2Task05 */
  Usart2Task05Handle = osThreadNew(StartTask05, NULL, &Usart2Task05_attributes);

  /* creation of DataTask06 */
  DataTask06Handle = osThreadNew(StartTask06, NULL, &DataTask06_attributes);

  /* creation of FlowTask07 */
  FlowTask07Handle = osThreadNew(StartTask07, NULL, &FlowTask07_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      osDelay(1);
	  TaskThreadHeartbeat[0] ++;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SPI1Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if(SPI1_CsEnabled == 1)                       //SPI1使能信号为1，则进入数据跟新程序
	  {
		  SPI1_CsEnabled = 0;                       //SPI1使能信号清零，则进入数据跟新程序
		  CAN_GeneratedData();                      //生成数据（测试）
          build_spi_response();                     //构建 SPI 返回帧 (电机反馈数据)
	  }
    //osDelay(1);
	  TaskThreadHeartbeat[1] ++;
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Can1Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	CAN_Message_t msg;
	CAN_Message_t can2_msg;
    uint32_t last_print_tick = 0;
  /* Infinite loop */
  for(;;)
  {
	  // 处理CAN1数据
	CANSend_DataConversion();
    CANcontrol();   //CAN发送协议生成及CAN发送
    //AND_CanSendServe(hcan1,CAN1_TxMessa);
	  // 从队列读取CAN1数据
	if(osMessageQueueGet(can1QueueHandle, &msg, NULL, 0) == osOK)
	{
	    
    }
    osDelay(100);
	TaskThreadHeartbeat[2] ++;
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Can2Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */

  // DMA测试代码
  // HAL_UART_Receive_DMA(&huart2, RxBuffer_USART2, 128);
  for(;;)
  {
    osDelay(1);
    /* DMA测试代码
    if(HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY || 
         HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_RX) 
      {
          // 参数：串口句柄，数据源指针，发送长度
          HAL_UART_Transmit_DMA(&huart2, RxBuffer_USART2, 128);
      }*/
	TaskThreadHeartbeat[3] ++;
  }

  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Usart2Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
/* 合并后的任务代码 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  
  // 1. 启动接收 (开启 IDLE 中断 + DMA)
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2, RxBuffer_OTA, 300);

  for(;;)
  {
    // 2. 等待通知，但最多等 1000ms (1秒)
    // pdMS_TO_TICKS(1000) 会自动根据你的 RTOS 时钟频率计算 Tick 数
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (ulNotificationValue > 0)
    {
        // === 情况 A：收到串口数据 (IDLE 中断触发) ===
        Check_OTA_Request();
        // 清空缓冲区 (防止脏数据干扰)
        memset(RxBuffer_OTA, 0, 300);      
        // 重新开启 DMA 接收下一包
        HAL_UART_Receive_DMA(&huart2, RxBuffer_OTA, 300);
    }
    else
    {
        // === 情况 B：超时 (1秒内无数据接收),发送心跳包 ===
        char *msg = "APP_V1.0_Running\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
    
    // 任务计数器自增 (保留原有逻辑)
    TaskThreadHeartbeat[4]++;
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the DataTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	TaskThreadHeartbeat[5] ++;
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the FlowTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	TaskThreadHeartbeat[6] ++;
  }
  /* USER CODE END StartTask07 */
}



void Check_OTA_Request(void)
{
    // 简单检查帧头 [AA 55] 和命令 [10]
    // Python 脚本里发送的是: [AA 55 LEN 0x10 ...]
    if (RxBuffer_OTA[0] == 0xAA && RxBuffer_OTA[1] == 0x55 && RxBuffer_OTA[3] == 0x10)
    {
        // 1. 发送 ACK (告诉 Python: 我收到跳转指令了)
        // 构造 ACK: AA 55 02 10 00
        uint8_t ack[] = {0xAA, 0x55, 0x02, 0x10, 0x00};
        HAL_UART_Transmit(&huart2, ack, 5, 100);
        
        // 2. 写入 OTA 标志位 (BOOT_FLAG_ADDR = 0x2001FFF0)
        *(__IO uint32_t *)0x2001FFF0 = 0xDEADBEEF;
        
        // 3. 延时确保串口发送完毕
        osDelay(100);
        
        // 4. 关闭中断并重启,防抖和保证操作的原子性
        __disable_irq();
        NVIC_SystemReset();
    }
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

