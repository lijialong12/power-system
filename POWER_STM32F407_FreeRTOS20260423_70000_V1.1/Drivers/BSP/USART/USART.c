/**
 ****************************************************************************************************
 * @file        USART.c
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-03-21
 * @brief       串口驱动层
 ****************************************************************************************************
 * 修改说明
 * 
 *
 ****************************************************************************************************
 */
#include "./BSP/USART/USART.h"
#include "stdio.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "./SYSTEM/delay/delay.h"
/* USER CODE BEGIN 0 */
/* 如果使用os,则包括下面的头文件即可 */
#if SYS_SUPPORT_OS
#include "FreeRTOS.h"                               /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif

/***********************************声明变量***************************************************/

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart6_rx;



char        UsartRxData4[USARTLEN_4];  	
uint16_t    UsartRxLen4 = 0;    				
uint8_t     UsartTxflag4 = 0;    				

char        UsartRxData5[USARTLEN_5];  
uint16_t    UsartRxLen5 = 0;    
uint8_t     UsartTxflag5 = 0;   


char        UsartRxData1[USARTLEN_1];  			//接收缓冲区
uint16_t    UsartRxLen1 = 0;    				//接收缓冲区数据长度
uint8_t     UsartTxflag1 = 0;    				//接收缓冲区数据长度


char        UsartRxData2[USARTLEN_2];  	
uint16_t    UsartRxLen2 = 0;    				
uint8_t     UsartTxflag2 = 0;    				

char        UsartRxData3[USARTLEN_3];  
uint16_t    UsartRxLen3 = 0;    
uint8_t     UsartTxflag3 = 0;    


char        UsartRxData6[USARTLEN_6];  
uint16_t    UsartRxLen6 = 0;    
uint8_t     UsartTxflag6 = 0;    
/************************** USART1 init function ***********************************************/

void Usart1_Dma2_Init(uint32_t baudrate)
{
    /* USER CODE BEGIN USART1_Init 0 */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 14, 14);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 13, 13);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    /* USER CODE END USART1_Init 1 */

    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
    //    Error_Handler();
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能串UART1 IDLE中断
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)UsartRxData1, USARTLEN_1);//开启DMA接收模式
}

void Usart2_Dma1_Init(uint32_t baudrate)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
    
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = baudrate;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//使能串UART1 IDLE中断
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)UsartRxData2, USARTLEN_2);//开启DMA接收模式
  /* USER CODE END USART2_Init 2 */

}


void Usart3_Dma1_Init(uint32_t baudrate)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
	  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
  huart3.Instance = USART3;
  huart3.Init.BaudRate = baudrate;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
   // Error_Handler();
  }
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//使能串UART1 IDLE中断
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)UsartRxData3, USARTLEN_3);//开启DMA接收模式
}


void Uart4_Dma1_Init(uint32_t baudrate)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	
  huart4.Instance = UART4;
  huart4.Init.BaudRate = baudrate;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    //Error_Handler();
  }
    /* USER CODE BEGIN UART4_MspInit 1 */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);//使能串UART4 IDLE中断
    HAL_UART_Receive_DMA(&huart4, (uint8_t *)UsartRxData4, USARTLEN_4);//开启DMA接收模式
}


void Uart5_Dma1_Init(uint32_t baudrate)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  huart5.Instance = UART5;
  huart5.Init.BaudRate = baudrate;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    //Error_Handler();
  }
  
    /* USER CODE BEGIN UART4_MspInit 1 */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);//使能串UART5 IDLE中断
    HAL_UART_Receive_DMA(&huart5, (uint8_t *)UsartRxData5, USARTLEN_5);//开启DMA接收模式
}


/**
  * @brief  可配置参数的USART6 DMA初始化函数
  * @param  BaudRate: 波特率（如9600、115200、921600等）
  * @param  StopBits: 停止位配置（UART_STOPBITS_1 / UART_STOPBITS_0_5 / UART_STOPBITS_2 / UART_STOPBITS_1_5）
  * @param  Parity: 奇偶校验配置（UART_PARITY_NONE / UART_PARITY_EVEN / UART_PARITY_ODD）
  * @note   固定配置：8位数据位、无硬件流控、16倍过采样、DMA2_Stream1_Channel5
  * @retval None
  */
void Usart6_Dma2_Init(uint32_t BaudRate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* -------------------------- 第一步：复位原有配置（清理旧状态） -------------------------- */
    // 关闭USART6时钟
    __HAL_RCC_USART6_CLK_DISABLE();
    // 复位GPIO引脚
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
    // 复位DMA
    if(hdma_usart6_rx.Instance != NULL)
    {
        HAL_DMA_DeInit(&hdma_usart6_rx);
    }
    // 关闭USART6中断
    HAL_NVIC_DisableIRQ(USART6_IRQn);

    /* -------------------------- 第二步：使能时钟（GPIO/USART6/DMA） -------------------------- */
    // 使能GPIOC时钟（PC6/PC7）
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // 使能USART6时钟
    __HAL_RCC_USART6_CLK_ENABLE();
    // 使能DMA2时钟（DMA2_Stream1依赖）
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* -------------------------- 第三步：配置USART6 GPIO（PC6=TX，PC7=RX） -------------------------- */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 无上下拉（可根据需求改为GPIO_PULLUP）
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 超高速
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;  // 复用映射到USART6
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* -------------------------- 第四步：配置USART6核心参数（支持动态入参） -------------------------- */
    huart6.Instance = USART6;
    huart6.Init.BaudRate = BaudRate;              // 动态波特率
    huart6.Init.WordLength = UART_WORDLENGTH_8B;  // 固定8位数据位（如需可改为入参）
    huart6.Init.StopBits = UART_STOPBITS_1;              // 动态停止位
    huart6.Init.Parity = UART_PARITY_NONE;                  // 动态奇偶校验
    huart6.Init.Mode = UART_MODE_TX_RX;           // 固定收发模式
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // 固定无硬件流控
    huart6.Init.OverSampling = UART_OVERSAMPLING_16; // 固定16倍过采样
	
    // 初始化UART
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
    }

    /* -------------------------- 第五步：配置USART6 RX的DMA（DMA2_Stream1） -------------------------- */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;  // USART6_RX对应DMA2_Stream1_Channel5
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; // 外设→内存（UART接收）
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;    // 外设地址不递增（UART_DR固定）
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;        // 内存地址递增（缓冲区）
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // 字节对齐
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;                // 正常模式（需手动重启）
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;      // 低优先级
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  // 关闭FIFO
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
    }

    // 绑定DMA句柄到UART句柄
    __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);

    /* -------------------------- 第六步：配置USART6中断（IDLE+NVIC） -------------------------- */
    // 使能USART6的IDLE中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // 配置USART6中断优先级并使能
    HAL_NVIC_SetPriority(USART6_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    /* -------------------------- 第七步：启动DMA接收 -------------------------- */
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)UsartRxData6, USARTLEN_6);
}





void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance==USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /*
        PA9     ------> USART1_TX
        PA10    ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*********************************** USART1 DMA Init *************************************/
        
        /* USART1_RX Init */
        hdma_usart1_rx.Instance = DMA2_Stream2;
        hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode = DMA_NORMAL;
        hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        {
        //      Error_Handler();
        }
        __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

        /* USART1_TX Init */
        hdma_usart1_tx.Instance = DMA2_Stream7;
        hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
        {
        //      Error_Handler();
        }
        __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 11, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);

    }
    
    if(uartHandle->Instance==USART2)
    {
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    }
		
	if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
     // Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA1_Stream4;
    hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      //Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE END UART4_MspInit 1 */
  }
  if(uartHandle->Instance==UART5)
  {
	  /* USER CODE BEGIN UART5_MspInit 0 */

	  /* USER CODE END UART5_MspInit 0 */
		/* UART5 clock enable */
		__HAL_RCC_UART5_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**UART5 GPIO Configuration
		PC12     ------> UART5_TX
		PD2     ------> UART5_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* UART5 DMA Init */
		/* UART5_RX Init */
		hdma_uart5_rx.Instance = DMA1_Stream0;
		hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart5_rx.Init.Mode = DMA_NORMAL;
		hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
		{
		  //Error_Handler();
		}

		__HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

		/* UART5_TX Init */
		hdma_uart5_tx.Instance = DMA1_Stream7;
		hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart5_tx.Init.Mode = DMA_NORMAL;
		hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
		{
		  //Error_Handler();
		}

		__HAL_LINKDMA(uartHandle,hdmatx,hdma_uart5_tx);

		/* UART5 interrupt Init */
		HAL_NVIC_SetPriority(UART5_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(UART5_IRQn);
	  /* USER CODE BEGIN UART5_MspInit 1 */

	  /* USER CODE END UART5_MspInit 1 */
	}
	
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag1 = 1;
    }    
    if (huart->Instance == USART2) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag2 = 1;
    }
	if (huart->Instance == USART3) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag3 = 1;
	}
		
	if (huart->Instance == UART4) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag4 = 1;
			//printf("发送成功！！\r\n");
    }
	if (huart->Instance == UART5) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag5 = 1;
			//printf("发送成功！！\r\n");
    }
	if (huart->Instance == USART6) // 确保是USART2的回调
    {
        // 传输完成后的处理逻辑
        UsartTxflag6 = 1;
		//printf("UsartTxflag6发送成功！！\r\n");
    }
}


/*
*  功能：复制串口缓冲区数据到目标数组
*  参数：
*   usart：串口索引 {1 2 3 4 5 6 7}
*   dest： 目标数组 {字符串}
*   maxLen 拷贝的限制长度
*
*/
void CopySerialData(uint8_t usart, char *dest, int maxLen)
{
    
    switch(usart)
    {
        case 1:       
                    {  int lenToCopy = (UsartRxLen1 < maxLen) ? USARTLEN_1 : maxLen;
                        memcpy(dest, UsartRxData1, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
                        if (lenToCopy < maxLen)
                        {
                            dest[lenToCopy] = ' ';
                        }
                    }
				    break;
        case 2:    
                    {  int lenToCopy = (UsartRxLen2 < maxLen) ? USARTLEN_2 : maxLen;
                        memcpy(dest, UsartRxData2, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
		
                        if (lenToCopy < maxLen)
                        {
							dest[lenToCopy] = ' ';
                        }
                    }
                      break;
        case 3:    
                    {  int lenToCopy = (UsartRxLen3 < maxLen) ? USARTLEN_3 : maxLen;
                        memcpy(dest, UsartRxData3, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
                        if (lenToCopy < maxLen)
                        {
							dest[lenToCopy] = ' ';
                        }
                    }
                      break;
					
        case 4:    
                    {  int lenToCopy = (UsartRxLen4 < maxLen) ? USARTLEN_4 : maxLen;
                        memcpy(dest, UsartRxData4, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
                        if (lenToCopy < maxLen)
                        {
							dest[lenToCopy] = ' ';
                        }
                    }
                      break;	
        case 5:    
                    {  int lenToCopy = (UsartRxLen5 < maxLen) ? USARTLEN_5 : maxLen;
                        memcpy(dest, UsartRxData5, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
                        if (lenToCopy < maxLen)
                        {
							dest[lenToCopy] = ' ';
                        }
                    }
                      break;	
										
		case 6:    
                    {  int lenToCopy = (UsartRxLen6 < maxLen) ? USARTLEN_6 : maxLen;
                        memcpy(dest, UsartRxData6, lenToCopy);
                        // 如果目标数组比缓冲区小，则在目标数组末尾添加空字符（确保字符串终止）
                        if (lenToCopy < maxLen)
                        {
							dest[lenToCopy] = ' ';
                        }
                    }
                      break;
		default: break;
    }
}

/*
*  功能：串口1发送函数
*  参数：
*   dest：发送目标字符串
*/
void Usart_Transmit(uint8_t usart, char *dest,uint16_t len)
{
	switch(usart)
    {
        case 1:  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dest, len); break;
        case 2:  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dest, len); break;
		case 3:  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)dest, len); while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET){vTaskDelay(1);}
				 delay_us(20);  //清空接收缓冲区，扔掉刚才产生的所有假数据毛刺！
				__HAL_UART_FLUSH_DRREGISTER(&huart3); // 清除RXNE标志
				 break;
		case 4:  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)dest, len); break;
		case 5:  HAL_UART_Transmit_DMA(&huart5, (uint8_t *)dest, len); while(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET){vTaskDelay(1);}
				 delay_us(20);  //清空接收缓冲区，扔掉刚才产生的所有假数据毛刺！
				__HAL_UART_FLUSH_DRREGISTER(&huart5); // 清除RXNE标志
				 break;
		         // 阻塞发送完成后，手动执行回调逻辑
        case 6:  if(HAL_UART_Transmit(&huart6, (uint8_t *)dest, len, 5) == HAL_OK)
				{
					UsartTxflag6 = 1; // 手动置位标志
				}
				break;
		default: break;
	}
}


/*
*  功能：清除串口缓冲区数据
*  参数：
*   usart：串口索引 1 2 3 4 5 6
*/
void ClearSerialBuffer(uint8_t usart) 
{
    switch(usart)
    {
        case 1: 
								{
										memset(UsartRxData1, 0, USARTLEN_1);
										UsartRxLen1 = 0;
										HAL_UART_Receive_DMA(&huart1, (uint8_t *)UsartRxData1, USARTLEN_1); //开启DMA数据接收
								}
								break;
        case 2:  
								{
										memset(UsartRxData2, 0, USARTLEN_2);
										UsartRxLen2 = 0;
										HAL_UART_Receive_DMA(&huart2, (uint8_t *)UsartRxData2, USARTLEN_2); //开启DMA数据接收
								}	
								break;	
        case 3:  
								{
										memset(UsartRxData3, 0, USARTLEN_3);
										UsartRxLen3 = 0;
										HAL_UART_Receive_DMA(&huart3, (uint8_t *)UsartRxData3, USARTLEN_3); //开启DMA数据接收
								}	
								break;		
        case 4:  
								{
										memset(UsartRxData4, 0, USARTLEN_4);
										UsartRxLen4 = 0;
										HAL_UART_Receive_DMA(&huart4, (uint8_t *)UsartRxData4, USARTLEN_4); //开启DMA数据接收
								}	
								break;	
        case 5:  
								{
										memset(UsartRxData5, 0, USARTLEN_5);
										UsartRxLen5 = 0;
										HAL_UART_Receive_DMA(&huart5, (uint8_t *)UsartRxData5, USARTLEN_5); //开启DMA数据接收
								}	
								break;									
        case 6:  
								{
										memset(UsartRxData6, 0, USARTLEN_6);
										UsartRxLen6 = 0;
										HAL_UART_Receive_DMA(&huart6, (uint8_t *)UsartRxData6, USARTLEN_6); //开启DMA数据接收
								}	
								break;	
		default: break;						
    }
}
 /************************************************************串口1中断************************************************************/
//串口1中断回调函数
void USART1_IRQHandler(void)
{
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
    #endif

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart1); //清除IDLE标志
		
		 HAL_UART_DMAStop(&huart1);//停止DMA，为了重新设置DMA发送多少数据
		
        UsartRxLen1 = USARTLEN_1 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
         
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)UsartRxData1, USARTLEN_1); //开启DMA数据接收
       
    }
    
    HAL_UART_IRQHandler(&huart1);
    
    #if SYS_SUPPORT_OS                              /* 使用OS */
		taskEXIT_CRITICAL_FROM_ISR(status_value);
    #endif
}


//DMA中断回调函数
void DMA2_Stream2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart1_rx);

}

//DMA中断回调函数
void DMA2_Stream7_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart1_tx);

}

 /************************************************************串口2中断************************************************************/
void USART2_IRQHandler(void)
{
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
    #endif
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart2); //清除IDLE标志
		
		HAL_UART_DMAStop(&huart2);//停止DMA，为了重新设置DMA发送多少数据
        
        UsartRxLen2 = USARTLEN_2 - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);	
		
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)UsartRxData2, USARTLEN_2); //开启DMA数据接收

    }

	HAL_UART_IRQHandler(&huart2);
	
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif

}


void DMA1_Stream5_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart2_rx);

}

void DMA1_Stream6_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

 /************************************************************串口3中断************************************************************/
void USART3_IRQHandler(void)
{
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
    #endif
    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart3); //清除IDLE标志、
		
		HAL_UART_DMAStop(&huart3);//停止DMA，为了重新设置DMA发送多少数据
        
        UsartRxLen3 = USARTLEN_3 - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)UsartRxData3, USARTLEN_3); //开启DMA数据接收
    }
		
	HAL_UART_IRQHandler(&huart3);
	
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif

}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}


void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

 /************************************************************串口6中断************************************************************/

void USART6_IRQHandler(void)
{
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();
    #endif

    // 关键4：先调用HAL库默认中断处理，再处理IDLE（修复顺序错误）
    HAL_UART_IRQHandler(&huart6);

    // 处理IDLE中断
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart6); // 清除IDLE标志
         HAL_UART_DMAStop(&huart6);          // 停止DMA

         // 计算实际接收长度（核心：此时NDTR是剩余计数，差值为已接收长度）
         UsartRxLen6 = USARTLEN_6 - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

         // 重启DMA接收（准备下一次数据）
         HAL_UART_Receive_DMA(&huart6, (uint8_t *)UsartRxData6, USARTLEN_6);
    }
	
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}



/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
	
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();
    #endif
  /* USER CODE BEGIN UART4_IRQn 0 */
    if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart4); //清除IDLE标志
		
		HAL_UART_DMAStop(&huart4);//停止DMA，为了重新设置DMA发送多少数据
        
        UsartRxLen4 = USARTLEN_4 - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
         
//		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)UsartRxData4, UsartRxLen4);
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)UsartRxData4, USARTLEN_4); //开启DMA数据接收
			
    }
	/* USER CODE END UART4_IRQn 0 */
	HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    #if SYS_SUPPORT_OS                              /* 使用OS */
     uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
    #endif
  /* USER CODE BEGIN UART5_IRQn 0 */
    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) != RESET)
    {
         __HAL_UART_CLEAR_IDLEFLAG(&huart5); //清除IDLE标志
		
		HAL_UART_DMAStop(&huart5);//停止DMA，为了重新设置DMA发送多少数据
        
        UsartRxLen5 = USARTLEN_5 - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
		
		HAL_UART_Receive_DMA(&huart5, (uint8_t *)UsartRxData5, USARTLEN_5); //开启DMA数据接收
				
    }
  /* USER CODE END UART5_IRQn 0 */
	HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif
  /* USER CODE END UART5_IRQn 1 */
}


