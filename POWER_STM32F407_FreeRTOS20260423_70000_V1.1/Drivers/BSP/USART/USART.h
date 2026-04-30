/**
 ****************************************************************************************************
 * @file        USART.h
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
#ifndef _USART_H
#define _USART_H

#include "./SYSTEM/sys/sys.h"


#define         USARTLEN_1            255      //串口DMA缓冲最大接收长度
#define         USARTLEN_2            255      //串口DMA缓冲最大接收长度
#define         USARTLEN_3            255      //串口DMA缓冲最大接收长度
#define         USARTLEN_4            255      //串口DMA缓冲最大接收长度
#define         USARTLEN_5            255      //串口DMA缓冲最大接收长度
#define         USARTLEN_6            255      //串口DMA缓冲最大接收长度

/******************** 便于外部调用变量********************************/

//串口1
extern uint16_t UsartRxLen1;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag1;   //发送完成标志位

//串口2
extern uint16_t UsartRxLen2;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag2;   //发送完成标志位

//串口3
extern uint16_t UsartRxLen3;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag3;   //发送完成标志位

//串口4
extern uint16_t UsartRxLen4;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag4;   //发送完成标志位

//串口5
extern uint16_t UsartRxLen5;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag5;   //发送完成标志位

//串口6
extern uint16_t UsartRxLen6;    //接收缓冲区数据长度
extern uint8_t  UsartTxflag6;   //发送完成标志位

/* 初始化 */
extern void Usart1_Dma2_Init(uint32_t baudrate);
extern void Usart2_Dma1_Init(uint32_t baudrate);
extern void Usart3_Dma1_Init(uint32_t baudrate);
extern void Uart4_Dma1_Init(uint32_t baudrate);
extern void Uart5_Dma1_Init(uint32_t baudrate);
extern void Usart6_Dma2_Init(uint32_t baudrate);


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;


//串口发送函数
extern void Usart_Transmit(uint8_t usart, char *dest,uint16_t len);
// 复制串口缓冲区数据到目标数组
extern void CopySerialData(uint8_t usart, char *dest, int maxLen);
// 清除串口缓冲区数据
extern void ClearSerialBuffer(uint8_t usart);


#endif


