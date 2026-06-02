#ifndef __HANDLE_1_TASK_H
#define __HANDLE_1_TASK_H

#include "./SYSTEM/sys/sys.h"
#include "includes.h"




#define 	Debug_Printf(...)       printf(__VA_ARGS__)



#define  	PC_FAIL 				1		//失败
#define     PC_SUCC					0		//成功
#define 	PC_LEN					255		//缓冲区长度



//手柄2通讯串口
#define		HANDLE_1Usart						3
#define     HANDLE_1Usart_Transmit				Usart_Transmit
#define     HANDLE_1UsartRxLen   				UsartRxLen3	
#define     HANDLE_1CopySerialData   			CopySerialData	
#define     HANDLE_1ClearSerialBuffer   		ClearSerialBuffer
#define     HANDLE_1UsartTxflag					UsartTxflag3
#define		HANDLE_1Usart_Init					Usart3_Dma1_Init




typedef struct{

uint8_t  	Link;			//通讯连接	


} HANDE_1DATA;


/* 存储一对整数的结构 */
typedef struct {
    int gyrosvalue1;
    int pressvalue2;
} AthPair;


extern AthPair pairsval;
extern HANDE_1DATA	handle_1data;



void handle_1_task_init(void);


#endif


