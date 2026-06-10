#ifndef __HANDLE_2_TASK_H
#define __HANDLE_2_TASK_H

#include "./SYSTEM/sys/sys.h"




#define 	Debug_Printf(...)       printf(__VA_ARGS__)



#define  	HANLDE_FAIL 				1		//失败
#define     HANLDE_SUCC					0		//成功
#define 	HANLDE_LEN					255		//缓冲区长度



//电机控制串口
#define		HandleUsart						5
#define     HandleUsart_Transmit			Usart_Transmit
#define     HandleUsartRxLen   				UsartRxLen5	
#define     HandleCopySerialData   			CopySerialData	
#define     HandleClearSerialBuffer   		ClearSerialBuffer
#define     HandleUsartTxflag				UsartTxflag5
#define		HandleUsart_Init				Uart5_Dma1_Init


typedef struct{

uint8_t  	Link1;						//0 手柄1未连接  1 手柄1连接	
uint8_t  	Link2;						//0 手柄2未连接  1 手柄2连接	
uint8_t  	Link1_Numb;					//0 手柄1接口没有手柄  手柄1接口手柄1  手柄1接口手柄2 
uint8_t  	Link2_Numb;					//0 手柄2接口没有手柄  手柄2接口手柄1  手柄2接口手柄2 
uint8_t  	hande1_send_interval;		//常规发送间隔 hande1_send_interval	ms
uint8_t  	Rerror1;					//连续Rerror1次失败，标记离线	
uint8_t  	hande2_send_interval;		//常规发送间隔：hande2_send_interval ms	
uint8_t	 	Rerror2;					//连续Rerror2次失败，标记离线
} HANDLE;


/* 存储一对整数的结构 */
typedef struct {
    int gyrosvalue1;
    int pressvalue2;
} AthPair;


extern AthPair pairsval;
extern HANDLE	handle;


void handle_2_task_init(void);

#endif








