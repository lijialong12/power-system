#ifndef __PC_TASK_H
#define __PC_TASK_H

#include "./SYSTEM/sys/sys.h"
#include "includes.h"




#define 	Debug_Printf(...)       printf(__VA_ARGS__)



#define  	PC_FAIL 				1		//失败
#define     PC_SUCC					0		//成功
#define 	PC_LEN					255		//缓冲区长度



//PC通讯串口
#define		PCUsart						3
#define     PCUsart_Transmit			Usart_Transmit
#define     PCUsartRxLen   				UsartRxLen3	
#define     PCCopySerialData   			CopySerialData	
#define     PCClearSerialBuffer   		ClearSerialBuffer
#define     PCUsartTxflag				UsartTxflag3
#define		PCUsart_Init				Usart3_Dma1_Init


typedef struct{

uint8_t  	Link;			//通讯连接	


} PCDATA;



extern PCDATA	pcdata;



void pc_task_init(void);


#endif


