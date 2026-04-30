#ifndef __PUMP_TASK_H
#define __PUMP_TASK_H

#include "./SYSTEM/sys/sys.h"


#define  	PUMPSET				1		//真
#define     PUMPRESET			0		//假
#define 	PUMPLEN				255		//缓冲区长度

#define Debug_Printf(...)       printf(__VA_ARGS__)


//蠕动泵数据接受串口
#define		RS485PUMP_Usart					6
#define     RS485PUMP_Usart_Transmit		Usart_Transmit
#define     RS485PUMP_UsartRxLen   			UsartRxLen6	
#define     RS485PUMP_CopySerialData   		CopySerialData	
#define     RS485PUMP_ClearSerialBuffer   	ClearSerialBuffer
#define     RS485PUMP_UsartTxflag			UsartTxflag6
#define		RS485PUMP_Usart_Init			Usart6_Dma2_Init



typedef struct{
	uint8_t pumpflag; //蠕动泵连接标志位 0 未连接 1 已连接 
	uint8_t run;	  //开始运行 0 未发送  1 发送  2 接收应答  3未应答 4应答错误
	uint8_t stop;	  //停止运行 0 未发送  1 发送  2 接收应答  3未应答 4 应答错误
	uint8_t gear;	  //挡位控制 0 未发送  1 一档  2 二档 3 三档 4 四档	5 五档 	6 六档	7 七档  8 接收应答  9 未应答 10 应答错误
} PUMPSTA;

extern PUMPSTA pumpsta;

void pump_task_init(void);

#endif








