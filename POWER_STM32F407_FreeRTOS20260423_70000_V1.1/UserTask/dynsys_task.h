#ifndef _DYNSYS_TASK_H
#define _DYNSYS_TASK_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/USART/USART.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/TIM/TIM.h"



#define Debug_Printf(...)       printf(__VA_ARGS__)




#define  	FAIL 				1		//失败
#define     SUCC				0		//成功
#define 	LEN					255		//缓冲区长度



//电机控制串口
#define		DynsyUsart					2
#define     DynsyUsart_Transmit			Usart_Transmit
#define     DynsyUsartRxLen   			UsartRxLen2	
#define     DynsyCopySerialData   		CopySerialData	
#define     DynsyClearSerialBuffer   	ClearSerialBuffer
#define     DynsyUsartTxflag			UsartTxflag2
#define		DynsyUsart_Init				Usart2_Dma1_Init


//传感器数据接受串口
//#define		RS485SENSOR_Usart					2
//#define       RS485SENSOR_Usart_Transmit			Usart_Transmit
//#define       RS485SENSOR_UsartRxLen   			UsartRxLen2	
//#define       RS485SENSOR_CopySerialData   		CopySerialData	
//#define       RS485SENSOR_ClearSerialBuffer   	ClearSerialBuffer
//#define       RS485SENSOR_UsartTxflag				UsartTxflag2
//#define		RS485SENSOR_Usart_Init				Usart2_Dma1_Init


//定时器
#define		DynsyTim_Stop				Tim6_Stop
#define		DynsyTim_Start				Tim6_Start
#define		DynsyTim_Init()				Tim6_Init(1000-1,168-1);				//定时0.5ms

//发送状态
#define		STRA			1			//发送正转
#define		REVE			2			//发送反转
#define		STRE			3			//发送正反转
#define		STOP			4			//停止
#define		FINISH			5			//应答完成
#define		ERERR			6			//未应答


//接收状态
//返回 0（无应答）或 1（正确）或 2（过流）或 3（应答错误）
#define		EMPTY			0			//无动作
#define		NORMAL			1			//正常
#define		OVERCUR			2			//过流
#define		OVEVOLT			3			//电压异常
#define		NOTROTAT		4			//电机不转
#define		PRESSURE		5			//压力值导致停止
#define		RXERERR			6			//应答错误
#define		NORES			7			//无应答
#define		RCURVOLT		8			//电机电压异常
#define		RCURVOLT_1		9			//电源异常,过流
#define		RCURVOLT_2		10			//电机电压异常+过流
#define		RCURVOLT_3		11			//电压异常+过流
#define		OVEVOLT_1		12			//电源电压异常


typedef struct{
    uint16_t  	CycleSend;      	// 工作模式  0 不发送 1发送正转  2 发送反转  3 发送正反转  4 正停止 5 反停止 6 正反停止
	uint32_t	CycleStraSped;		// 正转速
	uint32_t	CycleReveSped;		// 反转速度
	uint16_t	CycleSRSped;		// 正反转速度
	uint8_t 	CycleTime;			// 循环接收处理时间 单位1ms
	uint16_t 	DynsysTime;			// 循环发送处理时间 单位1ms
    uint8_t 	StaSend;        	// 0  未发送	1 发送正转  2 发送反转  3 发送正反转  4停止
    uint8_t		StaRece;   			// 返回 0（无动作）或 1（正确）或 2（过流）或 3（电机不转）4 (压力值达到停止) 或 4（应答错误）5 （无应答）
	uint8_t		Commu;   			//
	uint32_t 	speed_num;			//电机返回转速
} DYNSYSTA;




extern  DYNSYSTA 	DynsySta;
extern uint16_t Dynsytimout;
extern float sensor_value;

void dynsys_task_init(void);

#endif













