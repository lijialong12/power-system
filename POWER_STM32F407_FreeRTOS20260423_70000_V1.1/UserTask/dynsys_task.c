
 
/**
 ****************************************************************************************************
 * @file        dynsys_task.c
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-8-06
 * @brief       机器人项目
 * @license    	重庆博仕康科技有限公司
 ****************************************************************************************************
 * @attention
 * 修改说明
 * V1.0 20250805
 * 第一次发布
 *	修改：70000转数据16uint不够，将发送的转速缩小10倍————李佳龙 20251226
 ****************************************************************************************************
 */

#include "dynsys_task.h"
#include "includes.h"
#include "./MALLOC/malloc.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "./BSP/RS485/rs485.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define DYNSYS_PRINTF(format, ...)     //Debug_Printf("【DYNSYS_TASK】:"format "\r\n",##__VA_ARGS__)
#define BUFFER_SIZE 40  // 缓冲区容量

#define CALIBRATION   0  	// 传感器标定数值
#define	FALLINGEDGE   1.5	// 传感器下降沿总幅值

/* DYNSYS_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define DYNSYS_PRIO      	CONFIG_DYNSYS_PRIO                   		    /* 任务优先级 */
#define DYNSYS_STK_SIZE  	CONFIG_DYNSYS_STK_SIZE                         /* 任务堆栈大小 */
TaskHandle_t            	DYNSYSTask_Handler;  							/* 任务句柄 */
void DYNSYS_TASK(void *pvParameters);             						/* 任务函数 */

float sensor_value = 0;

//uint8_t pc_Txbff[8] = {0x00,0x01,0x05,0x12,0x00,0x01,0xF5,0x49};


//调试口开关
#define   Debug      1

typedef enum {
    SSTATE_IDLE = 0,      // 空闲状态
    SSTATE_SENDING,       // 正在发送
    SSTATE_WAITING_ACK,   // 等待应答
    SSTATE_TIMEOUT        // 超时状态
} DynsyState;


DYNSYSTA DynsySta = {0,0,0,0,10,100,0,0,0,0};		//控制接口默认值

//状态机
volatile DynsyState Dynstate = SSTATE_IDLE; //初始化为空闲状态

volatile uint8_t DynsyDataReady = 0; 		 //数据准备标志
uint16_t Dynsytimout = 0;					 //发送时间记录
static uint8_t  DynsyTimeoutTimes = 0;		 //接收超时次数
static uint8_t  RxError = 0;				 //接收的数据错误累计


//uint32_t speed_num = 0;

//发送数据帧格式
const char framhead[] = {0x83,0x68,0x43,0x00,0x02};			//帧头
const char framfunc = 0x0A;									//功能码
const char framgear[] = {0x01,0x7C}; 						//挡位
const char framcirc = 0x64;									//正反转圈数固定
const char frammode1 = 0x52;								//顺时针转 
const char frammode2 = 0x4C;								//逆时针转 
const char frammode3 = 0x4F;								//正反转 
const char framcrashstopOFF = 0x33;							//急停关闭
const char framcrashstopON = 0x34;							//急停打开
const char framwinlockON = 0x33;							//窗锁功能开
const char framwinlockOFF = 0x34;							//窗锁功能关
const char framreduratio[] = {0x33,0x34};					//减速比
const char framfixedval[] = {0x14,0x14};					//固定值


//接收数据帧格式
const char  Rxframhead[] = {0x83,0x68,0x43,0x00,0x02};		//帧头
const char  Rxframfunc = 0x0C;								//功能码
const char  Rxframgear[] = {0x01,0x7C}; 					//挡位
const char  Rxframcirc = 0x64;								//正反转圈数固定
const char  Rxframmode1 = 0x52;								//顺时针转 
const char  Rxframmode2 = 0x4C;								//逆时针转 
const char  Rxframmode3 = 0x4F;								//正反转 
const char	RxelectCur[] = {0x00 ,0x00};					//两个字节反馈电机实际AD采样电流
const char  Rxframfixedval = 0x00;							//固定值
//char	RxErCurrent = 0x01;								//过流
//char	RxErVoltage = 0x02;								//电源电压异常
//char	RxErVolCur =  0x03;								//电源异常,过流
// char	RxErVoltage_1 =  0x04;							//电机电压异常
// char	RxErVoltCur_1 =  0x05;							//电机电压异常+过流
//char	RxErVoltage_2 =  0x06;							//电压异常
// char	RxErVoltCur_3 =  0x07;							//电压异常+过流

// char	Rxnormacode = 0x00;								//正常
typedef enum
{
    Rxnormacode     = 0x00,  // 正常
    RxErCurrent      = 0x01,  // 过流
    RxErVoltage      = 0x02,  // 电源电压异常
    RxErVolCur      = 0x03,  // 电源异常,过流
    RxErVoltage_1    = 0x04,  // 电机电压异常
    RxErVoltCur_1    = 0x05,  // 电机电压异常+过流
    RxErVoltage_2    = 0x06,  // 电压异常
    RxErVoltCur_3    = 0x07   // 电压异常+过流
	
} RxStatusTypeDef;


//初始化数据帧
char init[] = {0x83,0x68,0x43,0x00,0x02,0x0A,0x00,0x00,0x01,0x7C,0x64,0x4F,0x34,0x34,0x14,0x14,0x03,0x0C};


	
//初始化压力传感器数据
//char RS485SENSOR_init[] = {0xA3,0x04,0x83,0x78};	//发送后传感器自动实时发送回来
	
//接收数据缓存区	
static uint8_t DynsyRxbff[LEN] = " ";	


//接收数据缓存区	
//static char RS485SENSORRxbff[LEN] = " ";	
	
/**
 * @brief       MODBUS CRC-16(MSB-LSB)校验码
 * @param       data: 校验码首地址
	*				length:	数据长度
 * @retval      16位CRC校验码
 */	
uint16_t modbus_crc16(const char *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // 初始化 CRC 寄存器
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i]; // 与当前字节异或
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { // 检查最低位
                crc >>= 1;     // 右移 1 位
                crc ^= 0xA001; // 与多项式异或
            } else {
                crc >>= 1;     // 直接右移
            }
        }
    }
    // 交换高位和低位字节
    return (crc << 8) | (crc >> 8);
}


/**
 * @brief       电机驱动板通讯初始化和检测
 * @param       无
 * @retval      0：成功
				1：失败
 */	
uint8_t Dynsylnit(void)
{
	
	uint8_t back = FAIL;
	DynsyUsart_Init(115200);	//初始化
	vTaskDelay(20);						//等待初始化完成
	memset(DynsyRxbff, 0, LEN * sizeof(char));				//先清空一下缓冲区
	DynsyUsart_Transmit(DynsyUsart,init,sizeof(init));
	vTaskDelay(200);						//等待接收完成
	
    if(DynsyUsartRxLen)
	{
		DynsyCopySerialData(DynsyUsart,(char *)DynsyRxbff,sizeof(DynsyRxbff));

		if((DynsyRxbff[0] == 0x83) && (DynsyRxbff[5] == 0x0C) && (DynsyRxbff[19] == 0x31)) //检查校验
		{
			back = SUCC;//通讯成功
			memset(DynsyRxbff, 0, LEN * sizeof(char));		//先清空一下缓冲区
		}
		else
		{
			back = FAIL;//通讯失败
			memset(DynsyRxbff, 0, LEN * sizeof(char));		//先清空一下缓冲区
		}	
		DynsyClearSerialBuffer(DynsyUsart);		
	}
	else{DynsyClearSerialBuffer(DynsyUsart); back = FAIL;}//通讯失败
	
	return back;
	
}


/**
 * @brief       字符串后面添加字符数据
 * @param       str:目标字符串首地址
				size：开始添加的地址位置
				c:字符数据
 * @retval      无
 */	
void appendChar(char *str, uint8_t size, char c) {

	*(str+size-1) =  c;
	*(str+size) =  '\0';
}



/**
 * @brief       驱动电机正转数据发送函数
 * @param       CircSped:转速 [0-60000]  act 1运行 0停止  loop 0 单次发送 1 循环发送  
 * @retval      无
 **/	
void MotorRotatesForward(uint32_t  CircSped,uint8_t act, uint8_t loop)
{
	CircSped = CircSped/10;							//将数据除10发送
	uint8_t *p = (uint8_t *)&CircSped;				//拆开成两个八位数据
	uint16_t size;									//字符累计
	uint8_t sramx = 0;                  			/* 默认为内部sram */
	char *str = NULL;
	if(loop){DynsySta.CycleSend = 1;}
	else{DynsySta.CycleSend = 0;}
	str  = mymalloc(sramx, 50);  /* 申请50*4字节 */	//动态申请内存
	size_t current_len = sizeof(framhead);			//计算发送的字符数
	memcpy(str,framhead,current_len);				//复制到目标指针里
	size = current_len;								//记录字符数
	size++;											//字符累计
	appendChar(str,size,framfunc);					//添加字符数
	size++;											//字符累计
	appendChar(str,size,p[1]);
	size++;											//字符累计
	appendChar(str,size,p[0]);
	current_len = sizeof(framgear);
	memcpy(str + size,framgear,current_len);
	size =size + current_len;
	size++;											//字符累计
	appendChar(str,size,framcirc);
	size++;											//字符累计
	appendChar(str,size,frammode1);
	size++;											//字符累计
	if(act)
	{appendChar(str,size,framcrashstopOFF);}
	else
	{appendChar(str,size,framcrashstopON);}

	size++;											//字符累计
	appendChar(str,size,framwinlockOFF);
	
	/***
	current_len = sizeof(framreduratio);
	memcpy(str + size,framreduratio,current_len);
	size =size + current_len;
	***/
	
	current_len = sizeof(framfixedval);
	memcpy(str + size,framfixedval,current_len);
	size =size + current_len;
	uint16_t CRC_P = modbus_crc16(str,size);		//计算校验码
	uint8_t *p_crc = (uint8_t *)&CRC_P;				//将校验码拆开成两个八位数据
	size++;											//字符累计
	appendChar(str,size,p_crc[0]);
	size++;											//字符累计
	appendChar(str,size,p_crc[1]);
	
	//发送前处理
	DynsyDataReady = SET;
	DynsyClearSerialBuffer(DynsyUsart);
	DynsyUsartTxflag = RESET;
	memset(DynsyRxbff, 0, LEN * sizeof(char));		//先清空一下缓冲区

	DynsyUsart_Transmit(DynsyUsart,str,size);

	if(act){DynsySta.StaSend = STRA; DynsyTim_Start();}
	else{DynsySta.StaSend = STOP; DynsyTim_Start();}
	
	myfree(sramx, str);   /* 释放内存 */
	str = 0;              /* 指向空地址 */
}




/**
 * @brief       驱动电机反转数据发送函数
 * @param       CircSped:转速 [0-60000]  act 1运行 0停止  loop 0 单次发送 2 循环发送  
 * @retval      无
 **/
void MotorRotatesBack(uint32_t  CircSped, uint8_t act, uint8_t loop)
{
	CircSped = CircSped/10;							//将数据除10发送
	uint8_t *p = (uint8_t *)&CircSped;				//拆开成两个八位数据
	uint16_t size;									//字符累计
	uint8_t sramx = 0;                  			/* 默认为内部sram */
	char *str = NULL;
	if(loop){DynsySta.CycleSend = 2;}
	else{DynsySta.CycleSend = 0;}
	str  = mymalloc(sramx, 50);  /* 申请50*4字节 */	//动态申请内存
	size_t current_len = sizeof(framhead);			//计算发送的字符数
	memcpy(str,framhead,current_len);				//复制到目标指针里
	size = current_len;								//记录字符数
	size++;											//字符累计
	appendChar(str,size,framfunc);					//添加字符数
	size++;											//字符累计
	appendChar(str,size,p[1]);
	size++;											//字符累计
	appendChar(str,size,p[0]);
	current_len = sizeof(framgear);
	memcpy(str + size,framgear,current_len);
	size =size + current_len;
	size++;											//字符累计
	appendChar(str,size,framcirc);
	size++;											//字符累计
	appendChar(str,size,frammode2);					//逆时针模式
	size++;											//字符累计
	if(act)
	{appendChar(str,size,framcrashstopOFF);}
	else
	{appendChar(str,size,framcrashstopON);}
	size++;											//字符累计
	appendChar(str,size,framwinlockOFF);
	
	/***
	current_len = sizeof(framreduratio);
	memcpy(str + size,framreduratio,current_len);
	size =size + current_len;
	***/
	
	current_len = sizeof(framfixedval);
	memcpy(str + size,framfixedval,current_len);
	size =size + current_len;
	uint16_t CRC_P = modbus_crc16(str,size);		//计算校验码
	uint8_t *p_crc = (uint8_t *)&CRC_P;				//将校验码拆开成两个八位数据
	size++;											//字符累计
	appendChar(str,size,p_crc[0]);
	size++;											//字符累计
	appendChar(str,size,p_crc[1]);
	
	//发送前处理
	DynsyDataReady = SET;
	DynsyClearSerialBuffer(DynsyUsart);
	DynsyUsartTxflag = RESET;
	memset(DynsyRxbff, 0, LEN * sizeof(char));		//先清空一下缓冲区

	DynsyUsart_Transmit(DynsyUsart,str,size);

	if(act){DynsySta.StaSend = REVE; DynsyTim_Start();}
	else{DynsySta.StaSend = STOP; DynsyTim_Start();}
	
	myfree(sramx, str);   /* 释放内存 */
	str = 0;              /* 指向空地址 */
	
}



/**
 * @brief       驱动电机正反转数据发送函数
 * @param       CircSped:转速 [0-60000]  act 1运行 0停止  loop 0 单次发送 3 循环发送  
 * @retval      无
 **/
void MotorRotatesFoBa(uint32_t  CircSped, uint8_t act, uint8_t loop)
{
	
	uint8_t *p = (uint8_t *)&CircSped;				//拆开成两个八位数据
	uint16_t size;									//字符累计
	uint8_t sramx = 0;                  			/* 默认为内部sram */
	char *str = NULL;
	if(loop){DynsySta.CycleSend = 3;}
	else{DynsySta.CycleSend = 0;}
	str  = mymalloc(sramx, 50);  /* 申请50*4字节 */	//动态申请内存
	size_t current_len = sizeof(framhead);			//计算发送的字符数
	memcpy(str,framhead,current_len);				//复制到目标指针里
	size = current_len;								//记录字符数
	size++;											//字符累计
	appendChar(str,size,framfunc);					//添加字符数
	size++;											//字符累计
	appendChar(str,size,p[1]);
	size++;											//字符累计
	appendChar(str,size,p[0]);
	current_len = sizeof(framgear);
	memcpy(str + size,framgear,current_len);
	size =size + current_len;
	size++;											//字符累计
	appendChar(str,size,framcirc);
	size++;											//字符累计
	appendChar(str,size,frammode3);					//正反转模式
	size++;											//字符累计
	if(act)
	{appendChar(str,size,framcrashstopOFF);}
	else
	{appendChar(str,size,framcrashstopON);}
	size++;											//字符累计
	appendChar(str,size,framwinlockOFF);
	
	/***
	current_len = sizeof(framreduratio);
	memcpy(str + size,framreduratio,current_len);
	size =size + current_len;
	***/
	
	current_len = sizeof(framfixedval);
	memcpy(str + size,framfixedval,current_len);
	size =size + current_len;
	uint16_t CRC_P = modbus_crc16(str,size);		//计算校验码
	uint8_t *p_crc = (uint8_t *)&CRC_P;				//将校验码拆开成两个八位数据
	size++;											//字符累计
	appendChar(str,size,p_crc[0]);
	size++;											//字符累计
	appendChar(str,size,p_crc[1]);

	//发送前处理
	DynsyDataReady = SET;
	DynsyClearSerialBuffer(DynsyUsart);
	DynsyUsartTxflag = RESET;
	memset(DynsyRxbff, 0, LEN * sizeof(char));		//先清空一下缓冲区

	DynsyUsart_Transmit(DynsyUsart,str,size);
	
	if(act){DynsySta.StaSend = STRE; DynsyTim_Start();}
	else{DynsySta.StaSend = STOP; DynsyTim_Start();}
	
	myfree(sramx, str);   /* 释放内存 */
	str = 0;              /* 指向空地址 */	
}





/**
 * @brief       验证MODBUS帧的CRC是否正确
 * @param       CircSped:转速 [0-60000]
 * @retval      返回1（错误）或0（正常）
 **/
int modbus_crc_check(char *frame, uint16_t frame_length) {
    if (frame_length < 3) return 1;  // 至少需要1字节数据 + 2字节CRC
 
    uint16_t calculated_crc = modbus_crc16(frame, frame_length - 2);  // 计算前N-2字节的CRC
    uint16_t received_crc = (frame[frame_length - 1] << 8) | frame[frame_length - 2];  // 提取CRC（MSB-LSB）
 
    return !(calculated_crc == received_crc); 	
}






/**
 * @brief       控制电机数据发送循环函数
 * @param       无
 * @retval      返回 0（无动作）或 1（正确）或 2（过流）或 3（应答错误）4 （无应答）
 **/
void DynsysUpdate(void)
{
	static	uint8_t cic = 255;		//等待串口发送完成最长时间
	switch(Dynstate) 
		{
			
			case SSTATE_IDLE:		//空闲状态
					if (DynsyDataReady) 
					{
						DynsyDataReady = RESET;
						Dynstate = SSTATE_SENDING;
					}
					break;
			case SSTATE_SENDING:		//发送状态
					{
						if((DynsyUsartTxflag != SET) && (--cic))
						{
							#if Debug	// 如果Debug已定义为1，编译此部分代码
							DYNSYS_PRINTF("等待串口发送完成 %d\r\n",cic);	
							#endif	
							if(cic == 0)
							{
								cic = 255;
								Dynstate = SSTATE_IDLE;
								#if Debug	// 如果Debug已定义为1，编译此部分代码
								DYNSYS_PRINTF("发送失败！！\r\n");
								#endif	
								DynsyDataReady = RESET;
							}
						}
						else
						{
							cic = 0;
							DynsyUsartTxflag = RESET;
							Dynstate = SSTATE_WAITING_ACK;
						}
					}
					break;				
			case SSTATE_WAITING_ACK:	//应答状态
					{
							if(DynsyUsartRxLen)
							{   
										DynsyCopySerialData(DynsyUsart,(char *)DynsyRxbff,sizeof(DynsyRxbff));	//拷贝串口数据
										 /*数据打印处理*/
//										 Usart_Transmit(1,(char *)DynsyRxbff,DynsyUsartRxLen);
//										 vTaskDelay(50);                                               /* 延时10ticks */
										if(!modbus_crc_check((char *)DynsyRxbff,DynsyUsartRxLen)) //检查校验		
										{
											
											if(DynsyRxbff[11])
											{
												if(DynsyRxbff[17] == Rxnormacode)
												{
													static uint8_t abc = 0;
													
													DynsySta.speed_num = ((DynsyRxbff[6]  << 8) | (DynsyRxbff[7] ));
													
													if(DynsyRxbff[11] == 0x4F)
													{
														DynsySta.speed_num = DynsySta.speed_num*5.3;
													}
													else
													{
														if(DynsySta.speed_num)DynsySta.speed_num = DynsySta.speed_num*10 + 30;//真实速度加补偿值
													}
													
//													DYNSYS_PRINTF("运行正常   %d\r\n",speed_num);
													
													#if Debug	// 如果Debug已定义为1，编译此部分代码
													//if((num  >= (DynsySta.CycleReveSped * 0.93) * 5.3) && (num  <= DynsySta.CycleReveSped * 1.07 * 5.3) && (abc >= 5))
													#endif													
													if(DynsySta.speed_num  > 0)
													{
														abc = 0;
														DynsyTim_Stop();
														//DYNSYS_PRINTF("运行正常   %d\r\n",num);
														DynsySta.StaRece = NORMAL;//运行正常
													}
													else
													{
														abc++;
														DynsyTim_Stop();

														if(abc > 3)
														{
															abc = 0;
															DynsySta.StaRece = NOTROTAT;//电机不转，线路或者问题
														}
													}
													
												}
												 else 
												{
													DynsyTim_Stop();
													DynsySta.StaRece = RXERERR;//发生错误
												}
											}
											else if(DynsyRxbff[11] == 0)
											{	
												switch(DynsyRxbff[17])
												{
													case Rxnormacode: 	DynsySta.StaRece = NORMAL; break;//运行正常
													case RxErCurrent: 	DynsySta.StaRece = OVERCUR; break;//运行正常
													case RxErVoltage: 	DynsySta.StaRece = RCURVOLT_1; break;//运行正常
													case RxErVolCur: 	DynsySta.StaRece = OVEVOLT_1; break;//运行正常
													case RxErVoltage_1: DynsySta.StaRece = RCURVOLT; break;//运行正常
													case RxErVoltCur_1: DynsySta.StaRece = RCURVOLT_2; break;//运行正常
													case RxErVoltage_2: DynsySta.StaRece = OVEVOLT; break;//运行正常
													case RxErVoltCur_3: DynsySta.StaRece = RCURVOLT_3; break;//运行正常
													default : break;
												}
												DynsyTim_Stop();
											}
											else
											{
													DynsyTim_Stop();
													DynsySta.StaRece = RXERERR;//发生错误
											}
											#if Debug	// 如果Debug已定义为1，编译此部分代码
											DYNSYS_PRINTF("Dynsy接收成功 = %d\r\n",DynsySta.StaRece);
											#endif	
											 DynsyDataReady = 0;
											 memset(DynsyRxbff, 0, LEN * sizeof(char));
											 DynsyTim_Stop();
											 Dynsytimout = 0;
											 DynsyClearSerialBuffer(DynsyUsart);
											 Dynstate = SSTATE_IDLE;
											 break;
										}
										else
										{	
											RxError++;	//接收的数据错误次数累计
											if(RxError >= 1)	//1次为接收错误
											{
												DynsySta.StaRece = RXERERR;//通讯失败
												RxError = 0;	//清零处理
												memset(DynsyRxbff, 0, LEN * sizeof(char));
												DynsyDataReady = 0;
												//错误处理返回给上位机
												#if Debug	// 如果Debug已定义为1，编译此部分代码
												DYNSYS_PRINTF("Dynsy接收的数据错误 = %d\r\n",DynsySta.StaRece);
												#endif
											}
											else
											{
												 DynsyDataReady = SET;
											}
											DynsyTim_Stop();
											Dynsytimout = 0;
											DynsyClearSerialBuffer(DynsyUsart);
											Dynstate = SSTATE_IDLE;
										}
							}
							else
							{
								if(Dynsytimout >= 100)	//100ms为超时时间
								{	
									Dynsytimout = 0; 	//定时器计时清零
									DynsyTimeoutTimes++;		//超时次数累计
									#if Debug	// 如果Debug已定义为1，编译此部分代码
									DYNSYS_PRINTF("TimeoutTimes = %d\r\n",DynsyTimeoutTimes);
									#endif
									if(DynsyTimeoutTimes >= 2)
									{
										Dynstate = SSTATE_TIMEOUT;	//进入超时状态
									}
								}		
							}
					}
					break;
			case SSTATE_TIMEOUT:		//超时状态
					{
						DynsyTimeoutTimes = 0;	//发送次数清零
						DynsyTim_Stop();		//关闭定时器
						Dynsytimout = 0;	//定时器计时清零
						
						DynsySta.StaRece = NORES;	//无数据
						
						DynsyDataReady = 0;
						//超时处理返回给上位机
						#if Debug	// 如果Debug已定义为1，编译此部分代码
						DYNSYS_PRINTF("接收超时错误！！！超时代码为 =  %d\r\n" ,DynsySta.StaRece);
						#endif
						Dynstate = SSTATE_IDLE;
					}
					break;
			default:	break;
		}
}





 
// 移动缓冲区数据并添加新元素（新增极值索引参数，最大值和最小值指针参数）
void shift_and_add(float buffer[], 
                   uint8_t *current_size,
                   float *max_value, 
                   float *min_value,
                   uint8_t *max_index, 
                   uint8_t *min_index,
                   float new_data) {
    // 检查是否需要扩容
    if (*current_size >= BUFFER_SIZE) {
        float removed_element = buffer[0];
        
        // 整体前移
        for (int i = 0; i < *current_size; i++) {
            buffer[i] = buffer[i + 1];
        }
        
        // 末尾添加新数据
        buffer[*current_size] = new_data;
        
        // 极值更新逻辑
        if (*current_size > 0) {
            bool need_recalc = false;
            
            // 检查被移除元素是否是当前极值
            if (removed_element == *max_value || 
                removed_element == *min_value) {
                need_recalc = true;
            }
            
            if (need_recalc) {
                // 重新计算整个数组的极值及其索引
                *max_value = buffer[0];
                *min_value = buffer[0];
                *max_index = 0;
                *min_index = 0;
                
                for (int i = 1; i <= *current_size; i++) {
                    if (buffer[i] > *max_value) {
                        *max_value = buffer[i];
                        *max_index = i;
                    }
                    if (buffer[i] < *min_value) {
                        *min_value = buffer[i];
                        *min_index = i;
                    }
                }
            } else {
                // 只需比较新元素与当前极值
                if (new_data > *max_value) {
                    *max_value = new_data;
                    *max_index = *current_size;  // 新元素位于末尾
                }
                if (new_data < *min_value) {
                    *min_value = new_data;
                    *min_index = *current_size;
                }
            }
        }
    } else {
        // 缓冲区未满时直接添加
        buffer[*current_size] = new_data;
        
        // 初始化或更新极值及其索引
        if (*current_size == 0) {
            *max_value = new_data;
            *min_value = new_data;
            *max_index = 0;
            *min_index = 0;
        } else {
            if (new_data > *max_value) {
                *max_value = new_data;
                *max_index = *current_size;  // 新元素索引
            }
            if (new_data < *min_value) {
                *min_value = new_data;
                *min_index = *current_size;
            }
        }
        (*current_size)++;
    }
}







/**
 * 将符合特定格式的字符串转换为浮点数
 * 
 * @param str 输入字符串，格式要求：包含一个浮点数，只有一位小数点，末尾是\n
 * @param result 输出参数，存储转换后的浮点数
 * @return 转换成功返回true，失败返回false
 */
bool str_to_float(const char* str, float* result) {
    if (str == NULL || result == NULL) {
        return false; // 无效指针
    }

    // 查找字符串长度，不包括结尾的空字符
    size_t len = strlen(str);
    
    // 检查字符串是否至少有4个字符（例如 "0.0\n"）
    if (len < 4) {
        return false;
    }
    
    // 检查最后一个字符是否是换行符
    if (str[len - 1] != '\n') {
        return false;
    }
    
    // 查找小数点的位置
    int dot_pos = -1;
    for (size_t i = 0; i < len - 1; i++) { // 不检查最后一个字符（\n）
        if (str[i] == '.') {
            // 检查是否已经有一个小数点
            if (dot_pos != -1) {
                return false; // 多个小数点
            }
            dot_pos = i;
        } else if (!isdigit((unsigned char)str[i])) {
            return false; // 包含非数字字符（除了一个小数点和结尾的\n）
        }
    }
    
    // 检查是否有小数点
    if (dot_pos == -1) {
        return false; // 没有小数点
    }
    
    // 检查小数点后是否有且仅有一位数字，且不是最后一个字符（因为最后是\n）
    if (dot_pos >= len - 2) {
        return false; // 小数点后没有数字或数字位数不足
    }
    
    // 现在进行实际转换
    // 计算整数部分
    int integer_part = 0;
    for (int i = 0; i < dot_pos; i++) {
        integer_part = integer_part * 10 + (str[i] - '0');
    }
    
    // 计算小数部分（只有一位）
    int fractional_part = str[dot_pos + 1] - '0';
    
    // 组合成浮点数
    *result = integer_part + fractional_part / 10.0f;
    
    return true;
}




void dynsys_task_init(void)
{
	taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )DYNSYS_TASK,
                (const char*    )"DYNSYS_TASK",
                (uint16_t       )DYNSYS_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DYNSYS_PRIO,
                (TaskHandle_t*  )&DYNSYSTask_Handler);
    taskEXIT_CRITICAL();            /* 退出临界区 */
}






/**
 * @brief       DYNSYS_TASK 
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void DYNSYS_TASK(void *pvParameters)
{
	static uint16_t	DynsysTIM = 0;
	static uint8_t num = 3;
	static uint8_t	CycleTIM = 0;
	
//	float buff[BUFFER_SIZE] = {0};
	vTaskDelay(1500);						//等待板子启动
	while(Dynsylnit() && (--num))
	{
		
		DYNSYS_PRINTF("通讯连接中！！！");
	}
	if(num == 0)
	{
		DYNSYS_PRINTF("通讯失败！！！");
	}
	else
	{
		DynsySta.Commu = 1;
	}
	
	DynsyTim_Init();
	
	/*
	RS485SENSOR_Usart_Init(115200);                       //初始化RS485 压力传感器数据 
	vTaskDelay(200);                                               //延时10ticks 
	DynsyUsart_Transmit(RS485SENSOR_Usart,RS485SENSOR_init,sizeof(RS485SENSOR_init));
	*/
	
    while(1)
    {
				DynsysTIM++;
				CycleTIM++;
				
				if(DynsysTIM > DynsySta.DynsysTime)
				{
					DynsysTIM = 0;
					switch(DynsySta.CycleSend)
					{ 
						case 1:{MotorRotatesForward(DynsySta.CycleStraSped,1,1);break; }
						case 2:{MotorRotatesBack(DynsySta.CycleReveSped,1,2);break;}
						case 3:{MotorRotatesFoBa(DynsySta.CycleSRSped,1,3);break;}
						case 4:{MotorRotatesForward(DynsySta.CycleStraSped,0,0);break;}
						case 5:{MotorRotatesBack(DynsySta.CycleReveSped,0,0);break;}
						case 6:{MotorRotatesFoBa(DynsySta.CycleSRSped,0,0);break;}
						default:break;
					}
					
				}
				if(CycleTIM > DynsySta.CycleTime)					//10ms循环一次
				{  
					CycleTIM = 0;	
					DynsysUpdate();		//接收查询
				}
		/*
				if(RS485SENSOR_UsartRxLen)	//压力传感器数据接收数量
				{
					static uint8_t numb = 0;	//数组缓冲区计数	
					float value_max;
					float value_min;
					uint8_t max_index;
					uint8_t min_index;
					RS485SENSOR_CopySerialData(RS485SENSOR_Usart,RS485SENSORRxbff,sizeof(RS485SENSORRxbff));
					bool success =  str_to_float((char *)RS485SENSORRxbff, &sensor_value);  	//字符串数据转浮点数
					
					if (success)
					{
						//DYNSYS_PRINTF(" 压力差值 = %.1f",sensor_value);
						if(sensor_value > CALIBRATION)	//滤除错误数据
						{
							shift_and_add(buff,&numb,&value_max,&value_min,&max_index,&min_index,sensor_value);	//赋值给缓冲区数组
							if(numb >= BUFFER_SIZE)	//数组元素大于缓存长度
							{		
									float diff;
									if(min_index > max_index)
									{
											diff  = value_max-value_min;
										
											if(diff > FALLINGEDGE)		//总高度达到15则停止
											{
												numb = 0;
												static uint8_t  adc = 0;
												//DYNSYS_PRINTF("停止！！ 压力差值 = %f",diff);
												if(adc == 0)
												{
													adc = 1;

												}
												else
												{

												}
												//RS485PC_Usart_Transmit(RS485PC_Usart,(char *)pc_Txbff,8);		//压力值达到上传上位机电机停止指令
												diff = 0;
												memset(buff, 0, BUFFER_SIZE);		//先清空一下缓冲区
												DynsySta.CycleSend = 4;
												
												switch(DynsySta.CycleSend)
												{ 
													case 1:{MotorRotatesForward(DynsySta.CycleStraSped,1,1);break; }
													case 2:{MotorRotatesBack(DynsySta.CycleReveSped,1,2);break;}
													case 3:{MotorRotatesFoBa(DynsySta.CycleSRSped,1,3);break;}
													case 4:{MotorRotatesForward(DynsySta.CycleStraSped,0,0);break;}
													case 5:{MotorRotatesBack(DynsySta.CycleReveSped,0,0);break;}
													case 6:{MotorRotatesFoBa(DynsySta.CycleSRSped,0,0);break;}
													default:break;
												}
												DynsySta.StaRece = PRESSURE;
												
											}
										
									}
									else 
									{
										diff = 0;		//总距离清零
									}						
							}
							
						}			
						
					} else {
								//printf("转换失败\n");
							}
					
					memset(RS485SENSORRxbff, 0, RS485SENSOR_UsartRxLen);		//先清空一下缓冲区
					DynsyClearSerialBuffer(RS485SENSOR_Usart);

				}	
		*/	

        vTaskDelay(1);                                               /* 延时10ticks */
    }
}



