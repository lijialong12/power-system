/**
 ****************************************************************************************************
 * @file        hmi.h
 * @author      lijialong
 * @version     V1.0
 * @date        2025-7-18
 * @brief       陶晶池串口屏 驱动代码
 * @license     重庆博仕康科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 修改说明
 * V1.0 20250718
 * 第一次发布
 *
 ****************************************************************************************************
 **/


#ifndef _HMI_
#define _HMI_

#include "./SYSTEM/sys/sys.h"
#include "stdio.h"


#define Debug_Printf(...)       printf(__VA_ARGS__)

#define  	SET				1		//真
#define     RESET			0		//假
#define 	LEN				255		//缓冲区长度


//串口
#define		HmiUsart					4
#define     HmiUsart_Transmit			Usart_Transmit
#define     HmiUsartRxLen   			UsartRxLen4	
#define     HmiCopySerialData   		CopySerialData	
#define     HmiClearSerialBuffer   		ClearSerialBuffer
#define     HmiUsartTxflag				UsartTxflag4
#define		HmiUsart_Init				Uart4_Dma1_Init



#define  ADDR_POWER			0x0			//EEPROM存光源大小地址
#define  ADDR_STRASPE		0x20		//EEPROM存正转速度大小地址
#define  ADDR_REVESPE		0x40		//EEPROM存反转速度大小地址
#define  ADDR_STRESPE		0x60		//EEPROM存正反转速度大小地址
#define  ADDR_MODE			0x80		//EEPROM存模式地址

#define  GRINDING_1			10000		//磨砖1挡位
#define  GRINDING_2			20000		//磨砖2挡位
#define  GRINDING_3			30000		//磨砖3挡位
#define  GRINDING_4			40000		//磨砖4挡位
#define  GRINDING_5			50000		//磨砖5挡位
#define  GRINDING_6			60000		//磨砖6挡位
#define  GRINDING_7			70000		//磨砖7挡位
#define  GRINDING_8			75000		//磨砖8挡位

#define	 PLANING_1			500			//刨削1挡位
#define	 PLANING_2			900			//刨削2挡位
#define	 PLANING_3			1300		//刨削3挡位
#define	 PLANING_4			1700		//刨削4挡位
#define	 PLANING_5			2100		//刨削5挡位
#define	 PLANING_6			2500		//刨削6挡位
#define	 PLANING_7			3000		//刨削7挡位
#define	 PLANING_8			3500		//刨削8挡位

typedef struct{

uint8_t  	Dynsysta;			//0 不转  1 正转	2 反转  3 正反转
uint32_t 	DynsystraSpe;		//正转速度
uint32_t 	DynsyReveSpe;		//反转速度
uint16_t 	DynsySRSpe;			//正反转速度
uint8_t  	num_flag;			//挡位切换标志值
uint8_t 	hmi_flag;
uint8_t 	handel;				//1 手柄1 	2 手柄2
uint8_t 	handelsta;			//手柄切换当前状态
uint8_t   op_delayflag;		//过流延时处理

} HMIVAR;

#define display_cps 		 5000

//Dynsysta = 1;			//当前旋转模式状态 0 不转  1 正转	2 反转  3 正反转
//DynsystraSpe = 70000;		//正转速度默认值
//DynsyReveSpe = 70000;		//反转速度默认值
//DynsySRSpe = 3500;			//正反转速度默认值
//num_flag = 2;				//默认2档 挡位切换标志值
//hmi_flag = 0;

extern uint16_t Errornumber;
extern uint16_t g_timer_3s;
extern uint16_t p_timer_3s;

extern 	HMIVAR	hmivar;

extern	uint8_t  hmi_Rxbff[LEN];

uint8_t hmi_init(void);
void HmiReceiveDate(void);
void HmiPara(void);
void InspectCamera(void);
void hmi_task_init(void);

#endif



