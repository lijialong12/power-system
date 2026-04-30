#include "pump_task.h"
#include "includes.h"


#define PUMP_PRINTF(format, ...)     //Debug_Printf("【PUMP_TASK】:"format "\r\n",##__VA_ARGS__)

/* PUMP_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define PUMP_PRIO      	CONFIG_PUMP_PRIO                   		     /* 任务优先级 */
#define PUMP_STK_SIZE  	CONFIG_PUMP_STK_SIZE                         /* 任务堆栈大小 */
TaskHandle_t            PUMPTask_Handler;  							 /* 任务句柄 */
void PUMP_TASK(void *pvParameters);             					 /* 任务函数 */



uint8_t RS485PUMP_Rxbff[PUMPLEN] = "";	//接收数据缓冲区		

typedef enum {
    STATE_IDLE = 0,      // 空闲状态
    STATE_SENDING,       // 正在发送
    STATE_WAITING_ACK,   // 等待应答
    STATE_TIMEOUT        // 超时状态
} PumpState;


PUMPSTA pumpsta = {0};

//状态机
volatile PumpState Pumptate = STATE_IDLE; //初始化为空闲状态

volatile uint8_t PumpDataReady = 0; 		 //数据准备标志
static uint16_t Pumptimout = 0;					 //发送时间记录
static uint8_t  RxError = 0;				 //接收的数据错误累计

static uint8_t dataSendFinish = 0; 	     // 数据发送完成标志
static uint8_t dataSendType = 0; 		 // 数据发送类型标志 0代表不发送


/**
 * @brief 计算MODBUS CRC-16校验码
 * @param data 输入数据指针
 * @param length 数据长度（字节数）
 * @return 计算得到的CRC-16校验码（低字节在前，高字节在后）
 */
static	uint16_t pump_modbus_crc16(const char *data, uint16_t length) {
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
    return crc;
}



/**
 * @brief       验证MODBUS帧的CRC是否正确
 * @param       CircSped:转速 [0-60000]
 * @retval      返回1（错误）或0（正常）
 **/
static int pump_modbus_crc_check(char *frame, uint16_t frame_length) {
    if (frame_length < 3) return 1;  // 至少需要1字节数据 + 2字节CRC
 
    uint16_t calculated_crc = pump_modbus_crc16(frame, frame_length - 2);  // 计算前N-2字节的CRC
    uint16_t received_crc = (frame[frame_length - 1] << 8) | frame[frame_length - 2];  // 提取CRC（MSB-LSB）
 
    return !(calculated_crc == received_crc); 	
}



//通讯初始化
uint8_t pump_init(void)
{
	uint8_t sta = RESET;
	char init[] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};	// 查看初始化指令（查看型号）
	RS485PUMP_Usart_Init(115200);								// 屏幕通讯
	vTaskDelay(20);
	memset(RS485PUMP_Rxbff, 0, LEN * sizeof(char));				// 先清空一下缓冲区
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)init,sizeof(init));
	vTaskDelay(200);
    if(RS485PUMP_UsartRxLen)
	{

		RS485PUMP_CopySerialData(RS485PUMP_Usart,(char *)RS485PUMP_Rxbff,sizeof(RS485PUMP_Rxbff));	
		if((RS485PUMP_Rxbff[0] == 0x01) && (RS485PUMP_Rxbff[1] == 0x03) && (RS485PUMP_Rxbff[2] == 0x02) && (RS485PUMP_Rxbff[3] == 0x03)) 
		{
			sta = PUMPRESET;//通讯成功
			pumpsta.pumpflag = 1;	//通讯成功，泵图标使能
		}
		else
		{
			sta = PUMPSET;//通讯失败
		}	
		RS485PUMP_ClearSerialBuffer(RS485PUMP_Usart);		
	}
	else{RS485PUMP_ClearSerialBuffer(RS485PUMP_Usart); sta = SET;}//通讯失败
	
	return sta; 
}





//运行速度大小
void pump_running_speed_send(int16_t speed)
{
	char runspeed[] = {0x01,0x06,0x00,0x1D,0x04,0xB0,0x1A,0xB8};
	
    uint8_t low_byte  = speed & 0xFF;        // 取低8位
    uint8_t high_byte = (speed >> 8) & 0xFF; // 取高8位
	
	runspeed[4] = high_byte;
	runspeed[5] = low_byte;
	
	uint16_t crc = pump_modbus_crc16(runspeed, sizeof(runspeed)-2);
	
	uint8_t low_crc  = crc & 0xFF;        // 取低8位
    uint8_t high_crc = (crc >> 8) & 0xFF; // 取高8位
	
	runspeed[6] = low_crc;
	runspeed[7] = high_crc;
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)runspeed,sizeof(runspeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}


//加速度大小
void pump_accel_speed_send(uint16_t speed)
{
	char acspeed[] = {0x01,0x06,0x00,0x1F,0x04,0xB0,0x1A,0xB8};
	
    uint8_t low_byte  = speed & 0xFF;        // 取低8位
    uint8_t high_byte = (speed >> 8) & 0xFF; // 取高8位
	
	acspeed[4] = high_byte;
	acspeed[5] = low_byte;
	
	uint16_t crc = pump_modbus_crc16(acspeed, sizeof(acspeed)-2);
	
	uint8_t low_crc  = crc & 0xFF;        // 取低8位
    uint8_t high_crc = (crc >> 8) & 0xFF; // 取高8位
	
	acspeed[6] = low_crc;
	acspeed[7] = high_crc;
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)acspeed,sizeof(acspeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}


//减速度大小
void pump_decel_speed_send(uint16_t speed)
{
	char despeed[] = {0x01,0x06,0x00,0x1F,0x04,0xB0,0x1A,0xB8};
	
    uint8_t low_byte  = speed & 0xFF;        // 取低8位
    uint8_t high_byte = (speed >> 8) & 0xFF; // 取高8位
	
	despeed[4] = high_byte;
	despeed[5] = low_byte;
	
	uint16_t crc = pump_modbus_crc16(despeed, sizeof(despeed)-2);
	
	uint8_t low_crc  = crc & 0xFF;        // 取低8位
    uint8_t high_crc = (crc >> 8) & 0xFF; // 取高8位
	
	despeed[6] = low_crc;
	despeed[7] = high_crc;
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)despeed,sizeof(despeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}



//开始运行
void pump_on_send(void)
{
	unsigned char onspeed[] = {0x01,0x06,0x00,0x27,0x00,0x02,0xB8,0x00};
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)onspeed,sizeof(onspeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}


//停止运行
void pump_off_send(void)
{
	unsigned char onspeed[] = {0x01,0x06,0x00,0x27,0x01,0x00,0x38,0x51};
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)onspeed,sizeof(onspeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}


//电机释放
void pump_release_send(void)
{
	unsigned char respeed[] = {0x01,0x06,0x00,0x2D,0x00,0x11,0xD9,0xCF};
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)respeed,sizeof(respeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}




//电机使能
void pump_enable_send(void)
{
	unsigned char enaspeed[] = {0x01,0x06,0x00,0x2D,0x00,0x12,0x99,0xCE};
	
	RS485PUMP_Usart_Transmit(RS485PUMP_Usart,(char *)enaspeed,sizeof(enaspeed));
	vTaskDelay(10);
	dataSendFinish = 1;		//数据发送完成标志，用于大量数据发送
}








//循环查询指令状态
void DetectionStatus(void)
{
	if(Pumptate == STATE_IDLE)
	{
		 if(pumpsta.gear == 1)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 1;	  //给定类型
		}
		else if(pumpsta.gear == 2)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 2;	  //给定类型
		}
		else if(pumpsta.gear == 3)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 3;	  //给定类型
		}
		else if(pumpsta.gear == 4)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 6;	  //给定类型
		}
		else if(pumpsta.gear == 5)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 7;	  //给定类型
		}
		else if(pumpsta.gear == 6)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 8;	  //给定类型
		}
		else if(pumpsta.gear == 7)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 9;	  //给定类型
		}
		else if(pumpsta.run == 1)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 4;	  //给定类型
		}
		else if(pumpsta.stop == 1)
		{
			PumpDataReady = PUMPSET; //数据准备完成
			dataSendType = 5;	  //给定类型
		}

	}	
}



//发送数据类型判断
static void JudgeDataSendType(void)
{
	switch(dataSendType)
	{								
		case 1:   pump_running_speed_send(-200);								break;//一档速度
		case 2:   pump_running_speed_send(-500);							break;//二档速度
		case 3:   pump_running_speed_send(-1000);							break;//三档速度
		case 4:   pump_enable_send(); vTaskDelay(600); pump_on_send();		break;//开始运行
		case 5:   pump_off_send(); vTaskDelay(300);	  pump_release_send();	break;//停止运行
		case 6:   pump_running_speed_send(-1500);							break;//四档速度
		case 7:   pump_running_speed_send(-2000);							break;//五档速度	
		case 8:   pump_running_speed_send(-2500);							break;//六档速度
		case 9:   pump_running_speed_send(-3000);							break;//七档速度		
		default:															break;
		
	}
}



//判断对应数据是否发送完成并设置对应标志位
static void dataSendFiniType(void)
{
	if((dataSendFinish == 1) && (dataSendType == 1))
	{
		PUMP_PRINTF("一档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 2))
	{
		PUMP_PRINTF("二档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成		
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 3))
	{
		PUMP_PRINTF("三档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 4))
	{
		PUMP_PRINTF("开始运行接收应答数据成功");
		pumpsta.run = 2; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 5))
	{
		PUMP_PRINTF("停止运行接收应答数据成功");
		pumpsta.stop = 2; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 6))
	{
		PUMP_PRINTF("四档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 7))
	{
		PUMP_PRINTF("五档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 8))
	{
		PUMP_PRINTF("六档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
	else if((dataSendFinish == 1) && (dataSendType == 9))
	{
		PUMP_PRINTF("七档速度接收应答数据成功");
		pumpsta.gear = 8; //应答完成	
		dataSendType = 0;
		dataSendFinish = 0;
	}
}



/**
 * @brief       控制电机数据发送循环函数
 * @param       无
 * @retval      返回 0（无动作）或 1（正确）或 2（过流）或 3（应答错误）4 （无应答）
 **/
void DumpUpdate(void)
{
	static	uint8_t cic = 255;		//等待串口发送完成最长时间
	switch(Pumptate) 
		{
			case STATE_IDLE:		//空闲状态
					if(PumpDataReady) 
					{	
						PumpDataReady = PUMPRESET;
						Pumptate = STATE_SENDING;
						RS485PUMP_ClearSerialBuffer(RS485PUMP_Usart);
						RS485PUMP_UsartTxflag = PUMPRESET;
						memset(RS485PUMP_Rxbff, 0, PUMPLEN * sizeof(char));		//先清空一下缓冲区
						JudgeDataSendType();		//判断发送哪一个数据
					}
					break;
			case STATE_SENDING:		//发送状态
					{
						if((RS485PUMP_UsartTxflag != PUMPSET) && (--cic))
						{
							if(cic == 0)
							{
								cic = 255;
								Pumptate = STATE_IDLE;
								PUMP_PRINTF("发送失败！！\r\n");
								PumpDataReady = PUMPRESET;
							}
						}
						else
						{
							PUMP_PRINTF("发送成功！！");
							cic = 0;
							Pumptate = STATE_WAITING_ACK;	
						}
					}
					break;				
			case STATE_WAITING_ACK:	//应答状态
					{
							if(RS485PUMP_UsartRxLen)
							{   
									RS485PUMP_CopySerialData(RS485PUMP_Usart,(char *)RS485PUMP_Rxbff,sizeof(RS485PUMP_Rxbff));	
									 /*数据打印处理*/
//									 Usart_Transmit(1,(char *)RS485PUMP_Rxbff,RS485PUMP_UsartRxLen);
//									 vTaskDelay(50);                                               /* 延时10ticks */
									if(!pump_modbus_crc_check((char *)RS485PUMP_Rxbff,RS485PUMP_UsartRxLen)) //检查校验		
									{
										 PUMP_PRINTF("PUMP接收应答数据成功");
										 PumpDataReady = PUMPRESET;
										 memset(RS485PUMP_Rxbff, 0, LEN * sizeof(char));
										 Pumptimout = 0;
										 RS485PUMP_ClearSerialBuffer(RS485PUMP_Usart);	
										 dataSendFiniType();		//接收完成处理状态机
										 Pumptate = STATE_IDLE;
									}
									else
									{	
										RxError++;	//接收的数据错误次数累计
										if(RxError >= 3)	//3次为接收错误
										{
											//通讯失败
											RxError = 0;	//清零处理
											memset(RS485PUMP_Rxbff, 0, LEN * sizeof(char));
											PumpDataReady = PUMPRESET;
											//错误处理返回给上位机
											PUMP_PRINTF("PUMP接收应答错误");
											pumpsta.gear = 10;	//应答错误
											pumpsta.run = 4;	//应答错误
											pumpsta.stop = 4;	//应答错误
											
										}
										else
										{
											 PumpDataReady = PUMPSET;
										}
										Pumptimout = 0;
										RS485PUMP_ClearSerialBuffer(RS485PUMP_Usart);	
										Pumptate = STATE_IDLE;
									}
							}
							else
							{
								Pumptimout++;
								if(Pumptimout >= 100)	//100ms为超时时间
								{	
									Pumptimout = 0; 	//定时器计时清零
									PUMP_PRINTF("接收超时");

									pumpsta.gear = 9;	//未应答
									pumpsta.run = 3;	//未应答
									pumpsta.stop = 3;	//未应答
									Pumptate = STATE_TIMEOUT;	//进入超时状态

								}		
							}
					}
					break;
			case STATE_TIMEOUT:		//超时状态
					{
						//超时处理返回给上位机
						#if	Debug
						printf("接收超时！！" );
						#endif
						PumpDataReady = PUMPRESET;
						Pumptate = STATE_IDLE;
					}
					break;
			default:	break;
		}
}










void pump_task_init(void)
{
	taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )PUMP_TASK,
                (const char*    )"PUMP_TASK",
                (uint16_t       )PUMP_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )PUMP_PRIO,
                (TaskHandle_t*  )&PUMPTask_Handler);
    taskEXIT_CRITICAL();            /* 退出临界区 */
}




/**
 * @brief       PUMP_TASK
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void PUMP_TASK(void *pvParameters)
{
	static uint8_t num = 10;
	vTaskDelay(1000);             
	while(pump_init() && (--num))
	{
		PUMP_PRINTF("蠕动泵通讯连接中！！！");
	}
	if(!num){PUMP_PRINTF("蠕动泵通讯失败！！！");}
	
	vTaskDelay(300);                                         /* 延时500ticks */
	pumpsta.gear= 4; //默认挡位设置为4
    while(1)
    {
		DetectionStatus();
		DumpUpdate();
		vTaskDelay(1);                                         /* 延时500ticks */

    }
}
