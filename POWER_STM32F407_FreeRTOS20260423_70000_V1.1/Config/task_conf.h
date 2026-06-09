
/**********************************************************************************************************
*
*                                      应用程序配置
*                                    
*                   主要是配置个任务的优先级以及堆栈大小
**********************************************************************************************************/

#ifndef  __TASK_CFG_H__
#define  __TASK_CFG_H__

/*********************************************************************************************************
*                              任务优先级-数值越大优先级越高
*								前3个任务优先级不可更改
*********************************************************************************************************/
#define CONFIG_START_PRIO      						1       //开始任务的优先级设置

#define CONFIG_DYNSYS_PRIO              			22   	//动力系统任务	
	
#define CONFIG_LED_PRIO                 			7   	//led任务	
#define CONFIG_PUMP_PRIO                 			18   	//蠕动泵任务
#define CONFIG_FOOT_PRIO                 			19   	//脚踏任务
#define CONFIG_HMI_PRIO               				16   	//陶晶池人机交互	
#define CONFIG_HANDLE_1_PRIO                 		6   	//手柄1任务
#define CONFIG_PC_PRIO                 				5   	//PC任务	
#define CONFIG_HANDLE_2_PRIO                 		6   	//手柄2任务	



/********************************************************************************************************
*                                            堆栈大小
*                             Size of the task stacks (# of OS_STK entries)
********************************************************************************************************/

#define CONFIG_START_STK_SIZE  					128
#define CONFIG_DYNSYS_STK_SIZE                  256
#define CONFIG_HMI_STK_SIZE                  	256
#define CONFIG_LED_STK_SIZE                 	128
#define CONFIG_PUMP_STK_SIZE                 	256
#define CONFIG_FOOT_STK_SIZE                 	256
#define CONFIG_HANDLE_1_STK_SIZE                256
#define CONFIG_PC_STK_SIZE                 		128
#define CONFIG_HANDLE_2_STK_SIZE                256

#endif

/*******************************************END**********************************************/

