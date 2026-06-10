
/**
 ****************************************************************************************************
 * @file        main.c
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-12-15
 * @brief       动力系统
 * @license    	重庆博仕康科技有限公司
 ****************************************************************************************************
 * @attention
 * 修改说明
 * V1.0 20250805
 * 第一次发布
 *
 ****************************************************************************************************
 */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/SRAM/sram.h"
#include "./MALLOC/malloc.h"
#include "./BSP/LED/led.h"
/*FreeRTOS*********************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "includes.h"
#include "./BSP/USART/USART.h"
#include "./BSP/KEY/key.h"
/******************************************************************************************************/
/*FreeRTOS配置*/

/* START_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define START_TASK_PRIO 	CONFIG_START_PRIO                  	  /* 任务优先级 */
#define START_STK_SIZE  	CONFIG_START_STK_SIZE                 /* 任务堆栈大小 */
TaskHandle_t            StartTask_Handler;  					  /* 任务句柄 */
void start_task(void *pvParameters);        					  /* 任务函数 */


/******************************************************************************************************/



// 堆栈溢出监测 钩子
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    //if( pcTaskName!=NULL ) printf("!!!任务:%s 堆栈溢出!!!\r\n", pcTaskName);
}





IWDG_HandleTypeDef iwdg_handler; /*独立看门狗句柄 */

/**
 * @brief       初始化独立看门狗 
 * @param       prer: IWDG_PRESCALER_4~IWDG_PRESCALER_256,对应4~256分频
 *   @arg       分频因子 = 4 * 2^prer. 但最大值只能是256!
 * @param       rlr: 自动重装载值,0~0XFFF. 
 * @note        时间计算(大概):Tout=((4 * 2^prer) * rlr) / 32 (ms). 
 * @retval      无
 */
void iwdg_init(uint32_t prer, uint16_t rlr)
{
    iwdg_handler.Instance = IWDG;
    iwdg_handler.Init.Prescaler = prer; /* 设置IWDG分频系数 */
    iwdg_handler.Init.Reload = rlr;     /* 从加载寄存器 IWDG->RLR 重装载值 */
    HAL_IWDG_Init(&iwdg_handler);       /* 初始化IWDG并使能 */
}

/**
 * @brief       喂独立看门狗
 * @param       无
 * @retval      无
 */
void iwdg_feed(void)
{
    HAL_IWDG_Refresh(&iwdg_handler);    /* 喂狗 */
}





int main(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
    delay_init(168);                    /* 延时初始化 */
//	Usart1_Dma2_Init(115200);			/*	调试口	*/
	my_mem_init(SRAMIN);                /* 初始化内部SRAM内存池 */
	my_mem_init(SRAMCCM);               /* 初始化内部CCM内存池 */
	led_init(); 
 
    xTaskCreate((TaskFunction_t )start_task,            /* 任务函数 */
                (const char*    )"start_task",          /* 任务名称 */
                (uint16_t       )START_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )START_TASK_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&StartTask_Handler);   /* 任务句柄 */
    vTaskStartScheduler();
   
}


/******************************************************************************************************/



/**
 * @brief       start_task
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void start_task(void *pvParameters)
{
	dynsys_task_init();
	hmi_task_init();
	pump_task_init();
	foot_task_init();
	led_task_init();
	handle_1_task_init();
	handle_2_task_init();
	pc_task_init();
	iwdg_init(IWDG_PRESCALER_64,2000);
	while(1)
	{
		/* LED0闪烁 */
		LED0 = 1;    
		LED1 = 1;
		LED2 = 1;
		vTaskDelay(500);                                           /* 延时1000ticks */
		LED0 = 0;    
		LED1 = 0;
		LED2 = 0;
		vTaskDelay(500);                                           /* 延时1000ticks */
		iwdg_feed();		//喂狗时间
	}

}

