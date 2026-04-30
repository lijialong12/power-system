/**
 ****************************************************************************************************
 * @file        exti.c
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-5-15
 * @brief       外部中断 驱动代码
 * @license     重庆博仕康科技有限公司
 ****************************************************************************************************
 **/

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/TIM/TIM.h"
#include "./BSP/USART/USART.h"
#include "includes.h"

//调试口开关
#define   Debug     0




/**
 * @brief       KEY1 外部中断服务程序
 * @param       无
 * @retval      无
 */
void KEY_INT_IRQHandler(void)
{ 
	#if SYS_SUPPORT_OS                              /* 使用OS */
	uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
	#endif
    HAL_GPIO_EXTI_IRQHandler(KEY0_INT_GPIO_PIN);         /* 调用中断处理公用函数 清除KEY0所在中断线 的中断标志位 */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY0_INT_GPIO_PIN);         /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
	
    HAL_GPIO_EXTI_IRQHandler(KEY1_INT_GPIO_PIN);         /* 调用中断处理公用函数 清除KEY1所在中断线 的中断标志位，中断下半部在HAL_GPIO_EXTI_Callback执行 */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);         /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif
}

/**
 * @brief       中断服务程序中需要做的事情
 *              在HAL库中所有的外部中断服务函数都会调用此函数
 * @param       GPIO_Pin:中断引脚号
 * @retval      无
 */
uint16_t BistTimFlag = 0;	//电机按键触发状态机
uint16_t PumpTimFlag = 0;	//脚踏按键触发状态机

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	#if Debug	// 如果Debug已定义为1，编译此部分代码
	printf("检测到按键按下\r\n");
	#endif                                           /* 延时20ticks */
    switch(GPIO_Pin)
    {
        case KEY0_INT_GPIO_PIN:
            if (KEY0 == 0)
            {
				BistTimFlag = 0;
				#if Debug	// 如果Debug已定义为1，编译此部分代码
				//printf("按键1  \r\n");	
				#endif                                           /* 延时20ticks */
				Tim2_Start();
				footvar.EXTI_footFlag = 1;
            }
            break;

        case KEY1_INT_GPIO_PIN:
            if (KEY1 == 0)
            {
				PumpTimFlag = 0;
				Tim4_Start();
				footvar.EXTI_pumpFlag = 1;
				#if Debug	// 如果Debug已定义为1，编译此部分代码
				//printf("按键2  \r\n");	
				#endif  

            }
            break;

        default : break;
    }
}



/**
 * @brief       外部中断初始化程序
 * @param       无
 * @retval      无
 */
void extix_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    key_init();
    gpio_init_struct.Pin = KEY0_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* 上升沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;					/* 默认下拉 */
    HAL_GPIO_Init(KEY0_INT_GPIO_PORT, &gpio_init_struct);    /* KEY0配置为上升沿触发中断 */

    gpio_init_struct.Pin = KEY1_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* 上升沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;					/* 默认下拉 */
    HAL_GPIO_Init(KEY1_INT_GPIO_PORT, &gpio_init_struct);    /* KEY1配置为上升沿触发中断 */

    HAL_NVIC_SetPriority(KEY_INT_IRQn, 10, 10);               /* 抢占0，子优先级2 */
    HAL_NVIC_EnableIRQ(KEY_INT_IRQn);                       /* 使能中断线4 */

}












