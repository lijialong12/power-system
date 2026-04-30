/**
 ****************************************************************************************************
 * @file        exti.h
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-5-15
 * @brief       外部中断 驱动代码
 * @license     重庆博仕康科技有限公司
 ****************************************************************************************************
 **/

#ifndef __EXTI_H
#define __EXTI_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 和 中断编号 & 中断服务函数 定义 */ 

#define KEY0_INT_GPIO_PORT              GPIOB
#define KEY0_INT_GPIO_PIN               GPIO_PIN_7
#define KEY0_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY1_INT_GPIO_PORT              GPIOB
#define KEY1_INT_GPIO_PIN               GPIO_PIN_6
#define KEY1_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
#define KEY_INT_IRQn                    EXTI9_5_IRQn
#define KEY_INT_IRQHandler              EXTI9_5_IRQHandler


/******************************************************************************************/
extern uint16_t BistTimFlag;
extern uint16_t PumpTimFlag;
void keyscan(void);
void extix_init(void);  /* 外部中断初始化 */

#endif

























