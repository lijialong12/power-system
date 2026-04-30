/**
 ****************************************************************************************************
 * @file        pwmdac.h
 * @author      LIJIALONG
 * @version     V1.0
 * @date        2021-11-03
 * @brief       PWM 输出 驱动代码
 ****************************************************************************************************
 */

#ifndef __PWM_H
#define __PWM_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* PWM 引脚 和 定时器 定义 */

/* PWM 默认是使用 PE14, 对应的定时器为 TIM1_CH3, 如果你要修改成其他IO输出, 则相应
 * 的定时器及通道也要进行修改. 请根据实际情况进行修改.
 */
#define PWM_GPIO_PORT                    GPIOE
#define PWM_GPIO_PIN                     GPIO_PIN_13
#define PWM_GPIO_CLK_ENABLE()            do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)     /* PE口时钟使能 */
#define PWM_GPIO_AFTIMX                  GPIO_AF1_TIM1

#define PWM_TIMX                         TIM1
#define PWM_TIMX_CHY                     TIM_CHANNEL_3                                   /* 通道Y,  1<= Y <=4 */
#define PWM_TIMX_CCRX                    PWMDAC_TIMX->CCR2                               /* 通道Y的输出比较寄存器 */
#define PWM_TIMX_CLK_ENABLE()            do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)      /* TIM1 时钟使能 */

/******************************************************************************************/

void pwm_init(uint16_t arr, uint16_t psc);   /* PWM初始化 */
void pwm_set(uint16_t vol);          	     /* PWM设置输出电压 */

#endif






