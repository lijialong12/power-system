/**
 ****************************************************************************************************
 * @file        pwm.c
 * @author      LIJIALONG
 * @version     V1.0
 * @date        2025-6-26
 * @brief       PWM 输出 驱动代码
 * @license     
 ****************************************************************************************************
 */

#include "./BSP/PWM/PWM.h"


TIM_HandleTypeDef g_tim1_handler;     /* 定时器句柄 */
TIM_OC_InitTypeDef g_tim1_ch4handler; /* 定时器1通道4句柄 */

/**
 * @brief       PWM初始化, 实际上就是初始化定时器
 * @note
 *              定时器的时钟来自APB1 / APB2, 当APB1 / APB2 分频时, 定时器频率自动翻倍
 *              所以, 一般情况下, 我们所有定时器的频率, 都是84Mhz 等于系统时钟频率
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft = 定时器工作频率, 单位: Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void pwm_init(uint16_t arr, uint16_t psc)
{
    g_tim1_handler.Instance = PWM_TIMX;                  /* 定时器9 */
    g_tim1_handler.Init.Prescaler = psc;                    /* 定时器分频 */
    g_tim1_handler.Init.CounterMode = TIM_COUNTERMODE_UP;   /* 向上计数模式 */
    g_tim1_handler.Init.Period = arr;                       /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_tim1_handler);                      /* 初始化PWM */

    g_tim1_ch4handler.OCMode = TIM_OCMODE_PWM1;                                         /* CH1/2 PWM模式1 */
    g_tim1_ch4handler.Pulse = 0;                                                  /* 设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半，即占空比为50% */
    g_tim1_ch4handler.OCPolarity = TIM_OCPOLARITY_LOW;                                 /* 输出比较极性为高 */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_handler, &g_tim1_ch4handler, PWM_TIMX_CHY);    /* 配置TIM2通道4 */

    HAL_TIM_PWM_Start(&g_tim1_handler, PWM_TIMX_CHY);    /* 开启PWM通道4 */
}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
 * @note
 *              此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (htim->Instance == PWM_TIMX)
    {
        PWM_TIMX_CLK_ENABLE();           /* 使能定时器 */
        PWM_GPIO_CLK_ENABLE();           /* PWM GPIO 时钟使能 */

        gpio_init_struct.Pin = PWM_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init_struct.Alternate = PWM_GPIO_AFTIMX;
        HAL_GPIO_Init(PWM_GPIO_PORT, &gpio_init_struct);         /* TIMX PWM CHY 引脚模式设置 */
    }
}

/**
 * @brief       设置PWM 占空比输出
 * @param       temp : 0~200
 * @retval      无
 */
void pwm_set(uint16_t temp)
{
    __HAL_TIM_SET_COMPARE(&g_tim1_handler, PWM_TIMX_CHY, temp);  /* 设置新的占空比 */
}



