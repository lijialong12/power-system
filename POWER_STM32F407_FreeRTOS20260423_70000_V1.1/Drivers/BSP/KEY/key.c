/**
 ****************************************************************************************************
 * @file        key.c
 * @author      李佳龙
 * @version     V1.0
 * @date        2025-5-15
 * @brief       按键扫描 驱动代码
 * @license     重庆博仕康科技有限公司
 ****************************************************************************************************
 **/

#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"
#include "includes.h"

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 
**/
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO配置参数存储变量 */
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
    KEY1_GPIO_CLK_ENABLE();                                     /* KEY1时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1引脚模式设置,上拉输入 */
	
    gpio_init_struct.Pin = KEY2_GPIO_PIN;                       /* KEY2引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);           /* KEY2引脚模式设置,上拉输入 */
	
}

/**
 * @brief       按键扫描函数
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 0, KEY0按下
 *              KEY1_PRES, 1, KEY1按下
 */
uint8_t key_scan(uint8_t mode, uint8_t gpio)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
	
	static	uint8_t tims = 0;
		
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* 支持连按 */
	
	switch(gpio)
	{
		case 0: 
		{
			if (key_up && (KEY0 == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
			{
				tims++;
				//vTaskDelay(20);                                           /* 延时1ticks */
				if(tims > 20)
				{
					key_up = 0;
					tims = 0;
				}

				if (KEY0 == 0)  keyval = KEY0_PRES;

			}
			else if (KEY0 == 1) /* 没有任何按键按下, 标记按键松开 */
			{
				key_up = 1;
			}
			break;
		}
		case 1:
		{
			if (key_up && (KEY1 == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
			{		
				tims++;
				//vTaskDelay(20);                                           /* 延时1ticks */
				if(tims > 20)
				{
					key_up = 0;
					tims = 0;
				}

				if (KEY1 == 0)  keyval = KEY1_PRES;

			}
			else if (KEY0 == 1) /* 没有任何按键按下, 标记按键松开 */
			{
				key_up = 1;
			}
			break;
		}
		case 2:
		{
			if (key_up && (KEY2 == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
			{
				tims++;
				//vTaskDelay(20);                                           /* 延时1ticks */
				if(tims > 20)
				{
					key_up = 0;
					tims = 0;
				}
				
				if (KEY2 == 0)  keyval = KEY2_PRES;

			}
			else if (KEY0 == 1) /* 没有任何按键按下, 标记按键松开 */
			{
				key_up = 1;
			}
			break;
		}
		
		default:break;
	}

    return keyval;              /* 返回键值 */ 
}




















