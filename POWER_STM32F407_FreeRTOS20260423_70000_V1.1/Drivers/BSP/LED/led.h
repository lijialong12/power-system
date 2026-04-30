#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 定义 */

#define LED0_GPIO_PORT                  GPIOB
#define LED0_GPIO_PIN                   GPIO_PIN_13

#define LED1_GPIO_PORT                  GPIOB
#define LED1_GPIO_PIN                   GPIO_PIN_14

#define LED2_GPIO_PORT                  GPIOB
#define LED2_GPIO_PIN                   GPIO_PIN_15


#define LED_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */

#define RS485_RE_GPIO_PORT                  GPIOG
#define RS485_RE_GPIO_PIN                   GPIO_PIN_8
#define RS485_RE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */
/******************************************************************************************/

/* LED端口定义 */

/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */
#define RS485_RE(x)   do{ x ? \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
                      }while(0)

#define	LED0														PBout(13)
#define	LED1														PBout(14)
#define	LED2														PBout(15)
#define	BEEP														PBout(12)
					  
																	
																	
#define	BEEP_ON										do{ BEEP = 1;\
														vTaskDelay(50);BEEP = 0;\
													  }while(0)
/******************************************************************************************/
/* 外部接口函数*/
void led_init(void);                                                                            /* 初始化 */
#endif
