
 
 #include "./BSP/LED/led.h"

/**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    LED_GPIO_CLK_ENABLE();  /* LED0时钟使能 */ 
	__HAL_RCC_GPIOC_CLK_ENABLE();		
	__HAL_RCC_GPIOA_CLK_ENABLE();
    RS485_RE_GPIO_CLK_ENABLE(); /* 使能 RS485_RE 脚时钟 */
	
    gpio_init_struct.Pin = LED0_GPIO_PIN | LED1_GPIO_PIN | LED2_GPIO_PIN;                   /* LED0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* 初始化LED0引脚 */
	
	//屏幕power使能引脚初始化
    gpio_init_struct.Pin = GPIO_PIN_7 ;                   /* LED0引脚 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);       /* 初始化LED0引脚 */
	
	//蜂鸣器使能引脚初始化
    gpio_init_struct.Pin = GPIO_PIN_12 ;                   /* BEEP引脚 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化LED0引脚 */
	
	//电机选择和使能引脚初始化
    gpio_init_struct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;                   /* LED0引脚 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                    /* 上拉 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);       /* 初始化LED0引脚 */
	
	//屏幕power使能引脚
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	
	//电机选择和使能引脚
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	
    gpio_init_struct.Pin = RS485_RE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_RE_GPIO_PORT, &gpio_init_struct);       /* RS485_RE 脚 模式设置 */

	//LED灯
    LED0 = 0;
	LED1 = 0;
	LED2 = 0;
}
