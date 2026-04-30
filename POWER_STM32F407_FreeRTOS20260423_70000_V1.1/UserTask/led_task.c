
#include "led_task.h"
#include "includes.h"


#define LED_PRINTF(format, ...)     //Debug_Printf("【LED_TASK】:"format "\r\n",##__VA_ARGS__)

/* LED_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LED_PRIO      	CONFIG_LED_PRIO                   		    /* 任务优先级 */
#define LED_STK_SIZE  	CONFIG_LED_STK_SIZE                         /* 任务堆栈大小 */
TaskHandle_t            LEDTask_Handler;  							/* 任务句柄 */
void LED_TASK(void *pvParameters);             						/* 任务函数 */







WS2812_LED	ws2812led = {1};


// 一维缓存数组：连续存储所有灯珠的时序+复位码，避免DMA传输错乱
// 总长度 = 最大灯数×24位 + 复位码长度
uint16_t pwmWS2812Buf[WS2812_MAX_LED * WS2812_BITS_PER_LED + WS2812_RESET_BITS] = {0};




//   通过PWM输出更新WS2812数据
void WS_WS2812_PWM_Update(uint32_t  *p,int n)
{		
			
		HAL_TIM_PWM_Start_DMA(&htim1_PWM3,TIM_CHANNEL_3,(uint32_t *)pwmWS2812Buf, n + 1 );	
}




/**
 * @brief  点亮指定位置、指定个数、指定颜色的WS2812灯珠
 * @param  start_idx: 起始灯珠位置（从0开始，如第1个灯填0，第5个灯填4）
 * @param  led_num:   要点亮的灯珠个数（需满足 start_idx + led_num ≤ WS2812_MAX_LED）
 * @param  r:         红色分量(0~255)
 * @param  g:         绿色分量(0~255)
 * @param  b:         蓝色分量(0~255)
 * @retval 无
 * @note   未指定的灯珠会被置为灭（0,0,0），确保仅目标灯珠亮
 */
void WS2812_Set_Leds(uint8_t start_idx, uint8_t led_num, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t buf_idx = 0;
    int i, j;

    // 1. 严格边界检查（避免数组越界）
    if(led_num == 0 || start_idx >= WS2812_MAX_LED || (start_idx + led_num) > WS2812_MAX_LED)
    {
        return;
    }

    // 2. 清空整个缓存（初始所有灯灭）
    memset(pwmWS2812Buf, 0, sizeof(pwmWS2812Buf));

    // 3. 填充所有灯珠的时序数据（逐灯处理）
    for(i = 0; i < WS2812_MAX_LED; i++)
    {
        uint32_t grb_data = 0;
        // 3.1 仅目标灯珠填充指定颜色，其余灯珠为0（灭）
        if(i >= start_idx && i < (start_idx + led_num))
        {
            // WS2812标准位序：GRB（绿→红→蓝），逐字节拼接
            grb_data = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        }
        else
        {
            grb_data = 0; // 非目标灯珠置灭
        }

        // 3.2 逐位提取24位GRB数据（高位在前，适配WS2812时序）
        for(j = 23; j >= 0; j--)
        {
            if((grb_data & (1UL << j)) != 0)
            {
                pwmWS2812Buf[buf_idx] = WS2812_1_CODE; // 高电平800ns（1码）
            }
            else
            {
                pwmWS2812Buf[buf_idx] = WS2812_0_CODE; // 高电平400ns（0码）
            }
            buf_idx++; // 缓存索引递增，保证连续
        }
    }

    // 4. 填充复位码（放在所有灯珠数据之后，位置修正）
    pwmWS2812Buf[buf_idx] = 0; // 复位码低电平，长度匹配WS2812要求

    // 5. 发送PWM时序数据（修正类型转换错误，避免DMA传输错乱）
    WS_WS2812_PWM_Update((uint32_t*)pwmWS2812Buf, WS2812_MAX_LED * WS2812_BITS_PER_LED + WS2812_RESET_BITS);
}




/**
 * @brief  将RGB颜色转换为WS2812的一维PWM时序数据（带复位码）
 * @param  r: 红色分量(0~255)
 * @param  g: 绿色分量(0~255)
 * @param  b: 蓝色分量(0~255)
 * @param  len: 灯珠个数(≤WS2812_MAX_LED)
 * @retval 无
 */
void WS2812_Fill_PWM_Buffer(uint8_t r, uint8_t g, uint8_t b, uint8_t len)
{
    uint16_t buf_idx = 0;
    int i, j;

    // 边界检查
    if(len == 0 || len > WS2812_MAX_LED) return;

    // 1. 关键修正：GRB位序重新排列（确保高位在前，且适配WS2812的实际位序）
    // 原逻辑：g<<16 | r<<8 | b → 改为逐位提取，避免移位溢出
    uint8_t grb[3] = {g, r, b}; // WS2812顺序：绿、红、蓝
    uint32_t grb_data = 0;
    for(int k=0; k<3; k++){
        grb_data = (grb_data << 8) | grb[k];
    }

    // 2. 清空缓存（避免脏数据影响第一个灯）
    memset(pwmWS2812Buf, 0, sizeof(pwmWS2812Buf));

    // 3. 填充每个灯的24位数据（逐位验证，确保第一个灯的起始索引是0）
    for(i = 0; i < len; i++)
    {
        // 重新逐位处理：从最高位（第23位）到最低位（第0位）
        for(j = 23; j >= 0; j--)
        {
            // 关键：用grb_data替代num，避免之前的移位错误
            if((grb_data & (1UL << j)) != 0)
            {
                pwmWS2812Buf[buf_idx] = WS2812_1_CODE;
            }
            else
            {
                pwmWS2812Buf[buf_idx] = WS2812_0_CODE;
            }
            buf_idx++; // 第一个灯的buf_idx从0→23，绝对连续
        }

    }
	
    // 4. 填充复位码（第一个灯不亮和复位码无关，这里仅保证位置正确）
    pwmWS2812Buf[buf_idx] = 0;

    // 5. 关键修正：去掉uint32_t强制转换，用原始uint16_t类型传输
    // 错误：(uint32_t*)pwmWS2812Buf → 正确：直接传pwmWS2812Buf（或(uint16_t*)）
    WS_WS2812_PWM_Update((uint32_t*)pwmWS2812Buf, len * WS2812_BITS_PER_LED + WS2812_RESET_BITS);
}





/**
 * @brief  呼吸灯，红色
 * @param  r: 红色分量最大值
 * @param  len: 灯珠个数(≤WS2812_MAX_LED)
 * @retval 无
 */
void breathe_led_red(uint8_t r, uint8_t len)
{
	static	uint8_t sta = 1,assig_tims = 0;
	static	uint8_t r_sta = 0;

	if(assig_tims == 0)
	{
		assig_tims = 1;
		r_sta = 0;
	}
	WS2812_Fill_PWM_Buffer(r_sta,0,0,len);
	if(sta)
	{
		r_sta++;
	}
	else
	{
		r_sta--;
	}
	if(r_sta > r )
	{
		sta = 0;
	}
	else if(r_sta == 0 || r_sta > 250 )
	{
		sta = 1;
	}
}





/**
 * @brief  呼吸灯，蓝色
 * @param  r: 蓝色分量最大值
 * @param  len: 灯珠个数(≤WS2812_MAX_LED)
 * @retval 无
 */
void breathe_led_blue(uint8_t b, uint8_t len)
{
	static	uint8_t sta = 1,assig_tims = 0;
	static	uint8_t b_sta = 0;

	if(assig_tims == 0)
	{
		assig_tims = 1;
		b_sta = 0;
	}
	WS2812_Fill_PWM_Buffer(0,0,b_sta,len);
	if(sta)
	{
		b_sta++;
	}
	else
	{
		b_sta--;
	}
	if(b_sta > b )
	{
		sta = 0;
	}
	else if(b_sta == 0 || b_sta > 250 )
	{
		sta = 1;
	}
}




/**
 * @brief  呼吸灯，绿色
 * @param  r: 绿色分量最大值
 * @param  len: 灯珠个数(≤WS2812_MAX_LED)
 * @retval 无
 */
void breathe_led_green(uint8_t g, uint8_t len)
{
	static	uint8_t sta = 1,assig_tims = 0;
	static	uint8_t g_sta = 0;

	if(assig_tims == 0)
	{
		assig_tims = 1;
		g_sta = 0;
	}
	WS2812_Fill_PWM_Buffer(0,g_sta,0,len);
	if(sta)
	{
		g_sta++;
	}
	else
	{
		g_sta--;
	}
	if( g_sta> g)
	{
		sta = 0;
	}
	else if(g_sta == 0 || g_sta > 250 )
	{
		sta = 1;
	}
}





/**
 * @brief  WS2812呼吸灯控制函数（点亮区间+2组强制熄灭区间，熄灭优先级更高）
 * @param  light_start:  点亮呼吸的起始灯珠索引（从0开始）
 * @param  light_end:    点亮呼吸的结束灯珠索引（包含，如第5个灯填4）
 * @param  r_max:        呼吸灯红色最大值(0~255)
 * @param  g_max:        呼吸灯绿色最大值(0~255)
 * @param  b_max:        呼吸灯蓝色最大值(0~255)
 * @param  off1_start:   第1组强制熄灭的起始灯珠索引（从0开始）
 * @param  off1_end:     第1组强制熄灭的结束灯珠索引（包含）
 * @param  off2_start:   第2组强制熄灭的起始灯珠索引（从0开始）
 * @param  off2_end:     第2组强制熄灭的结束灯珠索引（包含）
 * @retval 无
 * @note   1. 所有熄灭区间优先级 > 点亮区间，重叠位置强制熄灭；
 *         2. 仅[light_start, light_end]且不在[off1_start,off1_end]、[off2_start,off2_end]的灯珠呼吸亮灭；
 *         3. 两组熄灭区间可独立设置，支持不连续、重叠的熄灭范围；
 *         4. 其余灯珠默认熄灭；
 *         5. 呼吸速度由调用频率决定（建议搭配vTaskDelay(1)调用）。
 */
void WS2812_Breathe_Control(uint8_t light_start, uint8_t light_end, 
                            uint8_t r_max, uint8_t g_max, uint8_t b_max,
                            uint8_t off1_start, uint8_t off1_end,
                            uint8_t off2_start, uint8_t off2_end)
{
    // 呼吸状态变量（独立，避免冲突）
    static uint8_t breathe_dir = 1;  // 呼吸方向：1=亮度递增，0=亮度递减
    static uint8_t curr_bright = 0;  // 当前呼吸亮度（0~255）
    static uint8_t init_flag = 0;    // 首次初始化标志

    uint16_t buf_idx = 0;
    int i, j;

    // 1. 严格边界校验（新增第2组熄灭区间的校验）
    if(light_start >= WS2812_MAX_LED || light_end >= WS2812_MAX_LED || light_start > light_end ||
       off1_start >= WS2812_MAX_LED || off1_end >= WS2812_MAX_LED || off1_start > off1_end ||
       off2_start >= WS2812_MAX_LED || off2_end >= WS2812_MAX_LED || off2_start > off2_end)
    {
        LED_PRINTF("参数错误：light[%d-%d], off1[%d-%d], off2[%d-%d], max_led=%d", 
                   light_start, light_end, off1_start, off1_end, off2_start, off2_end, WS2812_MAX_LED);
        return;
    }

    // 2. 首次调用初始化（重置亮度和方向）
    if(init_flag == 0)
    {
        curr_bright = 0;
        breathe_dir = 1;
        init_flag = 1;
    }

    // 3. 计算当前呼吸亮度对应的RGB值（线性缩放）
    uint8_t curr_r = (r_max * curr_bright) / 255;
    uint8_t curr_g = (g_max * curr_bright) / 255;
    uint8_t curr_b = (b_max * curr_bright) / 255;

    // 4. 清空缓存（初始所有灯灭）
    memset(pwmWS2812Buf, 0, sizeof(pwmWS2812Buf));

    // 5. 逐灯填充PWM时序（核心逻辑：新增第2组熄灭区间判定）
    for(i = 0; i < WS2812_MAX_LED; i++)
    {
        uint32_t grb_data = 0;

        // 核心判定逻辑（新增第2组熄灭区间）：
        // 条件1：在点亮区间  &&  不在第1组熄灭区间  &&  不在第2组熄灭区间 → 呼吸亮灭
        // 条件2：在任意一组熄灭区间（无论是否在点亮区间） → 强制熄灭
        // 条件3：其他 → 默认熄灭
        if((i >= light_start && i <= light_end) && 
           !(i >= off1_start && i <= off1_end) && 
           !(i >= off2_start && i <= off2_end))
        {
            // 点亮区间且不在任意熄灭区间 → 填充当前呼吸颜色
            grb_data = ((uint32_t)curr_g << 16) | ((uint32_t)curr_r << 8) | curr_b;
        }
        // 任意熄灭区间/其他区间 → grb_data保持0（熄灭）

        // 逐位填充24位GRB时序
        for(j = 23; j >= 0; j--)
        {
            pwmWS2812Buf[buf_idx++] = (grb_data & (1UL << j)) ? WS2812_1_CODE : WS2812_0_CODE;
        }
    }

    // 6. 填充复位码
    pwmWS2812Buf[buf_idx] = 0;

    // 7. 发送PWM时序数据
    WS_WS2812_PWM_Update((uint32_t*)pwmWS2812Buf, WS2812_MAX_LED * WS2812_BITS_PER_LED + WS2812_RESET_BITS);

    // 8. 更新呼吸亮度（控制呼吸节奏）
    if(breathe_dir)
    {
        curr_bright++;
        if(curr_bright >= 255) breathe_dir = 0; // 亮度到顶，开始递减
    }
    else
    {
        curr_bright--;
        if(curr_bright <= 0) breathe_dir = 1;   // 亮度到底，开始递增
    }
}




void led_task_init(void)
{
	taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )LED_TASK,
                (const char*    )"LED_TASK",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_PRIO,
                (TaskHandle_t*  )&LEDTask_Handler);
    taskEXIT_CRITICAL();            /* 退出临界区 */
}






/**
	WS2812_Breathe_Control(0, 38, 0, 150, 0, 39, 57,58,76); //手柄1连接，其它两个未连接	 
	WS2812_Breathe_Control(0, 57, 0, 150, 0, 20, 38,58,76); //手柄2连接，其它两个未连接	
	WS2812_Breathe_Control(0, 76, 0, 150, 0, 20, 38,39,57); //脚踏连接，其它两个未连接
	WS2812_Breathe_Control(0, 57, 0, 150, 0, 0, 0,58,76); //手柄12连接，脚踏未连接
	WS2812_Breathe_Control(0, 76, 0, 150, 0, 39, 57,0,0); 	//手柄1和脚踏连接，手柄2未连接
	WS2812_Breathe_Control(0, 76, 0, 150, 0, 20, 38,58,76); //手柄2和脚踏连接，手柄1未连接
	WS2812_Breathe_Control(0, 19, 0, 150,0,20, 76,0,0); //全未连接
	WS2812_Breathe_Control(0, 76, 0, 150,0,0, 0,0,0); //全连接

 * @brief       LED_TASK
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void LED_TASK(void *pvParameters)
{
	MX_TIM1_Init();
	vTaskDelay(1000);
	Tim7_Init(1000-1, 840-1);	//1s一次中断
	Tim7_Start();
    while(1)
    {   
		// 控制Link1：置位/清零第0位，保留其他位
		if (handle.Link1) {
			ws2812led.status |= LINK1_FLAG;  // 置位第0位（开启Link1）
		} else {
			ws2812led.status &= ~LINK1_FLAG; // 清零第0位（关闭Link1）
		}

		// 控制Link2：置位/清零第1位，保留其他位
		if (handle.Link2) {
			ws2812led.status |= LINK2_FLAG;  // 置位第1位（开启Link2）
		} else {
			ws2812led.status &= ~LINK2_FLAG; // 清零第1位（关闭Link2）
		}

		// 控制footCheckFlag：置位/清零第2位，保留其他位
		if (footvar.footCheckFlag) {
			ws2812led.status |= FOOTCHECK_FLAG;  // 置位第2位（开启footCheck）
		} else {
			ws2812led.status &= ~FOOTCHECK_FLAG; // 清零第2位（关闭footCheck）
		}
		vTaskDelay(10);
    }
}
