#ifndef __LED_TASK_H
#define __LED_TASK_H

#include "./SYSTEM/sys/sys.h"




#define Debug_Printf(...)       printf(__VA_ARGS__)

// 适配你的硬件参数（和能点亮灯的代码完全对齐）
#define WS2812_MAX_LED    77          // 最大支持10个灯珠（按需调整）
#define WS2812_BITS_PER_LED 24        // 每个灯珠24位(GRB)
#define WS2812_RESET_BITS 1           // 复位码长度（和你右侧代码25/49/73对齐）
#define WS2812_1_CODE     140          // 1码（高电平~800ns）
#define WS2812_0_CODE     60          // 0码（高电平~400ns）



#define LINK1_FLAG    (1 << 0)  // 第0位：Link1状态
#define LINK2_FLAG    (1 << 1)  // 第1位：Link2状态
#define FOOTCHECK_FLAG (1 << 2) // 第2位：footCheckFlag状态


void WS2812_Breathe_Control(uint8_t light_start, uint8_t light_end, 
                            uint8_t r_max, uint8_t g_max, uint8_t b_max,
                            uint8_t off1_start, uint8_t off1_end,
                            uint8_t off2_start, uint8_t off2_end);


// 状态组合说明（status十进制值 → 对应状态）
// 0 → 所有状态关闭（Link1=0、Link2=0、footCheckFlag=0）
// 1 → 仅Link1开启（Link1=1、Link2=0、footCheckFlag=0）
// 2 → 仅Link2开启（Link1=0、Link2=1、footCheckFlag=0）
// 3 → Link1+Link2开启（Link1=1、Link2=1、footCheckFlag=0）
// 4 → 仅footCheckFlag开启（Link1=0、Link2=0、footCheckFlag=1）
// 5 → Link1+footCheckFlag开启（Link1=1、Link2=0、footCheckFlag=1）
// 6 → Link2+footCheckFlag开启（Link1=0、Link2=1、footCheckFlag=1）
// 7 → 所有状态开启（Link1=1、Link2=1、footCheckFlag=1）
typedef	struct{

uint16_t	status;				//脚踏挡位 
	
}
WS2812_LED;

extern	WS2812_LED	ws2812led;

void led_task_init(void);

#endif








