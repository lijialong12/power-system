
/**
 ****************************************************************************************************
 * @file        hmi.c
 * @author      lijialong
 * @version     V1.0
 * @date        2025-7-18
 * @brief       大彩串口屏 驱动代码
 * @license     重庆博仕康科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 修改说明
 * V1.0 20250718
 * 第一次发布
 *
 ****************************************************************************************************
 **/

#include "hmi_task.h"
#include "./BSP/USART/USART.h"
#include "./SYSTEM/delay/delay.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>  // 为了使用 memset 函数
#include "./BSP/TIM/TIM.h"
#include "./BSP/BUZZER/BEEP.h"
#include <stdbool.h>
#include "includes.h"
#include "./MALLOC/malloc.h"


//调试口开关
#define HMI_PRINTF(format, ...)     //Debug_Printf("【HMI_TASK】:"format "\r\n",##__VA_ARGS__)



// ===================== 宏定义 =====================
#define SLIDE_LEFT     1		//界面已经左滑标志
#define SLIDE_RIGHT    2		//界面已经右滑标志


// ===================== 全局变量（你定义的，不动） =====================
volatile uint8_t g_slide_flag = SLIDE_LEFT;      // 滑动标志位：1=左滑，2=右滑
uint16_t g_timer_3s = 0;        // 3秒定时器变量（外部递减，0=计时完成）
uint16_t p_timer_3s = 0;        // 3秒定时器变量（外部递减，0=计时完成）
volatile uint8_t timer_busy = 0;  // 0=空闲 1=电源占用 2=挡位占用

HMIVAR	hmivar = {1,70000,70000,3500,4,0,1,0,0};



/* HMI_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define HMI_PRIO      	CONFIG_HMI_PRIO                   		    /* 任务优先级 */
#define HMI_STK_SIZE  	CONFIG_HMI_STK_SIZE                         /* 任务堆栈大小 */
TaskHandle_t            HMITask_Handler;  							/* 任务句柄 */
void HMI_TASK(void *pvParameters);             						/* 任务函数 */


uint8_t hmi_Rxbff[LEN];



const unsigned char Grinding[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x09,0x03,0xFF,0xFC,0xFF,0xFF};	//磨砖模式

const unsigned char Planing[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x09,0x01,0xFF,0xFC,0xFF,0xFF};	//刨削模式


const unsigned char sensitivity_5[] = {0xEE,0x8A,0x5A,0xA5,0x04,0xFF,0xFC,0xFF,0xFF};	//触控5档灵敏度

const unsigned char forw_rotup[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x02,0x00,0xFF,0xFC,0xFF,0xFF};		//正转按钮弹起
const unsigned char forw_rotdown[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x02,0x01,0xFF,0xFC,0xFF,0xFF};	//正转按钮按下

const unsigned char reve_rotup[] = {0xEE,0xB1,0x10,0x00,0x03,0x00,0x03,0x00,0xFF,0xFC,0xFF,0xFF};		//反转按钮弹起
const unsigned char reve_rotdown[] = {0xEE,0xB1,0x10,0x00,0x03,0x00,0x03,0x01,0xFF,0xFC,0xFF,0xFF};	//反转按钮按下


const unsigned char both_sides_rotup[] = {0xEE,0xB1,0x10,0x00,0x03,0x00,0x02,0x00,0xFF,0xFC,0xFF,0xFF};	//正反转按钮弹起
const unsigned char both_sides_rotdown[] = {0xEE,0xB1,0x10,0x00,0x03,0x00,0x02,0x01,0xFF,0xFC,0xFF,0xFF};	//正反转按钮按下


const unsigned char power_gear_0_off[] 	 = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x00,0xFF,0xFC,0xFF,0xFF};	//动力挡位0档未激活
const unsigned char power_gear_1[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x01,0xFF,0xFC,0xFF,0xFF};		//动力挡位1档
const unsigned char power_gear_2[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x02,0xFF,0xFC,0xFF,0xFF};		//动力挡位2档
const unsigned char power_gear_3[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x03,0xFF,0xFC,0xFF,0xFF};		//动力挡位3档
const unsigned char power_gear_4[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x04,0xFF,0xFC,0xFF,0xFF};		//动力挡位4档
const unsigned char power_gear_5[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x05,0xFF,0xFC,0xFF,0xFF};		//动力挡位5档
const unsigned char power_gear_6[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x06,0xFF,0xFC,0xFF,0xFF};		//动力挡位6档
const unsigned char power_gear_7[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x07,0xFF,0xFC,0xFF,0xFF};		//动力挡位7档
const unsigned char power_gear_8[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x05,0x08,0xFF,0xFC,0xFF,0xFF};		//动力挡位8档


const unsigned char power_off[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x06,0x00,0xFF,0xFC,0xFF,0xFF};	//电机停止弹起
const unsigned char power_on[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x06,0x01,0xFF,0xFC,0xFF,0xFF};	//电机运行按下

const unsigned char foot_off[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x08,0x00,0xFF,0xFC,0xFF,0xFF};		//脚踏未插入
const unsigned char foot_on[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x08,0x01,0xFF,0xFC,0xFF,0xFF};		//脚踏插入

const unsigned char pump_on[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x07,0x01,0xFF,0xFC,0xFF,0xFF}; 		//蠕动泵连接
const unsigned char pump_off[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x07,0x00,0xFF,0xFC,0xFF,0xFF}; 		//蠕动泵未连接


const unsigned char pumpgear1_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x00,0xFF,0xFC,0xFF,0xFF};	//蠕动泵一档关闭
const unsigned char pumpgear2_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x01,0xFF,0xFC,0xFF,0xFF};		//蠕动泵一档
const unsigned char pumpgear3_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x02,0xFF,0xFC,0xFF,0xFF};		//蠕动泵二档
const unsigned char pumpgear4_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x03,0xFF,0xFC,0xFF,0xFF};		//蠕动泵三档
const unsigned char pumpgear5_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x04,0xFF,0xFC,0xFF,0xFF};		//蠕动泵四档
const unsigned char pumpgear6_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x05,0xFF,0xFC,0xFF,0xFF};		//蠕动泵五档
const unsigned char pumpgear7_off[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x06,0xFF,0xFC,0xFF,0xFF};		//蠕动泵六档
const unsigned char pumpgear1[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x07,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear2[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x08,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear3[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x09,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear4[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x0A,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear5[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x0B,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear6[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x0C,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档
const unsigned char pumpgear7[] = {0xEE,0xB1,0x23,0x00,0x03,0x00,0x05,0x0D,0xFF,0xFC,0xFF,0xFF};		//蠕动泵七档


const unsigned char handejoinoff[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x0A,0x00,0xFF,0xFC,0xFF,0xFF};	//手柄未连接
const unsigned char handejoin1[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x0A,0x01,0xFF,0xFC,0xFF,0xFF};		//手柄1连接
const unsigned char handejoin2[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x0A,0x02,0xFF,0xFC,0xFF,0xFF};		//手柄2连接
const unsigned char handejoin3[] = {0xEE,0xB1,0x23,0x00,0x02,0x00,0x0A,0x03,0xFF,0xFC,0xFF,0xFF};		//手柄全连接

const unsigned char init_prog_30[] = {0xEE, 0xB1, 0x10, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xFC, 0xFF, 0xFF}; //进度条30%
const unsigned char init_prog_60[] = {0xEE, 0xB1, 0x10, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x3C, 0xFF, 0xFC, 0xFF, 0xFF}; //进度条60%
const unsigned char init_prog_70[] = {0xEE, 0xB1, 0x10, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x46, 0xFF, 0xFC, 0xFF, 0xFF}; //进度条70%
const unsigned char init_prog_100[] = {0xEE, 0xB1, 0x10, 0x00, 0x01,0x00, 0x02, 0x00, 0x00, 0x00, 0x64, 0xFF, 0xFC, 0xFF, 0xFF};		//进度条100%

const unsigned char power_number[] = {0xEE,0xB1,0x10,0x00,0x00,0x00,0x05,0x34,0x30,0x30,0x30,0x30,0xFF,0xFC,0xFF,0xFF};	//转速显示

const unsigned char interface_left1[] = {0xEE,0xB1,0x80,0x00,0x00,0x00,0x09,0xFF,0xFC,0xFF,0xFF};	//左边子界面滑动到第一页界面
const unsigned char interface_left2[] = {0xEE,0xB1,0x81,0x00,0x00,0x00,0x09,0xFF,0xFC,0xFF,0xFF};	//左边子界面滑动到第二页界面

const unsigned char interface_right1[] = {0xEE,0xB1,0x80,0x00,0x00,0x00,0x0C,0xFF,0xFC,0xFF,0xFF};	//右边子界面滑动到第一页界面
const unsigned char interface_right2[] = {0xEE,0xB1,0x81,0x00,0x00,0x00,0x0C,0xFF,0xFC,0xFF,0xFF};	//右边子界面滑动到第二页界面

const unsigned char Sinterslider_left1[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x1B,0x00,0xFF,0xFC,0xFF,0xFF};	//左边子界面滑条到第一页界面
const unsigned char Sinterslider_left2[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x1B,0x01,0xFF,0xFC,0xFF,0xFF};	//左边子界面滑条到第二页界面

const unsigned char Sinterslider_right1[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x08,0x00,0xFF,0xFC,0xFF,0xFF};	//右边子界面滑条到第一页界面
const unsigned char Sinterslider_right2[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x08,0x01,0xFF,0xFC,0xFF,0xFF};	//右边子界面滑条到第二页界面

const unsigned char hande2_button[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x0B,0x01,0xFF,0xFC,0xFF,0xFF};	//手柄2
const unsigned char hande1_button[] = {0xEE,0xB1,0x10,0x00,0x02,0x00,0x0B,0x00,0xFF,0xFC,0xFF,0xFF};	//手柄1

const unsigned char fault_clear[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x09,0xFF,0xFC,0xFF,0xFF};	//清除所有故障

const unsigned char fault_pump1[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x00,0xFF,0xFC,0xFF,0xFF};	//水泵故障01
const unsigned char fault_pump2[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x01,0xFF,0xFC,0xFF,0xFF};	//水泵故障02
const unsigned char fault_motor1[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x02,0xFF,0xFC,0xFF,0xFF};	//电机故障01
const unsigned char fault_motor2[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x03,0xFF,0xFC,0xFF,0xFF};	//电机故障02
const unsigned char fault_motor3[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x04,0xFF,0xFC,0xFF,0xFF};	//电机故障03
const unsigned char fault_motor4[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x05,0xFF,0xFC,0xFF,0xFF};	//电机故障04
const unsigned char fault_motor5[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x06,0xFF,0xFC,0xFF,0xFF};	//电机故障05
const unsigned char fault_motor6[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x07,0xFF,0xFC,0xFF,0xFF};	//电机故障06
const unsigned char fault_motor7[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x06,0x08,0xFF,0xFC,0xFF,0xFF};	//电机过流

//通讯初始化
uint8_t hmi_init(void)
{
	uint8_t sta = RESET;
	char init[6] = {0xEE,0x04,0xFF,0xFC,0xFF,0xFF};
	HmiUsart_Init(115200);								// 屏幕通讯
	vTaskDelay(20); 
	memset(hmi_Rxbff, 0, LEN * sizeof(char));				//先清空一下缓冲区
	HmiUsart_Transmit(HmiUsart,(char *)init,sizeof(init));
	vTaskDelay(300);
    if(HmiUsartRxLen)
	{
		HmiCopySerialData(HmiUsart,(char *)hmi_Rxbff,sizeof(hmi_Rxbff));	//进行屏幕握手
		if((hmi_Rxbff[0] == 0xEE) && (hmi_Rxbff[1] == 0x55) && (hmi_Rxbff[2] == 0xFF) && (hmi_Rxbff[3] == 0xFC)) //检查校验
		{
			sta = RESET;//通讯成功
			HmiUsart_Transmit(HmiUsart,(char *)sensitivity_5,sizeof(sensitivity_5));	//设置灵敏度5级
		}
		else
		{
			sta = SET;//通讯失败
		}	
		HmiClearSerialBuffer(HmiUsart);		
	}
	else{HmiClearSerialBuffer(HmiUsart); sta = SET;}//通讯失败
	
	return sta; 
}








/************************************************************************

** Function name:    void SetTextInt32(uint16 screen_id,
                                       uint16 control_id,
                                       uint32 value,);
** Descriptions    : 设置文本为正整数和零
** input parameters: screen_id:  画面ID
**                   control_id:  控件ID
**                   value:  显示的整数
** output parameters: 无
** Returned value   : 无
************************************************************************/
void display_power_num(uint8_t screen_id, uint8_t control_id,uint32_t value)
{
	unsigned char header[] = {0xEE,0xB1,0x10};
	
	unsigned char screenId[] = {0x00, 0x00};
	
	unsigned char controlId[] = {0x00 ,0x00};
	
	unsigned char end[] = {0xFF,0xFC,0xFF,0xFF};
	
	unsigned char *out_value;
	unsigned char *combined;	
	
	int digit_count = 0;
	
	if(screen_id){screenId[1] = screen_id;}
	
	if(control_id){controlId[1] = control_id;}
	
	
    if (value > UINT32_MAX) {
        return ; // 实际上uint16_t不会大于UINT16_MAX，这里主要是示意
    }

    // 处理0的特殊情况
	if (value == 0) {
		digit_count = 1;  // 至少分配 1 字节
		out_value = (uint8_t *)malloc(digit_count);
		if (out_value == NULL) return;
		out_value[0] = 0x30;  // 修正索引为 0
	}
	else 
	{

		// 计算位数（临时变量用于计算）
		uint32_t temp = value;

		while (temp != 0) {
			temp /= 10;
			digit_count++;
		}

		// 分配数组内存
		out_value = (unsigned char *)malloc(digit_count * sizeof(unsigned char));
		if (out_value == NULL) {
			return ;
		}

		// 提取每一位并转换为十六进制
		temp = value;
		for (int i = digit_count - 1; i >= 0; i--) {
			uint8_t digit = temp % 10; // 获取最后一位
			(out_value)[i] = digit + 0x30;   // 十进制转十六进制（0-9对应0-9）
			temp /= 10;
		}
	}
	
    // 计算总长度
    int total_length = sizeof(header) + sizeof(screenId) + sizeof(controlId) + sizeof(end) + digit_count;

		// 分配数组内存
	combined = (unsigned char *)malloc(digit_count * sizeof(unsigned char));
		if (out_value == NULL) {
			return ;
		}
	
    // 逐个拷贝数组
    int offset = 0;
    memcpy(combined + offset, header, sizeof(header));
    offset += sizeof(header);
	
    memcpy(combined + offset, screenId, sizeof(screenId));
    offset += sizeof(screenId);
	
    memcpy(combined + offset, controlId, sizeof(controlId));
    offset += sizeof(controlId);
	
    memcpy(combined + offset, out_value, digit_count);
    offset += digit_count;
	
    memcpy(combined + offset, end, sizeof(end));
    offset += sizeof(end);
	
	HmiUsart_Transmit(HmiUsart,(char *)combined,total_length);
	

    // 释放内存
    free(out_value);
    // 释放内存
    free(combined);
}



void hal_led_set(uint8_t num,uint8_t flag)
{
	unsigned char rotation_speed[] = {0xEE,0xB1,0x23,0x00,0x00,0x00,0x0F,0x00,0xFF,0xFC,0xFF,0xFF};
	rotation_speed[6] = num+15;
	rotation_speed[7] = flag;
	
	HmiUsart_Transmit(HmiUsart,(char *)rotation_speed,sizeof(rotation_speed));
	vTaskDelay(10);
	
}



/**
 * @brief 根据转速和配置参数控制LED显示
 * @param rpm       当前转速（0到max_rpm）
 * @param max_rpm   转速上限（如40000）
 * @param led_count LED总数（如10）
 * @note 点亮：左到右依次亮；熄灭：右到左依次熄灭；0=低电平点亮，1=高电平熄灭
**/
void Real_time_rotation_speed(int rpm, int max_rpm, int led_count)
{
	
	static  uint8_t init_led = 0;	//初始化速度LED条
	
	
	if(!init_led)
	{
		init_led = 1;
		//	控制LED状态
		for (int i = 0; i < led_count; i++) {
			 hal_led_set(i, (i < 10) ? 1 : 0);
		}
	}
	
    // 参数检查
    if (rpm < 0) rpm = 0;
    if (rpm > max_rpm) rpm = max_rpm;
    if (led_count <= 0) return; // 无效LED数量

    // 静态变量：保存每个LED的当前状态（0=点亮，1=熄灭），仅初始化一次
    static int led_status[100] = {0}; // 假设LED总数不超过100，可根据实际调整
    // 静态变量：保存上一次的点亮数量，用于判断转速上升/下降
    static int last_active_leds = 0;

    // 计算本次需要点亮的LED数量（线性映射，逻辑不变）
    int target_leds = (rpm * led_count + max_rpm - 1) / max_rpm;
    // 确保转速>0时至少点亮1个LED
    if (rpm > 0 && target_leds == 0) {
        target_leds = 1;
    }
    // 转速为0时强制全灭
    if (rpm == 0) {
        target_leds = 0;
    }

    // ========== 核心逻辑：区分“点亮（上升）”和“熄灭（下降）” ==========
    if (target_leds > last_active_leds) {
        // 场景1：转速上升 → 左到右逐个点亮（原有逻辑）
        for (int i = last_active_leds; i < target_leds; i++) {
            if (i < led_count) { // 防止越界
                led_status[i] = 0; // 点亮（低电平）
                hal_led_set(i, 0);
            }
        }
    } else if (target_leds < last_active_leds) {
        // 场景2：转速下降 → 右到左逐个熄灭（核心修改）
        for (int i = last_active_leds - 1; i >= target_leds; i--) {
            if (i < led_count) { // 防止越界
                led_status[i] = 1; // 熄灭（高电平）
                hal_led_set(i, 1);
            }
        }
    }
    // 场景3：转速不变 → 不做任何操作

    // 更新上一次的点亮数量
    last_active_leds = target_leds;
}




//手柄使用选择设置
void Handle_selection_settings(uint8_t  seq)
{
	if(seq == 1)
	{
		//电机选择和使能引脚
		hmivar.handelsta = 1; //发送停止指令进行中
		HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
		
		if(handle.Link2)
		{
			switch(hmivar.Dynsysta)
			{
				case 1: DynsySta.CycleSend = 4;  DynsySta.CycleStraSped = hmivar.DynsystraSpe;   break;
				case 2: DynsySta.CycleSend = 5;  DynsySta.CycleReveSped = hmivar.DynsyReveSpe;   break;
				case 3: DynsySta.CycleSend = 6;  DynsySta.CycleSRSped = hmivar.DynsySRSpe;     break;	
				default:	break;
			}
			vTaskDelay(200);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //和驱动板1通讯
		vTaskDelay(200);
		hmivar.handelsta = 0; //发送停止指令完成
		hmivar.handel = 1;	//选择手柄1
		
	}
	else if(seq == 2)
	{
		//电机选择和使能引脚
		hmivar.handelsta = 1; //发送停止指令进行中
		HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
		if(handle.Link1)
		{
			switch(hmivar.Dynsysta)
			{
				case 1: DynsySta.CycleSend = 4;  DynsySta.CycleStraSped = hmivar.DynsystraSpe;   break;
				case 2: DynsySta.CycleSend = 5;  DynsySta.CycleReveSped = hmivar.DynsyReveSpe;   break;
				case 3: DynsySta.CycleSend = 6;  DynsySta.CycleSRSped = hmivar.DynsySRSpe;     break;	
				default:	break;
			}
			vTaskDelay(200);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);	//和驱动板2通讯
		vTaskDelay(200);
		hmivar.handelsta = 0; //发送停止指令完成
		hmivar.handel = 2; //选择手柄2
		
	}
	

}





//接收上位机数据处理函数
void HmiReceiveDate(void)
{
	if(HmiUsartRxLen)
	{

		HmiCopySerialData(HmiUsart,(char *)hmi_Rxbff,sizeof(hmi_Rxbff));
		
		if((hmi_Rxbff[0] == 0x5A) && (hmi_Rxbff[1] == 0x01) && (hmi_Rxbff[3] == 0xA5))
		{
			switch(hmi_Rxbff[2])
			{ 
				case 0x00 :	//注水切换
							{
								footvar.pump_tims_global++;
								if(footvar.pump_tims_global > 7)	//当前挡位
								{
									footvar.pump_powerflag = 0;
									footvar.pump_tims_global = 1;

								}
								footvar.pumpGearChangeflag = footvar.pump_tims_global; //赋值当前挡位
								pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
								switch(footvar.pumpGearChangeflag)	//判断目前是那个挡位
								{
									case 1: 
											{
												HMI_PRINTF("1档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear1_off,sizeof(pumpgear1_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									case 2: 
											{

												HMI_PRINTF("2档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear2_off,sizeof(pumpgear2_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									case 3: 
											{
												HMI_PRINTF("3档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear3_off,sizeof(pumpgear3_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									case 4: 
											{
												HMI_PRINTF("4档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear4_off,sizeof(pumpgear4_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令

												break;
											}
									case 5: 
											{
												HMI_PRINTF("5档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear5_off,sizeof(pumpgear5_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									case 6: 
											{
												HMI_PRINTF("6档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear6_off,sizeof(pumpgear6_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									case 7: 
											{
												HMI_PRINTF("7档运行\r\n");
												HmiUsart_Transmit(HmiUsart,(char *)pumpgear7_off,sizeof(pumpgear7_off));
												pumpsta.stop = 1;
												vTaskDelay(500); 								 //给一个蠕动泵动作时间
												pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
												break;
											}
									default:	 break;
								}
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x01:	//挡位切换
							{
								hmivar.num_flag++;
								if(hmivar.num_flag > 8)
								{
									hmivar.num_flag = 0;
									switch(hmivar.Dynsysta)
									{
										case 1:	{
													DynsySta.CycleStraSped = hmivar.DynsystraSpe; footvar.foot_gear = hmivar.DynsystraSpe;
													if(DynsySta.CycleStraSped == GRINDING_8) display_power_num(0,5,DynsySta.CycleStraSped+5000); 
													else{display_power_num(0,5,DynsySta.CycleStraSped);}
													break;
												}
										case 2: 
												{
													DynsySta.CycleReveSped = hmivar.DynsyReveSpe; footvar.foot_gear = hmivar.DynsyReveSpe;  
													if(DynsySta.CycleReveSped == GRINDING_8)display_power_num(0,5,DynsySta.CycleReveSped+5000); 
													else{display_power_num(0,5,DynsySta.CycleReveSped);}
													break;
												}
										case 3: DynsySta.CycleSRSped = hmivar.DynsySRSpe;  footvar.foot_gear = hmivar.DynsySRSpe;    display_power_num(0,5,DynsySta.CycleSRSped); break;	
										default:	break;
									}
									
									HmiUsart_Transmit(HmiUsart,(char *)power_gear_0_off,sizeof(power_gear_0_off));
								}
								switch(hmivar.num_flag)
								{
									case 1: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe; footvar.foot_gear = hmivar.DynsystraSpe; 
															display_power_num(0,5,DynsySta.CycleStraSped); 
															break;
													case 2: hmivar.DynsyReveSpe = GRINDING_1; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; footvar.foot_gear = hmivar.DynsyReveSpe;  
															display_power_num(0,5,DynsySta.CycleReveSped); 
															break;
													case 3: hmivar.DynsySRSpe = PLANING_1;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;  footvar.foot_gear = hmivar.DynsySRSpe;    
															display_power_num(0,5,DynsySta.CycleSRSped); 
															break;	
													default:	break;
												}
											}
											break;
									case 2: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe; footvar.foot_gear = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_2; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_2;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;  footvar.foot_gear = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 3: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  footvar.foot_gear = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_3; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_3;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;   footvar.foot_gear = hmivar.DynsySRSpe;    display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 4: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe; footvar.foot_gear = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_4; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_4;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;    footvar.foot_gear = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 5: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  footvar.foot_gear = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_5; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_5;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;    footvar.foot_gear = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 6: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe; footvar.foot_gear = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_6; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_6;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;   footvar.foot_gear = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 7: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  footvar.foot_gear = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
													case 2: hmivar.DynsyReveSpe = GRINDING_7; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
													case 3: hmivar.DynsySRSpe = PLANING_7;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;    footvar.foot_gear = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped); break;	
													default:	break;
												}
											}
											break;
									case 8: 
											{
												switch(hmivar.Dynsysta)
												{
													case 1: hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  footvar.foot_gear = hmivar.DynsystraSpe; 
															display_power_num(0,5,DynsySta.CycleStraSped + 5000);
															break;
													case 2: hmivar.DynsyReveSpe = GRINDING_8; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  footvar.foot_gear = hmivar.DynsyReveSpe; 
															display_power_num(0,5,DynsySta.CycleReveSped + 5000); 
															break;
													case 3: hmivar.DynsySRSpe = PLANING_8;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;    footvar.foot_gear = hmivar.DynsySRSpe;  
															display_power_num(0,5,DynsySta.CycleSRSped); 
															break;	
													default:	break;
												}
											}
											break;
									default:	break;
								}

								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				
				case 0x02:	//运行
							{
								if(footvar.foot_flag)	//脚踏是否正在使用
								{
									hmivar.hmi_flag = 1;
									DynsySta.speed_num = 0;	//速度清零
									HmiUsart_Transmit(HmiUsart,(char *)power_on,sizeof(power_on));
									switch(hmivar.Dynsysta)
									{
										case 1: 
												{
													DynsySta.CycleSend = 1;  
													DynsySta.CycleStraSped = hmivar.DynsystraSpe;
													footvar.foot_gear = hmivar.DynsystraSpe;		//手动设置挡位
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)forw_rotdown,sizeof(forw_rotdown));
													
													break;
												}
										case 2: {
													DynsySta.CycleSend = 2;  
													DynsySta.CycleReveSped = hmivar.DynsyReveSpe;
													footvar.foot_gear = hmivar.DynsyReveSpe;		//手动设置挡位
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)reve_rotdown,sizeof(reve_rotdown));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
													vTaskDelay(10);		
											
													break;
												}
										case 3: {
													DynsySta.CycleSend = 3;  
													DynsySta.CycleSRSped = hmivar.DynsySRSpe;
													footvar.foot_gear = hmivar.DynsySRSpe;		//手动设置挡位
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotdown,sizeof(both_sides_rotdown));
													vTaskDelay(10);
													HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
													vTaskDelay(10);										
													break;
												}										
										default:	break;
									}
								}
								else{HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));vTaskDelay(5);}
								
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				
				case 0x03:	//停止
							{
								if(footvar.foot_flag)	//脚踏是否正在使用
								{
									DynsySta.speed_num = 0;
									HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
									switch(hmivar.Dynsysta)
									{
										case 1: DynsySta.CycleSend = 4;  DynsySta.CycleStraSped = hmivar.DynsystraSpe;   break;
										case 2: DynsySta.CycleSend = 5;  DynsySta.CycleReveSped = hmivar.DynsyReveSpe;   break;
										case 3: DynsySta.CycleSend = 6;  DynsySta.CycleSRSped = hmivar.DynsySRSpe;     break;	
										default:	break;
									}
									vTaskDelay(5);
								}
								else{HmiUsart_Transmit(HmiUsart,(char *)power_on,sizeof(power_on));vTaskDelay(5);}
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				
				case 0x04:	// 选择手柄2
							{
								
								Handle_selection_settings(2);
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				
				case 0x05:	// 选择手柄1
							{
								
								Handle_selection_settings(1);
								HmiClearSerialBuffer(HmiUsart);
								break;
								
							}
				case 0x06:	//正转模式
							{
								HMI_PRINTF("正转模式");
								DynsySta.speed_num = 0;
								HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
								vTaskDelay(5);
								
								
								hmivar.Dynsysta = 1;
								DynsySta.StaSend = 1;
								switch(hmivar.Dynsysta)
								{
									case 1: DynsySta.CycleSend = 4;  break;
									case 2: DynsySta.CycleSend = 5;  break;
									case 3: DynsySta.CycleSend = 6;  break;	
									default:	break;
								}
								switch(hmivar.num_flag)
								{
									case 1: 
											{
												hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 2: 
											{
												hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 3: 
											{
												hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 4: 
											{
												hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 5: 
											{
												hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 6: 
											{
												hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 7: 
											{
												hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
											}
									case 8: 
											{
												hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped + display_cps); break;
											}
									default:	DynsySta.CycleStraSped = hmivar.DynsystraSpe; 
												if(hmivar.DynsystraSpe == GRINDING_8) display_power_num(0,5,DynsySta.CycleStraSped + display_cps);
												else  display_power_num(0,5,DynsySta.CycleStraSped);
												break;
								}
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)forw_rotdown,sizeof(forw_rotdown));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));

								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x07:	//反转模式
							{
								HMI_PRINTF("反转模式");
								DynsySta.speed_num = 0;
								HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
								vTaskDelay(5);
								
								hmivar.Dynsysta = 2;
								DynsySta.StaSend = 2;
								switch(hmivar.Dynsysta)
								{
									case 1: DynsySta.CycleSend = 4;  break;
									case 2: DynsySta.CycleSend = 5;  break;
									case 3: DynsySta.CycleSend = 6;  break;	
									default:	break;
								}
								switch(hmivar.num_flag)
								{
									case 1: 
											{
												hmivar.DynsyReveSpe = GRINDING_1; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 2: 
											{
												hmivar.DynsyReveSpe = GRINDING_2; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 3: 
											{
												hmivar.DynsyReveSpe = GRINDING_3; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 4: 
											{
												hmivar.DynsyReveSpe = GRINDING_4; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 5: 
											{
												hmivar.DynsyReveSpe = GRINDING_5; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 6: 
											{
												hmivar.DynsyReveSpe = GRINDING_6; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 7: 
											{
												hmivar.DynsyReveSpe = GRINDING_7; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped); break;
											}
									case 8: 
											{
												hmivar.DynsyReveSpe = GRINDING_8; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  display_power_num(0,5,DynsySta.CycleReveSped + display_cps); break;
											}
									default:	DynsySta.CycleReveSped = hmivar.DynsyReveSpe;
												if(hmivar.DynsyReveSpe == GRINDING_8) display_power_num(0,5,DynsySta.CycleReveSped + display_cps); 
												else display_power_num(0,5,DynsySta.CycleReveSped); 
												break;
								}
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)reve_rotdown,sizeof(reve_rotdown));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
								vTaskDelay(5);
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x08:	//正反转模式
							{
								HMI_PRINTF("正反转模式");
								DynsySta.speed_num = 0;
								HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)Planing,sizeof(Planing));	//刨削模式
								vTaskDelay(5);
								
								hmivar.Dynsysta = 3;
								DynsySta.StaSend = 3;
								switch(hmivar.Dynsysta)
								{
									case 1: DynsySta.CycleSend = 4;  break;
									case 2: DynsySta.CycleSend = 5;  break;
									case 3: DynsySta.CycleSend = 6;  break;	
									default:	break;
								}
								switch(hmivar.num_flag)
								{
									case 1: 
											{
												hmivar.DynsySRSpe = PLANING_1; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 2: 
											{
												hmivar.DynsySRSpe = PLANING_2; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 3: 
											{
												hmivar.DynsySRSpe = PLANING_3; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 4: 
											{
												hmivar.DynsySRSpe = PLANING_4; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 5: 
											{
												hmivar.DynsySRSpe = PLANING_5; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 6: 
											{
												hmivar.DynsySRSpe = PLANING_6; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 7: 
											{
												hmivar.DynsySRSpe = PLANING_7; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									case 8: 
											{
												hmivar.DynsySRSpe = PLANING_8; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
											}
									default:	DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotdown,sizeof(both_sides_rotdown));
								vTaskDelay(5);
								HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
								vTaskDelay(5);
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x09:	
							{
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x0A:	//转速增加
							{
							
								BEEP = 1;
								vTaskDelay(15);
								BEEP = 0;
								hmivar.num_flag = 0;	//将挡位清零
								HmiUsart_Transmit(HmiUsart,(char *)power_gear_0_off,sizeof(power_gear_0_off));vTaskDelay(10);
								switch(hmivar.Dynsysta)
								{
									case 1: 
											{
												if(hmivar.DynsystraSpe < GRINDING_8){hmivar.DynsystraSpe += 1000;}
												else if(hmivar.DynsystraSpe >= GRINDING_8){hmivar.DynsystraSpe = GRINDING_8;}
												DynsySta.CycleStraSped = hmivar.DynsystraSpe;
												
												if(hmivar.DynsystraSpe >= GRINDING_8)display_power_num(0,5,hmivar.DynsystraSpe + display_cps);
												else{display_power_num(0,5,hmivar.DynsystraSpe);}

												
												uint8_t pbuf[2];
												pbuf[0] = (hmivar.DynsystraSpe >> 8) & 0xFF; // 高8位
												pbuf[1] = hmivar.DynsystraSpe & 0xFF;        // 低8位
												at24cxx_write(ADDR_STRASPE,pbuf,2);

												break;
											}
									case 2: 											
											{
												if(hmivar.DynsyReveSpe < GRINDING_8){hmivar.DynsyReveSpe += 1000;}
												else if(hmivar.DynsyReveSpe >= GRINDING_8){hmivar.DynsyReveSpe = GRINDING_8;}
												DynsySta.CycleReveSped = hmivar.DynsyReveSpe;
												
												if(hmivar.DynsyReveSpe >= GRINDING_8)display_power_num(0,5,hmivar.DynsyReveSpe + display_cps);
												else{display_power_num(0,5,hmivar.DynsyReveSpe);}
												uint8_t pbuf[2];
												pbuf[0] = (hmivar.DynsyReveSpe >> 8) & 0xFF; // 高8位
												pbuf[1] = hmivar.DynsyReveSpe & 0xFF;        // 低8位
												at24cxx_write(ADDR_REVESPE,pbuf,2);
												
												break;
											}
									case 3: 											
											{
												if(hmivar.DynsySRSpe < PLANING_8){hmivar.DynsySRSpe += 100;}
												else if(hmivar.DynsySRSpe >= PLANING_8){hmivar.DynsySRSpe = PLANING_8;}
												DynsySta.CycleSRSped = hmivar.DynsySRSpe;
												
												display_power_num(0,5,DynsySta.CycleSRSped);
												uint8_t pbuf[2];
												pbuf[0] = (hmivar.DynsySRSpe >> 8) & 0xFF; // 高8位
												pbuf[1] = hmivar.DynsySRSpe & 0xFF;        // 低8位
												at24cxx_write(ADDR_STRESPE,pbuf,2);
												
												break;
											}	
									default:	break;
								}
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x0B:	//转速减少
							{
								BEEP = 1;
								vTaskDelay(15);
								BEEP = 0;
								hmivar.num_flag = 0; //将挡位清零
								HmiUsart_Transmit(HmiUsart,(char *)power_gear_0_off,sizeof(power_gear_0_off));vTaskDelay(10);
								switch(hmivar.Dynsysta)
								{
										case 1: 
												{
													if(hmivar.DynsystraSpe > 0){hmivar.DynsystraSpe -= 1000;}
													if(hmivar.DynsystraSpe < 3000)
													{hmivar.DynsystraSpe = 2000;}
													DynsySta.CycleStraSped = hmivar.DynsystraSpe;
													display_power_num(0,5,DynsySta.CycleStraSped);
													uint8_t pbuf[2];
													pbuf[0] = (hmivar.DynsystraSpe >> 8) & 0xFF; // 高8位
													pbuf[1] = hmivar.DynsystraSpe & 0xFF;        // 低8位
													at24cxx_write(ADDR_STRASPE,pbuf,2);	
													break;
												}
										case 2: 											
												{
													if(hmivar.DynsyReveSpe > 0){hmivar.DynsyReveSpe -= 1000;}
													if(hmivar.DynsyReveSpe < 3000)
													{hmivar.DynsyReveSpe = 2000;}
													DynsySta.CycleReveSped = hmivar.DynsyReveSpe;
													display_power_num(0,5,DynsySta.CycleReveSped);
													uint8_t pbuf[2];
													pbuf[0] = (hmivar.DynsyReveSpe >> 8) & 0xFF; // 高8位
													pbuf[1] = hmivar.DynsyReveSpe & 0xFF;        // 低8位
													at24cxx_write(ADDR_REVESPE,pbuf,2);	
													break;
												}
										case 3: 											
												{
													if(hmivar.DynsySRSpe > 0){hmivar.DynsySRSpe -= 100;}
													if(hmivar.DynsySRSpe < 500)
													{hmivar.DynsySRSpe = 500;}
													DynsySta.CycleSRSped = hmivar.DynsySRSpe;
													display_power_num(0,5,DynsySta.CycleSRSped);
													uint8_t pbuf[2];
													pbuf[0] = (hmivar.DynsySRSpe >> 8) & 0xFF; // 高8位
													pbuf[1] = hmivar.DynsySRSpe & 0xFF;        // 低8位
													at24cxx_write(ADDR_STRESPE,pbuf,2);	
													break;
												}	
										default:	break;
										
									}
									HmiClearSerialBuffer(HmiUsart);
									break;
							}
				case 0x0C:	
							{
								
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x0D:
							{

								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x0F:
							{

								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x10:
							{
								
								HmiClearSerialBuffer(HmiUsart);
								break;
							}
				case 0x11:
							{

								HmiClearSerialBuffer(HmiUsart);
								break;
							}
							
				default:	break;
			}				

			
		}
		
		memset(hmi_Rxbff, 0, LEN * sizeof(uint8_t));		//先清空一下缓冲区
		HmiClearSerialBuffer(HmiUsart);
		}
	
}



/**
 * @brief 更新速度图标
 * @param num   转速上限（如40000）根据设置来
**/
void HmiSpeedUpdate(uint32_t num)		
{
	
	Real_time_rotation_speed(DynsySta.speed_num,num,10);
	
	
	
} 

//检测脚踏更新脚踏图标
void HmiFootCheckUpdate(void)			
{
	static uint8_t fcsta = 0;
	
	if(footvar.footCheckFlag && (!fcsta))
	{
		fcsta = !fcsta;
		//更新插入图标
		HMI_PRINTF("更新脚踏插入图标");
		HmiUsart_Transmit(HmiUsart,(char *)foot_on,sizeof(foot_on));
		vTaskDelay(20);
	}
	else if((!footvar.footCheckFlag) && (fcsta))
	{
		fcsta = !fcsta;
		//更新未插入图标
		HMI_PRINTF("更新脚踏未插入图标");
		HmiUsart_Transmit(HmiUsart,(char *)foot_off,sizeof(foot_off));
		vTaskDelay(20);
	}

	
} 

//检测蠕动泵更新图标
void HmiPumpCheckUpdate(void)			
{
	static uint8_t psta = 0;
	
	if(pumpsta.pumpflag && (!psta))
	{
		psta = !psta;
		//更新连接图标
		HMI_PRINTF("更新蠕动泵插入图标");
		HmiUsart_Transmit(HmiUsart,(char *)pump_on,sizeof(pump_on));
		vTaskDelay(20);
	}
	else if((!pumpsta.pumpflag) && (psta))
	{
		psta = !psta;
		//更新未连接图标
		HMI_PRINTF("更新蠕动泵未插入图标");
		HmiUsart_Transmit(HmiUsart,(char *)pump_off,sizeof(pump_off));
		vTaskDelay(20);
	}
	
} 



// ===================== SendGearCmd 放在 HmiGearUpdate 上方 =====================
static void SendGearCmd(void)
{
    switch(hmivar.num_flag)
    {
        case 1:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_1,sizeof(power_gear_1)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_1; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_1;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 2:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_2,sizeof(power_gear_2)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_2; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_2;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 3:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_3,sizeof(power_gear_3)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_3; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_3;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 4:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_4,sizeof(power_gear_4)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_4; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_4;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 5:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_5,sizeof(power_gear_5)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_5; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_5;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 6:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_6,sizeof(power_gear_6)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_6; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_6;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 7:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_7,sizeof(power_gear_7)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_7; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped); break;
                case 3: hmivar.DynsySRSpe   = PLANING_7;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        case 8:
            HmiUsart_Transmit(HmiUsart,(char *)power_gear_8,sizeof(power_gear_8)); vTaskDelay(10);
            switch(hmivar.Dynsysta)
            {
                case 1: hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe; display_power_num(0,5,DynsySta.CycleStraSped + display_cps); break;
                case 2: hmivar.DynsyReveSpe = GRINDING_8; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; display_power_num(0,5,DynsySta.CycleReveSped + display_cps); break;
                case 3: hmivar.DynsySRSpe   = PLANING_8;  DynsySta.CycleSRSped   = hmivar.DynsySRSpe;   display_power_num(0,5,DynsySta.CycleSRSped);   break;
                default: break;
            }
            break;
        default:
            break;
    }
}


void HmiGearUpdate(void)
{
    static uint8_t  slide_phase    = 0;
    static uint8_t  delay_whether  = 0;
    static uint8_t  speed_flag     = 0;
    static uint8_t  last_foot_flag = 0;

    if(footvar.footGearChangeflag != last_foot_flag)
    {
        last_foot_flag = footvar.footGearChangeflag;
    }

    uint8_t need_change = 0;
    if(footvar.footGearChangeflag)
    {
        speed_flag++;
        if(speed_flag > 2)
        {
            speed_flag  = 0;
            need_change = 1;
            BEEP_ON;
        }
    }
    else
    {
        speed_flag = 0;
    }

    if(need_change)
    {
        need_change = 0;
        footvar.footGearChangeflag = 0;

        if(g_slide_flag == SLIDE_RIGHT)
        {
            slide_phase = 1;
        }
		else if(g_slide_flag == SLIDE_LEFT)
		{
			if(!delay_whether)
			{
				// 直接发，不管 timer_busy，发完立即复位
				SendGearCmd();
				vTaskDelay(5);
				HmiUsart_Transmit(HmiUsart,(char *)interface_right1,sizeof(interface_right1));
				vTaskDelay(5);
				HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_right1,sizeof(Sinterslider_right1));
				vTaskDelay(5);
				// 不启动定时器，不占 timer_busy，直接结束
				slide_phase   = 0;
				delay_whether = 0;
			}
		}
    }

    switch(slide_phase)
    {
        case 1:
            if(g_timer_3s == 0 && timer_busy == 0)
            {
                // 先占用
                timer_busy    = 1;
                p_timer_3s    = 0;
                Tim8_Start();
                delay_whether = 1;
                g_slide_flag  = SLIDE_LEFT;

                // 再发指令
                SendGearCmd();
                vTaskDelay(5);
                HmiUsart_Transmit(HmiUsart,(char *)interface_right1,sizeof(interface_right1));
                vTaskDelay(5);
                HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_right1,sizeof(Sinterslider_right1));
                vTaskDelay(5);

                slide_phase = 2;
            }
            break;

        case 2:
            if(p_timer_3s >= 25)
            {
                Tim8_Stop();
                p_timer_3s    = 0;
                timer_busy    = 0;   // ? 释放
                if(delay_whether == 1)
                {
                    SendGearCmd();   // 补发
                    vTaskDelay(5);
                    delay_whether = 0;
                }
                slide_phase = 0;     // ? 复位，下次才能正常触发
            }
            break;

        default:
            break;
    }
}






//根据脚踏更新模式图标
void HmiPatternUpdate(void)		
{
	if(footvar.foot_patternflag)
	{
		footvar.foot_patternflag = 0;
		switch(hmivar.Dynsysta)
		{
			case 1: DynsySta.CycleSend = 4;  DynsySta.StaSend = 1; 
					DynsySta.speed_num = 0;
					HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
					vTaskDelay(5);			
					HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)forw_rotdown,sizeof(forw_rotdown));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)interface_left1,sizeof(interface_left1));
					vTaskDelay(5); 
					HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_left1,sizeof(Sinterslider_left1));
					vTaskDelay(5); 
					BEEP_ON;
			
					switch(hmivar.num_flag)
					{
						case 1: 
								{
									hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 2: 
								{
									hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 3: 
								{
									hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 4: 
								{
									hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 5: 
								{
									hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 6: 
								{
									hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 7: 
								{
									hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 8: 
								{
									hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped + display_cps); break;
								}
						default:	DynsySta.CycleStraSped = hmivar.DynsystraSpe; 
									if(hmivar.DynsystraSpe == GRINDING_8) display_power_num(0,5,DynsySta.CycleStraSped  + display_cps); 
									else display_power_num(0,5,DynsySta.CycleStraSped); 
									break;
					}
					break;
			case 2: DynsySta.CycleSend = 5;  DynsySta.StaSend = 2;
					DynsySta.speed_num = 0;
					HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
					vTaskDelay(5);	
					HmiUsart_Transmit(HmiUsart,(char *)reve_rotdown,sizeof(reve_rotdown));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)interface_left2,sizeof(interface_left2));
					vTaskDelay(5); 
					HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_left2,sizeof(Sinterslider_left2));
					vTaskDelay(5); 	
					BEEP_ON;
					switch(hmivar.num_flag)
					{
						case 1: 
								{
									hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 2: 
								{
									hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 3: 
								{
									hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 4: 
								{
									hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 5: 
								{
									hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 6: 
								{
									hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 7: 
								{
									hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped); break;
								}
						case 8: 
								{
									hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  display_power_num(0,5,DynsySta.CycleStraSped + display_cps); break;
								}
						default:	DynsySta.CycleStraSped = hmivar.DynsystraSpe; 
									if(hmivar.DynsystraSpe == GRINDING_8)display_power_num(0,5,DynsySta.CycleStraSped + display_cps); 
									else display_power_num(0,5,DynsySta.CycleStraSped); 
									break;
					}
					break;
			case 3: DynsySta.CycleSend = 6;  DynsySta.StaSend = 3;
					DynsySta.speed_num = 0;
					HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)Planing,sizeof(Planing));	//刨削模式
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotdown,sizeof(both_sides_rotdown));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)interface_left2,sizeof(interface_left2));
					vTaskDelay(5);
					HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_left2,sizeof(Sinterslider_left2));
					vTaskDelay(5); 	
					BEEP_ON;					
					switch(hmivar.num_flag)
					{
						case 1: 
								{
									hmivar.DynsySRSpe = PLANING_1; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 2: 
								{
									hmivar.DynsySRSpe = PLANING_2; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 3: 
								{
									hmivar.DynsySRSpe = PLANING_3; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 4: 
								{
									hmivar.DynsySRSpe = PLANING_4; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 5: 
								{
									hmivar.DynsySRSpe = PLANING_5; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 6: 
								{
									hmivar.DynsySRSpe = PLANING_6; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 7: 
								{
									hmivar.DynsySRSpe = PLANING_7; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						case 8: 
								{
									hmivar.DynsySRSpe = PLANING_8; DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
								}
						default:	DynsySta.CycleSRSped = hmivar.DynsySRSpe;  display_power_num(0,5,DynsySta.CycleSRSped); break;
					}
					break;	
			default:	break;
		}
	}
}





/************************ 你原有的发送图标函数（完全保留） ************************/
static void SendPumpDriveIcon(void)
{
    if (footvar.pump_powerflag == 1)   // 电源开启 → 发送“运行”图标
    {
		HMI_PRINTF("footvar.pump_powerflag = %d",footvar.pump_powerflag);
		HMI_PRINTF("footvar.pumpGearChangeflag = %d",footvar.pumpGearChangeflag);
        switch (footvar.pumpGearChangeflag)
        {
			
            case 1: HmiUsart_Transmit(HmiUsart, (char *)pumpgear1, sizeof(pumpgear1)); break;
            case 2: HmiUsart_Transmit(HmiUsart, (char *)pumpgear2, sizeof(pumpgear2)); break;
            case 3: HmiUsart_Transmit(HmiUsart, (char *)pumpgear3, sizeof(pumpgear3)); break;
            case 4: HmiUsart_Transmit(HmiUsart, (char *)pumpgear4, sizeof(pumpgear4)); break;
            case 5: HmiUsart_Transmit(HmiUsart, (char *)pumpgear5, sizeof(pumpgear5)); break;
            case 6: HmiUsart_Transmit(HmiUsart, (char *)pumpgear6, sizeof(pumpgear6)); break;
            case 7: HmiUsart_Transmit(HmiUsart, (char *)pumpgear7, sizeof(pumpgear7)); break;
            default: break;
        }
    }
    else if(footvar.pump_powerflag == 2)                          // 电源关闭 → 发送“停止”图标
    {
        switch (footvar.pumpGearChangeflag)
        {
            case 1: HmiUsart_Transmit(HmiUsart, (char *)pumpgear1_off, sizeof(pumpgear1_off)); break;
            case 2: HmiUsart_Transmit(HmiUsart, (char *)pumpgear2_off, sizeof(pumpgear2_off)); break;
            case 3: HmiUsart_Transmit(HmiUsart, (char *)pumpgear3_off, sizeof(pumpgear3_off)); break;
            case 4: HmiUsart_Transmit(HmiUsart, (char *)pumpgear4_off, sizeof(pumpgear4_off)); break;
            case 5: HmiUsart_Transmit(HmiUsart, (char *)pumpgear5_off, sizeof(pumpgear5_off)); break;
            case 6: HmiUsart_Transmit(HmiUsart, (char *)pumpgear6_off, sizeof(pumpgear6_off)); break;
            case 7: HmiUsart_Transmit(HmiUsart, (char *)pumpgear7_off, sizeof(pumpgear7_off)); break;
            default: break;
        }
    }
}

/************************根据脚踏更新蠕动泵图标 核心处理函数 ************************/
void HmiPumpSlideProcess(void)
{
	// ===================== 内部静态变量（你定义的+补发计数） =====================
	static uint8_t slide_phase = 0;          // 流程阶段：0=空闲，1=等待第一次3秒，2=等待第二次3秒
	static uint8_t delay_whether = 0;          // 没有滑动子界面的时候第二次进来不需要延时情况
	
	// ===================== 【新增】记录上一次状态，用于判断变化 =====================
	static uint8_t last_pump_power = 0xFF;
	static uint8_t last_pump_gear = 0xFF;

    // ===================== 抢占式触发判断（原逻辑保留） =====================
    uint8_t new_trigger = 0;
    if( ((footvar.pump_tims_global_power) && (footvar.pumpautomaticflag == 1)) || 
        ((!footvar.pump_tims_global_power) && (footvar.pumpautomaticflag == 1)) || 
        ((footvar.pump_tims_global != 0) && (footvar.pumpautomaticflag == 1)) )
		{
			new_trigger = 1;
		}

	// ===================== 【核心】判断：开关 或 挡位变化 → 蜂鸣器响 =====================
	if( footvar.pump_powerflag != last_pump_power || footvar.pumpGearChangeflag != last_pump_gear )
	{
		// 只要变了，就响一声
		BEEP_ON;
		
		// 更新记录值
		last_pump_power = footvar.pump_powerflag;
		last_pump_gear = footvar.pumpGearChangeflag;
	}

    // 新触发 → 复位状态，启动流程
    if(new_trigger)
    {
		new_trigger = 0;
        
		// ===================== 滑动标志检测（原框架保留） =====================
		// 左滑触发
		if (g_slide_flag == SLIDE_LEFT)
		{   
			slide_phase = 1;   		
		}
		// 右滑触发
		else if (g_slide_flag == SLIDE_RIGHT)
		{  			
			slide_phase = 2;     			
			if(!delay_whether)
			{
				SendPumpDriveIcon();
				vTaskDelay(5);
				HMI_PRINTF("delay_whether = %d",delay_whether);
				footvar.pumpautomaticflag = 0;
			}
		}
    }

    // ===================== 阶段机处理（无补发，流程走完即结束） =====================
    switch (slide_phase)
    {
        case 1: // 等待第一次2秒计时完成
            if (p_timer_3s == 0)       	//开关那边的延时是否完成 
            {
				//【仅发送1次】蠕动泵图标 + 右滑界面指令
				SendPumpDriveIcon();
				vTaskDelay(5);
                HmiUsart_Transmit(HmiUsart,(char *)interface_right2,sizeof(interface_right2));
                vTaskDelay(5); 
                HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_right2,sizeof(Sinterslider_right2));
                vTaskDelay(5); 
				g_timer_3s = 0;  // 重启计时，打开定时器
				Tim5_Start();
                slide_phase = 2;  
				delay_whether = 1;	//才滑动子界面需要时间才能去接收下一次	
				g_slide_flag = SLIDE_RIGHT;  // 触发右滑		
            }
            break;

        case 2: // 等待第二次3秒计时完成
            if (g_timer_3s >= 25) //延时3s
            {
				Tim5_Stop();
				g_timer_3s = 0;
				footvar.pumpautomaticflag = 0;  // 清除脚踏触发标志
                // 流程全部结束，直接复位到空闲
				//蠕动泵图标 + 右滑界面指令
				if(delay_whether==1)
				{
					SendPumpDriveIcon();
					vTaskDelay(5); 
					delay_whether = 0;
					HMI_PRINTF("发送 SendPumpDriveIcon");
				}
                slide_phase = 0;
				
            }
            break;

        default:
            break;
    }
}




void HmiPowerUpdate(void)
{
    static uint8_t  slide_phase   = 0;
    static uint8_t  delay_whether = 0;
    static uint8_t  Powersta_flag = 0;
    static uint8_t  speed_flag    = 0;
    static uint8_t  last_foot_flag = 0;

    if(footvar.foot_flag != last_foot_flag)
    {
        last_foot_flag = footvar.foot_flag;
    }

    uint8_t need_change = 0;
    if((!footvar.foot_flag) && (!Powersta_flag))
    {
        speed_flag++;
        if(speed_flag > 2)
        {
            speed_flag    = 0;
            need_change   = 1;
            Powersta_flag = 1;
        }
    }
    else if((footvar.foot_flag) && (Powersta_flag))
    {
        speed_flag    = 0;
        need_change   = 1;
        Powersta_flag = 0;
    }
    else
    {
        speed_flag = 0;
    }

    if(need_change)
    {
        need_change = 0;
        if(g_slide_flag == SLIDE_RIGHT)
        {
            // 右滑 → 等蠕动泵完成后再发
            slide_phase = 1;
        }
		else if(g_slide_flag == SLIDE_LEFT)
		{
			if(!delay_whether)
			{
				if(Powersta_flag)
				{
					HMI_PRINTF("打开！！！\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)power_on,sizeof(power_on));
				}
				else
				{
					HMI_PRINTF("关闭！！！\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
				}
				vTaskDelay(5);
				HmiUsart_Transmit(HmiUsart,(char *)interface_right1,sizeof(interface_right1));
				vTaskDelay(5);
				HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_right1,sizeof(Sinterslider_right1));
				vTaskDelay(5);
				slide_phase   = 0;  // ? 复位
				delay_whether = 0;  // ? 清零，下次能正常进来
			}
		}
    }

    switch(slide_phase)
    {
        case 1:
            // ? 条件1：蠕动泵延时完成
            // ? 条件2：定时器空闲，抢到才能继续，抢不到停在case1等下次
            if(g_timer_3s == 0 && timer_busy == 0)
            {
                // 先占用
                timer_busy    = 1;
                p_timer_3s    = 0;
                Tim8_Start();
                delay_whether = 1;
                g_slide_flag  = SLIDE_LEFT;

                // 再发指令
                if(Powersta_flag)
                {
                    HMI_PRINTF("打开！！！\r\n");
                    HmiUsart_Transmit(HmiUsart,(char *)power_on,sizeof(power_on));
                }
                else
                {
                    HMI_PRINTF("关闭！！！\r\n");
                    HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
                }
                vTaskDelay(5);
                HmiUsart_Transmit(HmiUsart,(char *)interface_right1,sizeof(interface_right1));
                vTaskDelay(5);
                HmiUsart_Transmit(HmiUsart,(char *)Sinterslider_right1,sizeof(Sinterslider_right1));
                vTaskDelay(5);

                slide_phase = 2;
            }
            break;

        case 2:
            if(p_timer_3s >= 25)
            {
                Tim8_Stop();
                p_timer_3s = 0;
                timer_busy = 0;  // ? 释放
                if(delay_whether == 1)
                {
                    if(Powersta_flag)
                    {
                        HmiUsart_Transmit(HmiUsart,(char *)power_on,sizeof(power_on));
                        HMI_PRINTF("打开运行\r\n");
                    }
                    else
                    {
                        HmiUsart_Transmit(HmiUsart,(char *)power_off,sizeof(power_off));
                        HMI_PRINTF("关闭运行\r\n");
                    }
                    vTaskDelay(5);
                    delay_whether = 0;
                }
                slide_phase = 0;
            }
            break;

        default:
            break;
    }
}








void HmiHandleUpdate(void)		//更新开关图标 ,自动上传
{	
	static	uint8_t  Handlesta_1 = 0;	//状态发生更改自动上传状态机
	static	uint8_t  Handlesta_2 = 0;	//状态发生更改自动上传状态机
	
	if(Handlesta_1 != handle.Link1 || Handlesta_2 != handle.Link2)
	{
		HMI_PRINTF("手柄检测");
		Handlesta_1 = handle.Link1;
		Handlesta_2 = handle.Link2;
		if(handle.Link1 && handle.Link2)
		{
			HmiUsart_Transmit(HmiUsart,(char *)handejoin3,sizeof(handejoin3)); 
			vTaskDelay(10);  
			switch(handle.Link1_Numb)	
			{
				case 1: HmiUsart_Transmit(HmiUsart,(char *)hande1_button,sizeof(hande1_button)); //选择手柄1
						vTaskDelay(10);  	   
						break;
				case 2: HmiUsart_Transmit(HmiUsart,(char *)hande2_button,sizeof(hande2_button)); //选择手柄2
						vTaskDelay(10);  	   
						break;
				case 3: break;
				default:break;
			}
			
			
			HMI_PRINTF("全部亮");   /* 延时1ticks */
			Handle_selection_settings(1);
		}
		else if((!handle.Link1) && handle.Link2)
		{
			HMI_PRINTF("2亮");
			switch(handle.Link2_Numb)	//判断手柄序号
			{
				case 1: HmiUsart_Transmit(HmiUsart,(char *)hande1_button,sizeof(hande1_button)); //选择手柄1
						vTaskDelay(5);  
						HmiUsart_Transmit(HmiUsart,(char *)handejoin1,sizeof(handejoin1)); //选择手柄接口1
						vTaskDelay(5);                                               /* 延时1ticks */				
						break;
				case 2: HmiUsart_Transmit(HmiUsart,(char *)hande2_button,sizeof(hande2_button)); //选择手柄2
						vTaskDelay(5);  
						HmiUsart_Transmit(HmiUsart,(char *)handejoin2,sizeof(handejoin2)); //选择手柄接口2
						vTaskDelay(5);                                               /* 延时1ticks */				
						break;
				case 3: break;
				default:break;
			}
			Handle_selection_settings(2);
		}
		else if(handle.Link1 && (!handle.Link2))
		{
			HMI_PRINTF("1亮");

			switch(handle.Link1_Numb) //判断手柄序号
			{
				case 1: HmiUsart_Transmit(HmiUsart,(char *)hande1_button,sizeof(hande1_button)); //选择手柄1
						vTaskDelay(5);  
						HmiUsart_Transmit(HmiUsart,(char *)handejoin1,sizeof(handejoin1)); //选择手柄接口1
						vTaskDelay(5);                                               /* 延时1ticks */				
						break;
				case 2: HmiUsart_Transmit(HmiUsart,(char *)hande2_button,sizeof(hande2_button)); //选择手柄2
						vTaskDelay(5);  
						HmiUsart_Transmit(HmiUsart,(char *)handejoin2,sizeof(handejoin2)); //选择手柄接口2
						vTaskDelay(5);                                               /* 延时1ticks */				
						break;
				case 3: break;
				default:break;
			}
			Handle_selection_settings(1);
		}
		else
		{
			HMI_PRINTF("全灭");
			HmiUsart_Transmit(HmiUsart,(char *)handejoinoff,sizeof(handejoinoff)); 
			vTaskDelay(5);                                               /* 延时1ticks */
		}
	}
} 


uint16_t Errornumber = 0;
// 全局/局部变量，每个延时单独一个
static TickType_t delay_start = 0;

//更新错误图标 ,自动上传
void HmiErrorUpdate(void)		
{	
	vTaskDelay(5);
	static uint8_t Errorsta = 0;	//为了只上传一次设置的状态机
	//蠕动泵相关故障
	if((pumpsta.gear == 10 || pumpsta.run == 4 || pumpsta.stop == 4))		//应答错误
	{
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			
			Errornumber = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_pump2,sizeof(fault_pump2));vTaskDelay(5);
		}
	}
	else if((pumpsta.gear == 9 || pumpsta.run == 3 || pumpsta.stop == 3))		//无应答
	{
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			if(pumpsta.gear == 9){pumpsta.gear = 0;}
			if(pumpsta.run == 3){pumpsta.run = 0;}
			if(pumpsta.stop == 3){pumpsta.stop = 0;}
			Errornumber = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_pump1,sizeof(fault_pump1));vTaskDelay(5);
		}
		
	}
	//电机相关故障
	else if(DynsySta.StaRece == 2 || DynsySta.StaRece == 9 || DynsySta.StaRece == 10 || DynsySta.StaRece == 11)	//过流
	{
		
		if(!Errorsta)
		{
			hmivar.op_delayflag = 1;
			delay_start = xTaskGetTickCount(); // 触发计时起点
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor7,sizeof(fault_motor7));vTaskDelay(5);
		}

	}
	//电机相关故障
	else if(DynsySta.StaRece == 12)	//电源电压异常
	{
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor1,sizeof(fault_motor1));vTaskDelay(5);
		}

	}
	else if(DynsySta.StaRece == 3)	//电源电压异常+电机电压异常
	{
		
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor2,sizeof(fault_motor2));vTaskDelay(5);
		}
	}
	else if(DynsySta.StaRece == 4)	//电机不转
	{

		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor3,sizeof(fault_motor3));vTaskDelay(5);
		}
	}
	else if(DynsySta.StaRece == 6)	//应答错误
	{	
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor4,sizeof(fault_motor4));vTaskDelay(5);
		}
	}
	else if(DynsySta.StaRece == 7)	//无应答
	{
		
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			// 判断是否够2秒
			if(!hmivar.op_delayflag){HmiUsart_Transmit(HmiUsart,(char *)fault_motor5,sizeof(fault_motor5));vTaskDelay(5);}
		}
		
		
	}
	else if(DynsySta.StaRece == 8)	//电机电压异常
	{
		
		if(!Errorsta)
		{
			Errorsta = 1;
			Tim3_Start();
			HMI_PRINTF("Tim3_Start()");
			Errornumber = 0;
			DynsySta.StaRece = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_motor6,sizeof(fault_motor6));vTaskDelay(5);
		}
		
	}
	if(Errorsta && (DynsySta.StaRece == 0))	//消除电机故障图标
	{
		if(Errornumber > 400)
		{
			Errorsta = 0;
			Tim3_Stop();
			HMI_PRINTF("Tim3_Stop()");
			Errornumber = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_clear,sizeof(fault_clear));vTaskDelay(5);
		}
	}
	if(Errorsta && (pumpsta.gear == 0 || pumpsta.run == 0 || pumpsta.stop == 0)) //消除蠕动泵故障图标
	{
		if(Errornumber > 400)
		{
			Errorsta = 0;
			Tim3_Stop();
			HMI_PRINTF("Tim3_Stop()");
			Errornumber = 0;
			HmiUsart_Transmit(HmiUsart,(char *)fault_clear,sizeof(fault_clear));vTaskDelay(5);
		}
	}
	if(Errornumber > 400)	//防止图标一直消除不了的意外情况
	{
		Errorsta = 0;
		Tim3_Stop();
		HMI_PRINTF("Tim3_Stop()");
		Errornumber = 0;
		HmiUsart_Transmit(HmiUsart,(char *)fault_clear,sizeof(fault_clear));vTaskDelay(5);
	}
	if(DynsySta.StaRece == 1)
	{
			if(hmivar.op_delayflag)
			{
				// 判断是否够2秒
				if((TickType_t)(xTaskGetTickCount() - delay_start) >= pdMS_TO_TICKS(2000))
				{
					hmivar.op_delayflag = 0;
				}
			
			}
	}
} 




//上电初始化设置默认参数
void  DynsyParam_settings(void)
{
	HmiSpeedUpdate(footvar.foot_gear);
	
	vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)fault_clear,sizeof(fault_clear));vTaskDelay(10);
	
	
	switch(hmivar.num_flag)	//初始化挡位和对应转速
	{
		case 1: 
				{

					hmivar.DynsystraSpe = GRINDING_1; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_1,sizeof(power_gear_1));

					hmivar.DynsyReveSpe = GRINDING_1; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_1,sizeof(power_gear_1));

					hmivar.DynsySRSpe = PLANING_1;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_1,sizeof(power_gear_1));vTaskDelay(10);
						
				}
				break;
		case 2: 
				{
					hmivar.DynsystraSpe = GRINDING_2; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_2,sizeof(power_gear_2));

					hmivar.DynsyReveSpe = GRINDING_2; DynsySta.CycleReveSped = hmivar.DynsyReveSpe; 
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_2,sizeof(power_gear_2));

					hmivar.DynsySRSpe = PLANING_2;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;    
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_2,sizeof(power_gear_2));vTaskDelay(10);

				}
				break;
		case 3: 
				{

					hmivar.DynsystraSpe = GRINDING_3; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_3,sizeof(power_gear_3));
					
					hmivar.DynsyReveSpe = GRINDING_3; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_3,sizeof(power_gear_3));

					hmivar.DynsySRSpe = PLANING_3;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_3,sizeof(power_gear_3));vTaskDelay(10);

				}
				break;
		case 4: 
				{

					hmivar.DynsystraSpe = GRINDING_4; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_4,sizeof(power_gear_4));
					
					hmivar.DynsyReveSpe = GRINDING_4; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_4,sizeof(power_gear_4));

					hmivar.DynsySRSpe = PLANING_4;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_4,sizeof(power_gear_4));vTaskDelay(10);

				}
				break;
		case 5: 
				{

					hmivar.DynsystraSpe = GRINDING_5; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_5,sizeof(power_gear_5));
					
					hmivar.DynsyReveSpe = GRINDING_5; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_5,sizeof(power_gear_5));

					hmivar.DynsySRSpe = PLANING_5;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_5,sizeof(power_gear_5));vTaskDelay(10);

				}
				break;
		case 6: 
				{

					hmivar.DynsystraSpe = GRINDING_6; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_6,sizeof(power_gear_6));
					
					hmivar.DynsyReveSpe = GRINDING_6; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_6,sizeof(power_gear_6));

					hmivar.DynsySRSpe = PLANING_6;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_6,sizeof(power_gear_6));vTaskDelay(10);

				}
				break;
		case 7: 
				{

					hmivar.DynsystraSpe = GRINDING_7; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_7,sizeof(power_gear_7));
					
					hmivar.DynsyReveSpe = GRINDING_7; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_7,sizeof(power_gear_7));

					hmivar.DynsySRSpe = PLANING_7;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_7,sizeof(power_gear_7));vTaskDelay(10);

				}
				break;
		case 8: 
				{

					hmivar.DynsystraSpe = GRINDING_8; DynsySta.CycleStraSped = hmivar.DynsystraSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_8,sizeof(power_gear_8));
					
					hmivar.DynsyReveSpe = GRINDING_8; DynsySta.CycleReveSped = hmivar.DynsyReveSpe;  
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_8,sizeof(power_gear_8));

					hmivar.DynsySRSpe = PLANING_8;	DynsySta.CycleSRSped = hmivar.DynsySRSpe;     
					vTaskDelay(10); HmiUsart_Transmit(HmiUsart,(char *)power_gear_8,sizeof(power_gear_8));vTaskDelay(10);

				}
				break;
		default:	break;
	}
	
	//初始化旋转模式，和当前模式下对应转速显示-*/
	switch(hmivar.Dynsysta)
	{
		case 1:   
				HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)forw_rotdown,sizeof(forw_rotdown));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
				vTaskDelay(10);
				display_power_num(0,5,DynsySta.CycleStraSped);
				vTaskDelay(10);
				break;
		case 2:    
				HmiUsart_Transmit(HmiUsart,(char *)reve_rotdown,sizeof(reve_rotdown));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotup,sizeof(both_sides_rotup));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)forw_rotup,sizeof(forw_rotup));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)Grinding,sizeof(Grinding));		//磨砖模式
				vTaskDelay(10);
				display_power_num(0,5,DynsySta.CycleReveSped);
				break;
		case 3:      
				HmiUsart_Transmit(HmiUsart,(char *)reve_rotup,sizeof(reve_rotup));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)both_sides_rotdown,sizeof(both_sides_rotdown));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)forw_rotdown,sizeof(forw_rotdown));
				vTaskDelay(10);
				HmiUsart_Transmit(HmiUsart,(char *)Planing,sizeof(Planing));		//刨削模式
				vTaskDelay(10);
				display_power_num(0,5,DynsySta.CycleSRSped); 
				break;	
		default:	break;
	}
	
	//蠕动泵初始化参数
	footvar.pumpGearChangeflag = footvar.pump_tims_global; //赋值当前挡位
	switch(footvar.pumpGearChangeflag)	//判断目前是那个挡位
	{
		case 1: 
				{
					HMI_PRINTF("1档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear1_off,sizeof(pumpgear1_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		case 2: 
				{

					HMI_PRINTF("2档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear2_off,sizeof(pumpgear2_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		case 3: 
				{
					HMI_PRINTF("3档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear3_off,sizeof(pumpgear3_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		case 4: 
				{
					HMI_PRINTF("4档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear4_off,sizeof(pumpgear4_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令

					break;
				}
		case 5: 
				{
					HMI_PRINTF("5档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear5_off,sizeof(pumpgear5_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		case 6: 
				{
					HMI_PRINTF("6档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear6_off,sizeof(pumpgear6_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		case 7: 
				{
					HMI_PRINTF("7档运行\r\n");
					HmiUsart_Transmit(HmiUsart,(char *)pumpgear7_off,sizeof(pumpgear7_off));
					pumpsta.stop = 1;
					vTaskDelay(500); 								 //给一个蠕动泵动作时间
					pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
					break;
				}
		default:	 break;
	}
}




void hmi_task_init(void)
{
	taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )HMI_TASK,
                (const char*    )"HMI_TASK",
                (uint16_t       )HMI_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )HMI_PRIO,
                (TaskHandle_t*  )&HMITask_Handler);
    taskEXIT_CRITICAL();            /* 退出临界区 */
}









/**
 * @brief       HMI_TASK
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void HMI_TASK(void *pvParameters)
{
	uint8_t val_flag = 0;		//脚踏是否使用检测速度	
	uint8_t tim_flag = 0;		//给一个延迟时间
	static uint8_t num = 3;
	
	vTaskDelay(500);                                             
    while(hmi_init() && (--num))
	{
		HMI_PRINTF("连接中！！！");
	}
	if(!num)
	{
		HMI_PRINTF("通讯失败！！！");
	}
	num = 3;
	HmiUsart_Transmit(HmiUsart,(char *)init_prog_30,sizeof(init_prog_30));
	vTaskDelay(30);   //等待串口屏处理完成
	
	do
	{
		HmiUsart_Transmit(HmiUsart,(char *)init_prog_60,sizeof(init_prog_60));
		vTaskDelay(30);   //等待串口屏处理完成
	}
	
	while(!DynsySta.Commu && (--num));

	num = 3;
	do
	{
		HmiUsart_Transmit(HmiUsart,(char *)init_prog_70,sizeof(init_prog_70));
		vTaskDelay(30);   //等待串口屏处理完成
	}
	while(!pumpsta.pumpflag && (--num));

	num = 3;
	do
	{
		HmiUsart_Transmit(HmiUsart,(char *)init_prog_100,sizeof(init_prog_100));
		vTaskDelay(30);   //等待串口屏处理完成
	}
	while(at24cxx_check() && (--num));
		
	
	Tim3_Init(10000-1,84-1);  	//定时1ms
	Tim5_Init(10000-1,840-1);	//用于子界面滑动，定时10ms 用于子界面右滑动
	Tim8_Init(10000-1,1680-1);	//用于子界面滑动，定时10ms 用于子界面左滑动
	DynsyParam_settings();
	vTaskDelay(1000);                                               /* 延时1ticks 进行任务调度 */
    while(1)
    {
		
		HmiReceiveDate();		//接收屏幕数据处理
		HmiPowerUpdate();		//脚踏电机开关图标
		HmiGearUpdate();		//脚踏更新挡位图标
		HmiPatternUpdate();		//脚踏更新模式切换
		HmiFootCheckUpdate();	//检测脚踏更新脚踏图标
		HmiPumpCheckUpdate();	//检测蠕动泵更新图标
		HmiPumpSlideProcess();	//根据脚踏更新蠕动泵图标
		HmiHandleUpdate();		//手柄状态实时检测
		HmiErrorUpdate();		//故障代码显示
		/******************* 脚踏实时更改速度图标 **********************/
		if((DynsySta.CycleSend < 4) && (DynsySta.CycleSend > 0))
		{
			tim_flag = 0;	//计数清零
			val_flag = 1; 	//延时状态置位
			HmiSpeedUpdate(footvar.foot_gear);
		}
		if(val_flag == 1)
		{
			if(tim_flag > 10)	//给10ms一个延迟时间
			{
				DynsySta.speed_num = 0; //必须速度清零
				HMI_PRINTF("关闭速度图标");
				tim_flag = 0;	//计数清零
				val_flag = 0;  //延时状态清零
				HmiSpeedUpdate(footvar.foot_gear);
			}
			tim_flag++;
		}
		/************************************************************/
        vTaskDelay(1);                                               /* 延时1ticks 进行任务调度 */
    }
}

















