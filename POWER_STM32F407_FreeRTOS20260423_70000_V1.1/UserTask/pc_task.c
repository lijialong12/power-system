#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/SRAM/sram.h"
#include "./MALLOC/malloc.h"
#include "./BSP/USART/USART.h"
#include <string.h>  // 必须包含strlen/strstr头文件
#include <stdio.h>   // 必须包含sprintf头文件
/*FreeRTOS*********************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "includes.h"
/******************************************************************************************************/

// 调试打印（按需开启）
#define PC_PRINTF(format, ...)    // Debug_Printf("【PC_TASK】:"format "\r\n",##__VA_ARGS__)

/* PC_TASK 任务配置 */
#define PC_PRIO      	CONFIG_PC_PRIO                    /* 任务优先级 */
#define PC_STK_SIZE  	CONFIG_PC_STK_SIZE                 /* 任务堆栈大小 */
TaskHandle_t            PCTask_Handler;                   /* 任务句柄 */
void PC_TASK(void *pvParameters);                        /* 任务函数 */

// 安全匹配JSON字段的宏
#define SAFE_STRSTR(src, sub)  ((src == NULL || sub == NULL) ? NULL : strstr(src, sub))

// 全局业务参数
uint8_t g_continuous_send_flag = 0;   // 持续发送标记：0-停止 1-持续发送
uint32_t g_send_interval = 50;       // 持续发送间隔（ms），可按需调整


/**
 * @brief  串口初始化
 * @retval 0-成功
 */
uint8_t pc_init(void)
{
    PCUsart_Init(115200);        // 初始化串口波特率115200
    vTaskDelay(20);              // 等待初始化稳定
    return 0;
}

/**
 * @brief  校验上位机JSON指令是否为目标get指令
 * @param  json_str: 接收到的JSON字符串
 * @retval 1-是目标指令 0-不是
 */
uint8_t verify_get_cmd(char *json_str)
{
    // 第一步：校验基础合法性
    if(json_str == NULL || strlen(json_str) < 10) return 0;
    // 安全匹配核心字段
    if(SAFE_STRSTR(json_str, "\"cmd\":\"get\"") != NULL &&
       SAFE_STRSTR(json_str, "\"zhuansu\"") != NULL)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  安全发送JSON字符串（无帧头帧尾）
 * @param  json_str: 要发送的JSON字符串
 * @retval 无
 */
void send_json_string(char *json_str) {
    if(json_str == NULL || strlen(json_str) == 0) {
        PC_PRINTF("JSON字符串为空");
        return;
    }
    
    // 直接发送JSON字符串
    Usart_Transmit(PCUsart, json_str, strlen(json_str));
    vTaskDelay(5);
}


// 修复后的转速线性映射：原始 0~75000 → 显示 0~80000
// 支持负数输入，自动钳位到0~80000范围
// 使用64位整数运算，彻底解决溢出问题
uint32_t speed_map_75000_to_80000(int32_t raw_speed)
{
    // 第一步：边界检查，负数直接返回0
    if(raw_speed < 0) {
        return 0;
    }
    
    // 第二步：上限检查，超过75000直接返回80000
    if(raw_speed > 75000) {
        return 80000;
    }
    
    // 核心修复：先强制转换为64位整数再相乘，彻底避免溢出
    // 64位整数最大值是9e18，完全足够容纳75000*80000=6e9
    return (uint32_t)(((uint64_t)raw_speed * 80000ULL) / 75000ULL);
}


/**
 * @brief  构造并发送单次JSON数据（封装复用）
 * @retval 无
 */
void send_single_json(void)
{
    char *pc_buff = NULL;
    uint8_t sramx = 0;
    uint32_t send_speed;
    
    // 申请内存并初始化
    pc_buff = mymalloc(sramx, 128);
    if(pc_buff == NULL) {
        PC_PRINTF("内存申请失败");
        return;
    }
    memset(pc_buff, 0, 128);
    
    // 先计算要发送的转速值
    if(hmivar.num_flag == 8)
    {
        send_speed = speed_map_75000_to_80000(DynsySta.speed_num);
    }
    else
    {
        // 修复：即使不映射，也做边界检查，避免负数
        send_speed = (DynsySta.speed_num < 0) ? 0 : (uint32_t)DynsySta.speed_num;
    }
    
    // 核心修复：使用%lu格式符打印uint32_t类型
    // 绝对不能再用%d！%d只适用于有符号int
    sprintf(pc_buff, "{\"zhuansu\":%lu}", send_speed);
    
    PC_PRINTF("num_flag = %d, 原始转速 = %d, 发送转速 = %lu", 
              hmivar.num_flag, DynsySta.speed_num, send_speed);
    
    // 发送数据
    send_json_string(pc_buff);
    
    // 释放内存（避免泄漏）
    myfree(sramx, pc_buff);
    pc_buff = NULL;
}

/**
 * @brief  解析上位机JSON指令（无帧头帧尾校验）
 * @param  buf: 接收缓冲区
 * @param  len: 数据长度
 * @retval 无
 */
void parse_uart_data(uint8_t *buf, uint8_t len) {
    // 1. 将接收数据转为以\0结尾的字符串
    char json_buf[256] = {0};
    if(len > 0 && len < sizeof(json_buf)) {
        memcpy(json_buf, buf, len);
        json_buf[len] = '\0';  // 确保字符串以\0结尾（重要！）
    } else {
        PC_PRINTF("接收数据长度异常：%d", len);
        return; // 数据过长，直接返回
    }

    // 2. 校验是否为目标get指令，触发持续发送
    if(verify_get_cmd(json_buf)) {
        g_continuous_send_flag = 1;  // 置位持续发送标记
        PC_PRINTF("触发持续发送模式，间隔：%dms", g_send_interval);
    } else {
        // 可扩展：接收其他指令（如停止发送）
        if(SAFE_STRSTR(json_buf, "\"cmd\":\"stop\"") != NULL) {
            g_continuous_send_flag = 0;  // 清零停止发送
            PC_PRINTF("停止持续发送");
        }
    }
}

/**
 * @brief  PC通讯任务主函数（核心：非阻塞循环发送+实时接收）
 * @param  pvParameters: 任务参数
 * @retval 无
 */
void PC_TASK(void *pvParameters)
{
    uint8_t *recv_buff = NULL;
    uint8_t sramx = 0;
    uint8_t times = 0;
    uint32_t send_ticks = 0;  // 持续发送计时
    
    pc_init();  // 初始化串口
    PC_PRINTF("PC通讯任务启动");

    while(1)
    {
        // 1. 每5ms处理一次接收数据（实时响应上位机指令）
        if(times >= 1)
        {
            times = 0;
            // 申请接收缓冲区
            recv_buff = mymalloc(sramx, 255);
            if(recv_buff != NULL && PCUsartRxLen > 0)
            {
                // 拷贝并解析串口数据
                PCCopySerialData(PCUsart, (char *)recv_buff, 255);
                parse_uart_data(recv_buff, PCUsartRxLen);
                // 清空串口缓冲区
                PCClearSerialBuffer(PCUsart);
                PCUsartRxLen = 0;  // 强制清零，避免残留
            }
            // 释放接收缓冲区
            myfree(sramx, recv_buff);
            recv_buff = NULL;
        }
        
        // 2. 持续发送逻辑（非阻塞，不影响指令接收）
        if(g_continuous_send_flag == 1)
        {
            send_ticks++;
            // 达到发送间隔则发送一次
            if(send_ticks >= g_send_interval)
            {
                send_single_json();  // 构造并发送数据
                send_ticks = 0;      // 重置计时
            }
        }
        
        vTaskDelay(1);  // 1ms延时，兼顾响应速度与CPU占用
        times++;
    }
}

/**
 * @brief  PC通讯任务初始化
 * @retval 无
 */
void pc_task_init(void)
{
    taskENTER_CRITICAL();           // 进入临界区
    xTaskCreate((TaskFunction_t)PC_TASK,
                (const char*)"PC_TASK",
                (uint16_t)PC_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)PC_PRIO,
                (TaskHandle_t*)&PCTask_Handler);
    taskEXIT_CRITICAL();            // 退出临界区
}


