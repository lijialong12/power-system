#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/SRAM/sram.h"
#include "./MALLOC/malloc.h"
#include "./BSP/USART/USART.h"
#include <string.h>  // 必须包含strlen/strstr头文件
/*FreeRTOS*********************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "includes.h"
/******************************************************************************************************/

// 调试打印（按需开启）
#define PC_PRINTF(format, ...)     //Debug_Printf("【PC_TASK】:"format "\r\n",##__VA_ARGS__)

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


const char handle_link1[] = {0x48,0x44,0x31,0x0D,0x0A};		//手柄1连接，未选择使用

const char handle_select1[] = {0x55,0x53,0x31,0x0D,0x0A};	//手柄1连接，并选择使用

const char pumpuser1[] 	= {0x50,0x55,0x31,0x0D,0x0A};		//手柄1蠕动泵正在使用

uint8_t handle1_Rxbff[LEN] = "";



/************************* 可配置参数宏 *************************/
#define HANDLE1_SEND_INTERVAL    100     // 常规发送间隔：100ms
#define UART_REINIT_INTERVAL     300000  // 串口重初始化间隔：5分钟=5*60*1000ms

/************************* 状态机枚举定义 *************************/
typedef enum {
    HANDLE1_IDLE = 0,           // 空闲状态
    HANDLE1_SEND_PUMP,          // 发送蠕动泵指令
    HANDLE1_SEND_SELECT,        // 发送手柄选择指令
    HANDLE1_SEND_LINK,          // 发送常规连接指令
    HANDLE1_WAIT_RESP,          // 等待设备响应
    HANDLE1_CHECK_RESP,         // 校验响应数据
    HANDLE1_UART_REINIT,        // 串口安全重初始化状态
} Handle1_StateTypeDef;


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

/**
 * @brief  构造并发送单次JSON数据（封装复用）
 * @retval 无
 */
void send_single_json(void)
{
    char *pc_buff = NULL;
    uint8_t sramx = 0;
    
    // 申请内存并初始化
    pc_buff = mymalloc(sramx, 128);
    if(pc_buff == NULL) {
        PC_PRINTF("内存申请失败");
        return;
    }
    memset(pc_buff, 0, 128);
    
    // 构造JSON（使用DynsySta.speed_num）
    sprintf(pc_buff, "{\"zhuansu\":%d}", DynsySta.speed_num);
    
    // 发送数据
    send_json_string(pc_buff);
    PC_PRINTF("发送数据：%s", pc_buff);
    
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
    } else {
        return; // 数据过长，直接返回
    }

    // 2. 校验是否为目标get指令，触发持续发送
    if(verify_get_cmd(json_buf)) {
        g_continuous_send_flag = 1;  // 置位持续发送标记
        PC_PRINTF("触发持续发送模式");
    } else {
        // 可扩展：接收其他指令（如停止发送）
        if(SAFE_STRSTR(json_buf, "\"cmd\":\"stop\"") != NULL) {
            g_continuous_send_flag = 0;  // 清零停止发送
            PC_PRINTF("停止持续发送");
        }
    }
}




/*************************手柄接口1 核心业务函数 *************************/
void handle1_link_status(void)
{
    static Handle1_StateTypeDef state = HANDLE1_IDLE;
    static uint32_t last_send_tick = 0;    // 上次发送的时间戳
    static uint32_t last_reinit_tick = 0;  // 上次串口初始化的时间戳
    static uint8_t  retry_cnt = 0;          // 通讯失败重试计数
    static uint8_t  uart_reinit_req = 0;    // 串口重初始化请求标志
    static Handle1_StateTypeDef last_send_state = HANDLE1_SEND_LINK; // 记录上次发送的指令类型

    uint32_t current_tick = xTaskGetTickCount();

    // ===================== 1. 5分钟定时触发串口重初始化请求 =====================
    if ((current_tick - last_reinit_tick >= UART_REINIT_INTERVAL) && (uart_reinit_req == 0))
    {
        uart_reinit_req = 1;  // 标记需要初始化，等待当前通讯周期结束后执行
        last_reinit_tick = current_tick;
    }

    // ===================== 2. 200ms定时发送触发（非阻塞） =====================
    if ((state == HANDLE1_IDLE) && (current_tick - last_send_tick >= HANDLE1_SEND_INTERVAL))
    {
        // 有初始化请求，优先进入初始化流程
        if (uart_reinit_req == 1)
        {
            state = HANDLE1_UART_REINIT;
        }
        // 无初始化请求，按业务逻辑选择发送的指令
        else if (footvar.pump_tims_global_power && hmivar.handel == 1)
        {
            state = HANDLE1_SEND_PUMP;
            last_send_state = HANDLE1_SEND_PUMP;
        }
        else if (hmivar.handel == 1)
        {
            state = HANDLE1_SEND_SELECT;
            last_send_state = HANDLE1_SEND_SELECT;
        }
        else
        {
            state = HANDLE1_SEND_LINK;
            last_send_state = HANDLE1_SEND_LINK;
        }
        last_send_tick = current_tick; // 更新发送时间戳
    }

    // ===================== 3. 状态机核心流程 =====================
    switch (state)
    {
        // --------------------- 串口安全重初始化流程 ---------------------
        case HANDLE1_UART_REINIT:
        {
            // 步骤1：停止DMA发送，避免初始化冲突
            HAL_UART_DMAStop(&huart3);
            
            // 步骤2：清空所有收发缓冲区，重置接收长度
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            PCClearSerialBuffer(PCUsart);
            PCUsartRxLen = 0;

            // 步骤3：安全执行串口初始化（使用你定义的宏）
            PCUsart_Init(115200);

            // 步骤4：重置标志位和重试计数
            uart_reinit_req = 0;
            retry_cnt = 0;

            // 步骤5：初始化完成，回到空闲状态，下一个200ms周期自动发送
            state = HANDLE1_IDLE;
            break;
        }

        // --------------------- 发送蠕动泵指令 ---------------------
        case HANDLE1_SEND_PUMP:
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            PCUsart_Transmit(PCUsart, (char *)pumpuser1, sizeof(pumpuser1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 发送手柄选择指令 ---------------------
        case HANDLE1_SEND_SELECT:
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            PCUsart_Transmit(PCUsart, (char *)handle_select1, sizeof(handle_select1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 发送常规连接指令 ---------------------
        case HANDLE1_SEND_LINK:
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            PCUsart_Transmit(PCUsart, (char *)handle_link1, sizeof(handle_link1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 等待设备响应 ---------------------
        case HANDLE1_WAIT_RESP:
            // 有数据接收，拷贝数据进入校验
            if (PCUsartRxLen > 0)
            {
                PCCopySerialData(PCUsart, (char *)handle1_Rxbff, sizeof(handle1_Rxbff));
                PCClearSerialBuffer(PCUsart);
                PCUsartRxLen = 0;
                state = HANDLE1_CHECK_RESP;
            }
            // 无数据，超时进入校验（判定本次通讯失败）
            else if (current_tick - last_send_tick >= HANDLE1_SEND_INTERVAL / 2)
            {
                state = HANDLE1_CHECK_RESP;
            }
            break;

        // --------------------- 校验响应数据 ---------------------
        case HANDLE1_CHECK_RESP:
        {
            uint8_t comm_ok = 0;

            // 根据发送的指令类型，匹配对应的响应校验规则
            if (last_send_state == HANDLE1_SEND_PUMP)
            {
                if ((handle1_Rxbff[0] == 0x50) && (handle1_Rxbff[1] == 0x55))
                    comm_ok = 1;
            }
            else if (last_send_state == HANDLE1_SEND_SELECT)
            {
                if ((handle1_Rxbff[0] == 0x55) && (handle1_Rxbff[1] == 0x53))
                    comm_ok = 1;
            }
            else // HANDLE1_SEND_LINK
            {
                if ((handle1_Rxbff[0] == 0x48) && (handle1_Rxbff[1] == 0x44))
                    comm_ok = 1;
            }
            // 通讯成功处理
            if (comm_ok)
            {
                retry_cnt = 0;
                handle.Link1 = 1; // 标记通讯正常
				switch(handle1_Rxbff[2])
				{
					case 0x31: handle.Link1_Numb = 1; break;
					case 0x32: handle.Link1_Numb = 2; break;
					case 0x33: handle.Link1_Numb = 3; break;
					default:break;
				}
				
            }
            // 通讯失败处理
            else
            {
                retry_cnt++;
                // 连续3次失败，标记离线
                if (retry_cnt > 2)
                {
                    retry_cnt = 0;
                    handle.Link1 = 0;
					handle.Link1_Numb = 0;
                }
            }

            // 校验完成，回到空闲状态，等待下一个200ms周期
            state = HANDLE1_IDLE;
            break;
        }

        // 异常状态兜底，回到空闲
        default:
            state = HANDLE1_IDLE;
            break;
    }
}

/**
 * @brief  PC通讯主任务（FreeRTOS任务）
 * @param  pvParameters: 任务参数
 * @retval 无
 */
void PC_TASK(void *pvParameters)
{
    // 任务启动时，仅初始化一次串口（使用你定义的宏）
    PCUsart_Init(115200);
    PC_PRINTF("PC通讯+手柄1通讯任务启动");

    while(1)
    {
        handle1_link_status();    // 状态机核心循环
        vTaskDelay(1);   // 1ms系统调度，兼顾实时性和CPU占用
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


