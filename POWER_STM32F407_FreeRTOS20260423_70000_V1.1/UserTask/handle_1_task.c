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
#define HANDLE_1_PRINTF(format, ...)    // Debug_Printf("【HANDLE_1_TASK】:"format "\r\n",##__VA_ARGS__)

/* PC_TASK 任务配置 */
#define HANDLE_1_PRIO      	CONFIG_HANDLE_1_PRIO                    /* 任务优先级 */
#define HANDLE_1_STK_SIZE  	CONFIG_HANDLE_1_STK_SIZE                 /* 任务堆栈大小 */
TaskHandle_t            HANDLE_1Task_Handler;                   /* 任务句柄 */
void HANDLE_1_TASK(void *pvParameters);                        /* 任务函数 */

// 安全匹配JSON字段的宏
#define SAFE_STRSTR(src, sub)  ((src == NULL || sub == NULL) ? NULL : strstr(src, sub))


const char handle_link1[] = {0x67,0x65,0x74,0x68,0x0D,0x0A};		//手柄1连接，未选择使用	geth

const char handle_select1[] = {0x67,0x65,0x74,0x75,0x0D,0x0A};	//手柄1连接，并选择使用 getu

const char pumpuser1[] 	= {0x67,0x65,0x74,0x70,0x0D,0x0A};		//手柄1蠕动泵正在使用	getp

uint8_t handle1_Rxbff[LEN] = "";



/************************* 可配置参数宏 *************************/
#define HANDLE1_SEND_INTERVAL    20     // 常规发送间隔：100ms
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




AthPair pairsval = {0,0};


// 解析函数整体替换
static int ath_parse_frame(const char *ascii_frame, AthPair *out_pair)
{
    // 1. 参数合法性检查
    if (!ascii_frame || !out_pair) {
        return -1;
    }

    // 2. 跳过帧头 "AT?" 4个字符，指向数据部分
    const char *payload = ascii_frame + 4;

    // 3. 去除帧尾 \r \n
    size_t len = strlen(payload);
    while (len > 0 && (payload[len - 1] == '\n' || payload[len - 1] == '\r')) {
        ((char *)payload)[--len] = '\0';
    }

    // 4. 解析 "数字1,数字2"
    int a, b;
    if (sscanf(payload, "%d,%d", &a, &b) != 2) {
        return -3;
    }

    // 5. 直接写入静态结构体，不再 malloc
    out_pair->gyrosvalue1 = a;
    out_pair->pressvalue2 = b;

    return 0;
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

    // ===================== 2. 100ms定时发送触发（非阻塞） =====================
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
            HANDLE_1ClearSerialBuffer(HANDLE_1Usart);
            HANDLE_1UsartRxLen = 0;

            // 步骤3：安全执行串口初始化（使用你定义的宏）
            HANDLE_1Usart_Init(115200);

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
            HANDLE_1Usart_Transmit(HANDLE_1Usart, (char *)pumpuser1, sizeof(pumpuser1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 发送手柄选择指令 ---------------------
        case HANDLE1_SEND_SELECT:
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            HANDLE_1Usart_Transmit(HANDLE_1Usart, (char *)handle_select1, sizeof(handle_select1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 发送常规连接指令 ---------------------
        case HANDLE1_SEND_LINK:
            memset(handle1_Rxbff, 0, LEN * sizeof(char));
            HANDLE_1Usart_Transmit(HANDLE_1Usart, (char *)handle_link1, sizeof(handle_link1));
            state = HANDLE1_WAIT_RESP;
            break;

        // --------------------- 等待设备响应 ---------------------
        case HANDLE1_WAIT_RESP:
            // 有数据接收，拷贝数据进入校验
            if (HANDLE_1UsartRxLen > 0)
            {
                HANDLE_1CopySerialData(HANDLE_1Usart, (char *)handle1_Rxbff, sizeof(handle1_Rxbff));
                HANDLE_1ClearSerialBuffer(HANDLE_1Usart);
                HANDLE_1UsartRxLen = 0;
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
			if ((handle1_Rxbff[0] == 'A') && (handle1_Rxbff[1] == 'T'))
			{
				// 根据发送的指令类型，匹配对应的响应校验规则
				if (last_send_state == HANDLE1_SEND_PUMP)
				{
					if (handle1_Rxbff[2] == 'P')
						comm_ok = 1;
					int ret = ath_parse_frame((char *)handle1_Rxbff, &pairsval);

					if (ret == 0) {
							//printf(" %d, %d\n", pairsval.gyrosvalue1, pairsval.pressvalue2);
					} else {
						//printf("解析失败，错误码: %d\n", ret);
					}
				}
				else if (last_send_state == HANDLE1_SEND_SELECT)
				{
					if (handle1_Rxbff[2] == 'U')
						comm_ok = 1;
					int ret = ath_parse_frame((char *)handle1_Rxbff, &pairsval);
					if (ret == 0) {
							//printf(" %d, %d\n", pairsval.gyrosvalue1, pairsval.pressvalue2);
					} else {
						//printf("解析失败，错误码: %d\n", ret);
					}
				}
				else // HANDLE1_SEND_LINK
				{
					if (handle1_Rxbff[2] == 'H')
						comm_ok = 1;	
					int ret = ath_parse_frame((char *)handle1_Rxbff, &pairsval);
					if (ret == 0) {
					
					//printf(" %d, %d\n", pairsval.gyrosvalue1, pairsval.pressvalue2);
					} else {
						//printf("解析失败，错误码: %d\n", ret);
					}
				}
			}
            // 通讯成功流水灯处理
            if (comm_ok)
            {
                retry_cnt = 0;
                handle.Link1 = 1; // 标记通讯正常
				//HANDLE_1_PRINTF("解析成功1: %d\n", handle1_Rxbff[3]);
				switch(handle1_Rxbff[3])
				{
					case 0x31: handle.Link1_Numb = 1; break;
					case 0x32: handle.Link1_Numb = 2; break;
					case 0x33: handle.Link1_Numb = 3; break;
					default:	handle.Link1_Numb = 0;  break;
				}
				
            }
            // 通讯失败处理
            else
            {
                retry_cnt++;
                // 连续50次失败，标记离线
				
                if (retry_cnt > 50)
                {
					HANDLE_1_PRINTF("解析失败失败失败1: %d\n", handle1_Rxbff[3]);
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
 * @brief  手柄接口1通讯主任务（FreeRTOS任务）
 * @param  pvParameters: 任务参数
 * @retval 无
 */
void HANDLE_1_TASK(void *pvParameters)
{
    // 任务启动时，仅初始化一次串口（使用你定义的宏）
    HANDLE_1Usart_Init(115200);
    HANDLE_1_PRINTF("手柄1通讯任务启动");

    while(1)
    {
        handle1_link_status();    // 状态机核心循环
        vTaskDelay(1);   // 1ms系统调度，兼顾实时性和CPU占用
    }
}







/**
 * @brief  手柄接口1通讯任务初始化
 * @retval 无
 */
void handle_1_task_init(void)
{
    taskENTER_CRITICAL();           // 进入临界区
    xTaskCreate((TaskFunction_t)HANDLE_1_TASK,
                (const char*)"HANDLE_1_TASK",
                (uint16_t)HANDLE_1_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)HANDLE_1_PRIO,
                (TaskHandle_t*)&HANDLE_1Task_Handler);
    taskEXIT_CRITICAL();            // 退出临界区
}


