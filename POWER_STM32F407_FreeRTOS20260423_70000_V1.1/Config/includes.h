/*
************************************************************************************************
主要的包含文件

************************************************************************************************
*/

#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>


#include "nvicprio_conf.h"
/******************SYSTEM*************************************/
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"

/**********************操作系统头文件*******************************/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "task_conf.h"
#include "./MALLOC/malloc.h"


/**********************内设驱动头文件*******************************/
#include "./BSP/LED/led.h"
#include "led_task.h"
#include "dynsys_task.h"
#include "hmi_task.h"
#include "./BSP/IIC/24cxx.h"
#include "./BSP/USART/USART.h"
#include "pump_task.h"
#include "./BSP/KEY/key.h"
#include "./BSP/ADC/adc.h"
#include "foot_task.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/TIM_PWM_DMA/TIM1_CH3_DMA2_6_PWM.h"
#include "handle_1_task.h"
#include "handle_2_task.h"
#include "pc_task.h"
#endif

