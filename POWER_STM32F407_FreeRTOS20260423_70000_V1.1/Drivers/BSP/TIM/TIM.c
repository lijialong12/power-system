#include "./BSP/TIM/TIM.h"
#include "stdio.h"
#include "includes.h"


TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim2;


/* TIM2 init function */
void Tim2_Init(uint16_t arr, uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    //Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    //Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    //Error_Handler();
  }
	//HAL_TIM_Base_Start_IT(&htim2); /* 开启定时器2定时中断 */
}

/* TIM3 init function */
void Tim3_Init(uint16_t arr, uint16_t psc)
{
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = psc;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = arr;
  HAL_TIM_Base_Init(&htim3);
  //HAL_TIM_Base_Start_IT(&htim3);                       /* 使能定时器x和定时器更新中断 */
}

/* TIM4 init function */
void Tim4_Init(uint16_t arr, uint16_t psc)
{
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = psc;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = arr;
  HAL_TIM_Base_Init(&htim4);
  //HAL_TIM_Base_Start_IT(&htim4);                       /* 使能定时器x和定时器更新中断 */
}
/* TIM5 init function */
void Tim5_Init(uint16_t arr, uint16_t psc)
{
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = psc;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = arr;
  HAL_TIM_Base_Init(&htim5);
  //HAL_TIM_Base_Start_IT(&htim5);                       /* 使能定时器x和定时器更新中断 */
}

/* TIM6 init function */
void Tim6_Init(uint16_t arr, uint16_t psc)
{
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = psc;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = arr;
  HAL_TIM_Base_Init(&htim6);
  //HAL_TIM_Base_Start_IT(&htim6);                       /* 使能定时器x和定时器更新中断 */

}

/* TIM7 init function */
void Tim7_Init(uint16_t arr, uint16_t psc)
{
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = psc;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = arr;
  HAL_TIM_Base_Init(&htim7);
  //HAL_TIM_Base_Start_IT(&htim7);                       /* 使能定时器x和定时器更新中断 */
}

/* TIM1 init function */
void Tim1_Init(uint16_t arr, uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = psc;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = arr;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    //Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    //Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    //Error_Handler();
  }
}


/* TIM8 init function */
void Tim8_Init(uint16_t arr, uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = psc;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = arr;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    //Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    //Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    //Error_Handler();
  }
	//HAL_TIM_Base_Start_IT(&htim8); /* 开启定时器8定时中断 */
}


void Tim3_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim3);                       /* 使能定时器x和定时器更新中断 */
}

void Tim3_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim3);                       /* 失能定时器x和定时器更新中断 */
}


void Tim4_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim4);                       /* 使能定时器x和定时器更新中断 */
}

void Tim4_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim4);                       /* 失能定时器x和定时器更新中断 */
}

void Tim5_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim5);                       /* 使能定时器x和定时器更新中断 */
}

void Tim5_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim5);                       /* 失能定时器x和定时器更新中断 */
}

void Tim6_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim6);                       /* 使能定时器x和定时器更新中断 */
}

void Tim6_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim6);                       /* 失能定时器x和定时器更新中断 */
}

void Tim7_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim7);                       /* 使能定时器x和定时器更新中断 */
}

void Tim7_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim7);                       /* 失能定时器x和定时器更新中断 */
}


void Tim1_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim1);                       /* 使能定时器x和定时器更新中断 */
}

void Tim1_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);                       /* 失能定时器x和定时器更新中断 */
}

void Tim8_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim8);                       /* 使能定时器x和定时器更新中断 */
}

void Tim8_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim8);                       /* 失能定时器x和定时器更新中断 */
}

void Tim2_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim2);                       /* 使能定时器x和定时器更新中断 */
}

void Tim2_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);                       /* 失能定时器x和定时器更新中断 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
    __HAL_RCC_TIM6_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
  if(tim_baseHandle->Instance==TIM7)
  {
    __HAL_RCC_TIM7_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM7_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
  if(tim_baseHandle->Instance==TIM5)
  {
    __HAL_RCC_TIM5_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM5_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
  if(tim_baseHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM4_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
  if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM3_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  if(tim_baseHandle->Instance==TIM1)
  {
    /* TIM1 clock enable */
//    __HAL_RCC_TIM1_CLK_ENABLE();

//    /* TIM1 interrupt Init */
//    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 12, 0);
//    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
//    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 12, 0);
//    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  }
  if(tim_baseHandle->Instance==TIM8)
  {
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  }
  if(tim_baseHandle->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  
  
}

/* USER CODE BEGIN 1 */
void TIM1_BRK_TIM9_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim1);
  //HAL_TIM_IRQHandler(&htim9);

}

void TIM1_UP_TIM10_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim1);

}

void TIM8_BRK_TIM12_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim8);
  //HAL_TIM_IRQHandler(&htim12);

}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim8);
}

void TIM7_IRQHandler(void)
{
	#if SYS_SUPPORT_OS                              /* 使用OS */
	uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
	#endif
	HAL_TIM_IRQHandler(&htim7);
	
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif
}

void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}


/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	

    if (htim->Instance == TIM6)
    {
		Dynsytimout++;
    }
	#if SYS_SUPPORT_OS                              /* 使用OS */
	uint32_t status_value=taskENTER_CRITICAL_FROM_ISR();//进入临界区，进入有一个返回值要保存起来
	#endif
	if (htim->Instance == TIM7)
    {
		switch(ws2812led.status)
		{
			case 0:	WS2812_Breathe_Control(0, 19, 0, 150,0,20, 76,0,0); break;//全未连接	
			case 1: WS2812_Breathe_Control(0, 38, 0, 150, 0, 39, 57,58,76); break;//手柄1连接，其它两个未连接	 
			case 2:	WS2812_Breathe_Control(0, 57, 0, 150, 0, 20, 38,58,76); break;//手柄2连接，其它两个未连接	
			case 3:	WS2812_Breathe_Control(0, 57, 0, 150, 0, 0, 0,58,76); break;//手柄12连接，脚踏未连接
			case 4:	WS2812_Breathe_Control(0, 76, 0, 150, 0, 20, 38,39,57); break;//脚踏连接，其它两个未连接
			case 5:	WS2812_Breathe_Control(0, 76, 0, 150, 0, 39, 57,0,0); break;//手柄1和脚踏连接，手柄2未连接
			case 6:	WS2812_Breathe_Control(0, 76, 0, 150, 0, 20, 38,0,0); break;//手柄2和脚踏连接，手柄1未连接
			case 7:	WS2812_Breathe_Control(0, 76, 0, 150,0,0, 0,0,0);  break;//全连接
			default: break;
			
		}
    }
	#if SYS_SUPPORT_OS                              /* 使用OS */
	taskEXIT_CRITICAL_FROM_ISR(status_value);
	#endif	
    if (htim->Instance == TIM4)
    {
		PumpTimFlag++;
    }
    if (htim->Instance == TIM3)
    {
		Errornumber++;
    }
    if (htim->Instance == TIM5)
    {
		g_timer_3s++;
		//printf("g_timer = %d\r\n",g_timer_3s);
    }
    if (htim->Instance == TIM1)
    {

    }
    if (htim->Instance == TIM8)
    {
		p_timer_3s++;
		//printf("%d\n",p_timer_3s);
    }
	if (htim->Instance == TIM2)
    {
		BistTimFlag++;
		
    }
	

}
