#include "foot_task.h"
#include "includes.h"
#include <stdlib.h>   // 用于 abs() 绝对值函数
#include <math.h>     // 用于 round() 四舍五入函数
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define FOOT_PRINTF(format, ...)     //Debug_Printf("【FOOT_TASK】:"format "\r\n",##__VA_ARGS__)

/* PUMP_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
 
#define FOOT_PRIO      	CONFIG_FOOT_PRIO                   		    /* 任务优先级 */
#define FOOT_STK_SIZE  	CONFIG_FOOT_STK_SIZE                         /* 任务堆栈大小 */
TaskHandle_t            FOOTTask_Handler;  							/* 任务句柄 */
void FOOT_TASK(void *pvParameters);             						/* 任务函数 */


#define BUFFER_SIZE 20  // 缓冲区容量

#define CALIBRATION   1200  	// 传感器标定数值
#define	FALLINGEDGE   35	// 传感器下降沿总幅值

uint16_t buffsensval[BUFFER_SIZE] = {0};



// 一阶低通滤波结构体
typedef struct {
    float alpha;    // 滤波系数（0 < alpha <= 1）
    float last_out; // 上一次输出
} LPF1D;

// 初始化
void lpf1DInit(LPF1D *lpf, float alpha, float init_val) {
    lpf->alpha = alpha;
    lpf->last_out = init_val;
}

// 滤波更新（每次采样调用一次）
float lpf1DUpdate(LPF1D *lpf, float input) {
    lpf->last_out = lpf->alpha * input + (1 - lpf->alpha) * lpf->last_out;
    return lpf->last_out;
}



//电机速度部分卡尔曼算法
KalmanFilter1D kf;


//压力传感器
LPF1D lpf;


FOOTVAR	footvar = {
	75000,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	2,
	4,
	0,
	4,
	0,
	0,
	0,
};







// 卡尔曼一阶滤波器
// 线性映射函数：电压值 -> 整数
// 参数：
//   v_in: 输入电压值（浮点数）	
//   v_min: 电压最小值（范围下限）
//   v_max: 电压最大值（范围上限）
//   n_max: 输出整数最大值（范围上限，n_min 固定为0）
// 返回值：映射后的整数（0 ~ n_max）
uint32_t voltage_to_integer(float v_in, float v_min, float v_max, uint32_t n_max) {
    // 校验输入范围合法性
    if (v_max <= v_min) {
        FOOT_PRINTF("错误：电压最大值必须大于最小值！\n");
        return 0; // 返回异常值
    }

    // 限制输入电压在 [v_min, v_max] 范围内（防止超出范围导致结果异常）
    if (v_in < v_min) v_in = v_min;
    if (v_in > v_max) v_in = v_max;

    // 线性映射计算
    float ratio = (v_in - v_min) / (v_max - v_min); // 归一化到 0~1
    int result = round(ratio * n_max);              // 缩放并四舍五入为整数

    return result;
}


// 卡尔曼一阶滤波器
void kalmanFilter1DInit(KalmanFilter1D* kf, double initial_x, double initial_P,
    double Q, double R, double F, double H) {
    if (kf == NULL) return;

    kf->x = initial_x;  // 初始状态估计
    kf->P = initial_P;  // 初始估计误差协方差
    kf->Q = Q;          // 过程噪声协方差
    kf->R = R;          // 测量噪声协方差
    kf->F = F;          // 状态转移系数
    kf->H = H;          // 测量系数
    kf->K = 0.0;        // 初始化卡尔曼增益
}

void kalmanFilter1DInitDefault(KalmanFilter1D* kf, double initial_x,
    double initial_P, double Q, double R) {
    // 默认F和H为1.0，适用于大多数一维场景
    kalmanFilter1DInit(kf, initial_x, initial_P, Q, R, 1.0, 1.0);
}

void kalmanFilter1DPredict(KalmanFilter1D* kf) {
    if (kf == NULL) return;

    // 预测状态: x' = F * x
    kf->x = kf->F * kf->x;

    // 预测误差协方差: P' = F * P * F + Q
    kf->P = kf->F * kf->F * kf->P + kf->Q;
}

double kalmanFilter1DUpdate(KalmanFilter1D* kf, double z) {
    if (kf == NULL) return 0.0;

    // 计算卡尔曼增益: K = P' * H / (H * P' * H + R)
    kf->K = kf->P * kf->H / (kf->H * kf->H * kf->P + kf->R);

    // 更新状态估计: x = x' + K * (z - H * x')
    kf->x = kf->x + kf->K * (z - kf->H * kf->x);

    // 更新估计误差协方差: P = (1 - K * H) * P'
    kf->P = (1 - kf->K * kf->H) * kf->P;

    return kf->x;
}

double kalmanFilter1DGetState(const KalmanFilter1D* kf) {
    if (kf == NULL) return 0.0;
    return kf->x;
}

void kalmanFilter1DSetQ(KalmanFilter1D* kf, double Q) {
    if (kf != NULL) {
        kf->Q = Q;
    }
}

void kalmanFilter1DSetR(KalmanFilter1D* kf, double R) {
    if (kf != NULL) {
        kf->R = R;
    }
}




/**
 * @brief 环形缓冲区写入、极值维护及严格下降沿时序判定
 */
void shift_and_add_ring(uint16_t buffer[],
                        uint16_t *current_size,
                        uint16_t *write_index,  // 环形缓冲区的当前写入物理位置
                        uint16_t *max_value,
                        uint16_t *min_value,
                        uint16_t *max_index,
                        uint16_t *min_index,
                        uint16_t new_data) {
    
    uint16_t idx = *write_index;
    bool is_full = (*current_size >= BUFFER_SIZE);
    bool need_recalc = false;

    // 1. 如果缓冲区已满，检查即将被覆盖的最老数据是否是当前的极值
    if (is_full) {
        if (idx == *max_index || idx == *min_index) {
            need_recalc = true; 
        }
    }

    // 2. 将新数据直接覆盖写入，无内存搬移（O(1)）
    buffer[idx] = new_data;

    // 3. 更新写指针与当前缓冲区大小
    *write_index = (idx + 1) % BUFFER_SIZE;
    if (!is_full) {
        (*current_size)++;
    }

    // 4. 动态维护最大值、最小值及其物理索引
    if (*current_size == 1) {
        // 迎接第一个数据，初始化极值
        *max_value = new_data;
        *min_value = new_data;
        *max_index = idx;
        *min_index = idx;
    } 
    else if (need_recalc) {
        // 【最坏情况】：原极值被覆盖，全盘重新寻找极值（O(N)）
        *max_value = buffer[0];
        *min_value = buffer[0];
        *max_index = 0;
        *min_index = 0;
        for (uint16_t i = 1; i < BUFFER_SIZE; i++) {
            if (buffer[i] > *max_value) {
                *max_value = buffer[i];
                *max_index = i;
            }
            if (buffer[i] < *min_value) {
                *min_value = buffer[i];
                *min_index = i;
            }
        }
    } 
    else {
        // 【常规/未满情况】：老极值未被破坏，用新数据做增量对比（O(1)）
        if (new_data >= *max_value) {
            *max_value = new_data;
            *max_index = idx;
        }
        if (new_data <= *min_value) {
            *min_value = new_data;
            *min_index = idx;
        }
    }
}




// 移动缓冲区数据并添加新元素（新增极值索引参数，最大值和最小值指针参数）
void shift_and_add(uint16_t buffer[],
                   uint16_t *current_size,
                   uint16_t *max_value,
                   uint16_t *min_value,
                   uint16_t *max_index,
                   uint16_t *min_index,
                   uint16_t new_data) {
    if (*current_size >= BUFFER_SIZE) {
        float removed_element = buffer[0];

        // 前移元素（只移动 BUFFER_SIZE-1 次）
        for (int i = 0; i < BUFFER_SIZE - 1; i++) {
            buffer[i] = buffer[i + 1];
        }
        // 新数据放入末尾（索引 BUFFER_SIZE-1）
        buffer[BUFFER_SIZE - 1] = (uint16_t)new_data;

        // 极值更新
        if (removed_element == *max_value || removed_element == *min_value) {
            // 重新计算
            *max_value = buffer[0];
            *min_value = buffer[0];
            *max_index = 0;
            *min_index = 0;
            for (int i = 1; i < BUFFER_SIZE; i++) {
                if (buffer[i] > *max_value) {
                    *max_value = buffer[i];
                    *max_index = i;
                }
                if (buffer[i] < *min_value) {
                    *min_value = buffer[i];
                    *min_index = i;
                }
            }
        } else {
            if (new_data > *max_value) {
                *max_value = new_data;
                *max_index = BUFFER_SIZE - 1;
            }
            if (new_data < *min_value) {
                *min_value = new_data;
                *min_index = BUFFER_SIZE - 1;
            }
        }
    } else {
        buffer[*current_size] = (uint16_t)new_data;
        if (*current_size == 0) {
            *max_value = new_data;
            *min_value = new_data;
            *max_index = 0;
            *min_index = 0;
        } else {
            if (new_data > *max_value) {
                *max_value = new_data;
                *max_index = *current_size;
            }
            if (new_data < *min_value) {
                *min_value = new_data;
                *min_index = *current_size;
            }
        }
        (*current_size)++;
    }
}




//电机速度和脚踏采集电压值对照控制函数
static void foot_motorspeed_control(void)
{
	uint16_t adcx;
	float temp;

	if(footvar.footCheckFlag && (footvar.pressflag == 0)) //是否接入脚踏并且压力阈值是否达到
	{
		switch(hmivar.num_flag)	//屏幕挡位是多少对应脚踏就是多少
		{
			
			case 0:	
					{
						switch(hmivar.Dynsysta)	//判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = hmivar.DynsystraSpe;		//手动设置挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 2:
									{
										footvar.foot_gear = hmivar.DynsyReveSpe;		//手动设置挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = hmivar.DynsySRSpe;		//手动设置挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							default:	break;
						}

						break;							
					}
			case 1:	
					{
						switch(hmivar.Dynsysta)	//判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_1;		//1挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_1;		//1挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_1;		//1挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							default:	break;
						}

						break;							
					}
			case 2: 
					{	
						switch(hmivar.Dynsysta)	//判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_2;		//2挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										FOOT_PRINTF("temp = %f",temp);
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数

										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_2;		//2挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数

										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_2;		//2挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
										
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							default:	break;
						}	
						break;
					}
			case 3: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_3;		//3挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_3;		//3挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_3;		//3挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}
			case 4: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_4;		//4挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_4;		//4挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_4;		//4挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}	
			case 5: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_5;		//5挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_5;		//5挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_5;		//5挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}
			case 6: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_6;		//6挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_6;		//6挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_6;		//6挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}	
			case 7: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_7;		//7挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_7;		//7挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_7;		//7挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}
			case 8: 
					{
						switch(hmivar.Dynsysta) //判断转向
						{
							case 1: 						
									{
										footvar.foot_gear = GRINDING_8;		//8挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							case 2:
									{
										footvar.foot_gear = GRINDING_8;		//8挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
																		
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
									}
									break;
							case 3: 
									{
										footvar.foot_gear = PLANING_8;		//8挡位
										adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
										temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
										
										// 更新步骤
										double filtered = kalmanFilter1DUpdate(&kf, temp);
										
										float result = round(temp * 100) / 100.0f;		//保留两位小数
									
										footvar.retur = voltage_to_integer(result,volthres,volthres_max,footvar.foot_gear);	//获取对应电机转速
										
									}
									break;
							default:	break;
						}
						break;
					}
					
			default:	footvar.retur = 0; break;	//挡位取消 	关闭电机
		}
	}
	else
	{ footvar.retur = 0;}
	
	
	if(hmivar.handelsta == 1)  //切换手柄过程发送停止指令
	{

		footvar.retur = 0; 			
		DynsySta.speed_num = 0;	//转速清零
//		switch(hmivar.Dynsysta) //判断停止转向
//		{
//			case 1: DynsySta.CycleSend = 4;  DynsySta.CycleStraSped = hmivar.DynsystraSpe;   break;
//			case 2: DynsySta.CycleSend = 5;  DynsySta.CycleReveSped = hmivar.DynsyReveSpe;   break;
//			case 3: DynsySta.CycleSend = 6;  DynsySta.CycleSRSped = hmivar.DynsySRSpe;     break;	
//			default:	break;
//		}

	}

	if(footvar.retur > 0)	//脚踏踩下
	{
			footvar.foot_flag = 0;		//脚踏踩下标志置位
			switch(hmivar.num_flag)
			{
				case 0:	
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
							break;
						}
				case 1:	
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
							break;
						}
				case 2: 
						{	
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}						
							break;
						}
				case 3: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
	
							break;
						}	
				case 4: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
	
							break;
						}
				case 5: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
	
							break;
						}
				case 6: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
	
							break;
						}
				case 7: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
	
							break;
						}
				case 8: 
						{
							switch(hmivar.Dynsysta) //判断转向
							{
								case 1: DynsySta.CycleStraSped = footvar.retur;  DynsySta.CycleSend = 1;   break;
								case 2: DynsySta.CycleReveSped = footvar.retur;  DynsySta.CycleSend = 2;  break;
								case 3: DynsySta.CycleSRSped = footvar.retur;  DynsySta.CycleSend = 3;  break;	
								default:	break;
							}
							break;
						}
				default:	break;
			}
	}
	else	//脚踏完全松开
	{
		//if(!footvar.foot_flag)	//判断脚踏
		{
			//FOOT_PRINTF("关闭电机");
			footvar.foot_flag = 1;	//脚踏松开标志
			DynsySta.speed_num = 0;	//转速清零
			switch(hmivar.Dynsysta) //判断停止转向
			{
				case 1: DynsySta.CycleSend = 4;  DynsySta.CycleStraSped = hmivar.DynsystraSpe;   break;
				case 2: DynsySta.CycleSend = 5;  DynsySta.CycleReveSped = hmivar.DynsyReveSpe;   break;
				case 3: DynsySta.CycleSend = 6;  DynsySta.CycleSRSped = hmivar.DynsySRSpe;     break;	
				default:	break;
			}
		}
	}

	if(footvar.pressensor)	//压力传感正在使用
	{
		static uint16_t numb = 0;	//数组缓冲区计数	
		static uint16_t write_idx = 0; // 新增：环形缓冲区写入索引位置
		static uint16_t value_max = 0;
		static uint16_t value_min = 0xFFFF;
		static uint16_t max_index = 0;
		static uint16_t min_index = 0;

		if(pairsval.pressvalue2 > CALIBRATION 
		   && pairsval.pressvalue2 < 4096)  // ★ 加合理范围过滤，滤掉野值
		{
			//FOOT_PRINTF(" 压力值 = %d",pairsval.pressvalue2);
			shift_and_add(buffsensval,&numb,&value_max,&value_min,&max_index,&min_index,pairsval.pressvalue2);	//赋值给缓冲区数组
			if(numb >= BUFFER_SIZE)	//数组元素大于缓存长度
			{		
				uint16_t diff;
				if(min_index > max_index)
				{
					diff  = value_max-value_min;
					//FOOT_PRINTF("压力差值 = %d\n",diff);
					if(diff > FALLINGEDGE)		//总高度达到15则停止
					{
						numb = 0;
						FOOT_PRINTF("停止！！ 压力差值 = %d\n",diff);
						diff = 0;
						memset(buffsensval, 0, BUFFER_SIZE);		//先清空一下缓冲区
						footvar.pressflag = 1; //压力值达到停止电机
						DynsySta.StaRece = PRESSURE;
					}
				}
				else 
				{
					diff = 0;		//总距离清零
				}						
			}
		}	
		if(footvar.pressflag == 1)
		{
			adcx = adc_get_result_average(ADC_ADCX_CHY, 10);    /* 获取通道5的转换值，10次取平均 */
			temp = (float)adcx * (volthres_js / 4096);                  /* 获取计算后的带小数的实际电压值，比如3.1111 */
			
			// 更新步骤
			double filtered = kalmanFilter1DUpdate(&kf, temp);
			float result = round(temp * 100) / 100.0f;		//保留两位小数
			
			if(result < 1.0f)
			{
				footvar.pressflag = 0;
			}
			else
			{
				
			}
		}
	}
	
}







//脚踏按钮逻辑控制
static void foot_motor_control(void)
{
	//电机事件
	if(footvar.EXTI_footFlag)
	{
		static uint8_t gerflag = 0;
		if((!gerflag) && (BistTimFlag > 3))
		{
			if(key_scan(1,0) == 1)
			{
				gerflag = 1;
			}
			else
			{
				BistTimFlag = 0;
				gerflag = 0;
				Tim2_Stop();
			}
		}
		else if((gerflag) && (BistTimFlag < 1500) && (BistTimFlag > 20))
		{
			if(key_scan(1,0) == 0)
			{
				footvar.footGearChangeflag = 1;	//置位
				Tim2_Stop();
				BistTimFlag = 0;
				gerflag = 0;

				footvar.EXTI_footFlag = 0;
				hmivar.num_flag++;
				if(hmivar.num_flag > 8)
				{
					hmivar.num_flag = 1;
				}
				FOOT_PRINTF("切换挡位 = %d",hmivar.num_flag);
				BEEP_ON;
			}
		}
		else if((gerflag) && (BistTimFlag > 2000))
		{
			Tim2_Stop();
			footvar.foot_patternflag = 1;	//置位
			hmivar.Dynsysta++;
			if(hmivar.Dynsysta > 3)
			{
				hmivar.Dynsysta = 1;
			}
			FOOT_PRINTF("切换模式 = %d",hmivar.Dynsysta);
			BistTimFlag = 0;
			gerflag = 0;
			footvar.EXTI_footFlag = 0;
			BEEP_ON;
		}
	}
	
	//蠕动泵事件
	if(footvar.EXTI_pumpFlag)
	{
		static uint8_t pumpflagsta = 0;
		if((!pumpflagsta) && (PumpTimFlag > 3))
		{
			if(key_scan(1,1) == 2)
			{
				pumpflagsta = 1;
			}
			else
			{
				PumpTimFlag = 0;
				pumpflagsta = 0;
				Tim4_Stop();
			}
		}
		else if((pumpflagsta) && (PumpTimFlag < 1500) && (PumpTimFlag > 20))
		{
			if(key_scan(1,1) == 0)
			{
				if(!footvar.pump_tims_global_power)
				{
					footvar.pump_tims_global_power = !footvar.pump_tims_global_power;
					pumpsta.run = 1; //发送给蠕动泵开始指令
					footvar.pump_powerflag = 1;	    //图标置位
					footvar.pumpautomaticflag = 1;	//置位状态改变
					FOOT_PRINTF("蠕动泵开");
					BEEP_ON;
				}
				else
				{
					footvar.pump_tims_global_power = !footvar.pump_tims_global_power;
					pumpsta.stop = 1; //发送给蠕动泵停止指令
					FOOT_PRINTF("蠕动泵关");
					footvar.pump_powerflag = 2;	//图标置位
					footvar.pumpautomaticflag = 1;	//置位状态改变
					BEEP_ON;
				}
				Tim4_Stop();
				PumpTimFlag = 0;
				pumpflagsta = 0;
				footvar.EXTI_pumpFlag = 0;
			}
		}
		else if((pumpflagsta) && (PumpTimFlag > 2000))
		{
			footvar.pump_tims_global++;
			Tim4_Stop();
			if(footvar.pump_tims_global > 7)	//次数
			{
				footvar.pump_tims_global = 1;
			}
			footvar.pumpautomaticflag = 1;	//置位状态改变
			pumpsta.gear = footvar.pump_tims_global;	//发送给蠕动泵当前挡位指令
			footvar.pumpGearChangeflag = footvar.pump_tims_global; //赋值当前挡位
			PumpTimFlag = 0;
			pumpflagsta = 0;
			footvar.EXTI_pumpFlag = 0;
			FOOT_PRINTF("蠕动泵切换挡位 = %d",pumpsta.gear);
			BEEP_ON;
		}
	}
	
}


//检测脚踏是否插入
uint8_t foot_check(void)
{
	if(key_scan(1,2) == 3)
	{
		footvar.footCheckFlag = 1;
	}
	else
	{
		footvar.footCheckFlag = 0;
	}
	return footvar.footCheckFlag;
}




void foot_task_init(void)
{
	taskENTER_CRITICAL();           /* 进入临界区 */
    /* 创建任务1 */
    xTaskCreate((TaskFunction_t )FOOT_TASK,
                (const char*    )"FOOT_TASK",
                (uint16_t       )FOOT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )FOOT_PRIO,
                (TaskHandle_t*  )&FOOTTask_Handler);
    taskEXIT_CRITICAL();            /* 退出临界区 */
}




/**
 * @brief       FOOT_TASK
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void FOOT_TASK(void *pvParameters)
{
	extix_init();
	adc_init();
	Tim2_Init(1000-1,84-1);				//定时查看按键1触发 ，定时1ms
	Tim4_Init(1000-1,84-1);				//定时查看按键2触发 ，定时1ms
	
	
	// 初始化：alpha=0.8，初始值1450
	lpf1DInit(&lpf, 0.8f, 1450.0f);

    // 初始化滤波器
    // 初始估计值设为0，初始协方差设为10，过程噪声0.01，测量噪声10

    kalmanFilter1DInitDefault(&kf, 0, 10, 0.01, 10);
		// 预测步骤
	kalmanFilter1DPredict(&kf);
	
    while(1)
    {
		foot_check(); //检测脚踏
		foot_motorspeed_control();	//脚踏速度和挡位控制
		foot_motor_control();		//脚踏挡位和速度，蠕动泵挡位和速度切换函数
        vTaskDelay(1);                                           /* 延时1ticks */ 
    }
}
