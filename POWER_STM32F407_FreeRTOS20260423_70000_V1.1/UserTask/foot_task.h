#ifndef __FOOT_TASK_H
#define __FOOT_TASK_H

#include "./SYSTEM/sys/sys.h"




#define 	Debug_Printf(...)       printf(__VA_ARGS__)


#define		volthres 	1.0

#define		volthres_max 	1.75

#define		volthres_js 	3.3

#include <stdio.h>
/**
 * 一维卡尔曼滤波器结构体
 * 包含滤波器的所有状态和参数
 */
typedef struct {
    // 状态估计值（当前最优估计）
    double x;

    // 估计误差协方差（状态不确定性）
    double P;

    // 过程噪声协方差（模型不确定性）
    double Q;

    // 测量噪声协方差（传感器不确定性）
    double R;

    // 卡尔曼增益
    double K;

    // 状态转移系数（通常为1.0）
    double F;

    // 测量系数（通常为1.0）
    double H;
} KalmanFilter1D;

/**
 * 初始化一维卡尔曼滤波器
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *   initial_x - 初始状态估计值
 *   initial_P - 初始估计误差协方差
 *   Q - 过程噪声协方差
 *   R - 测量噪声协方差
 *   F - 状态转移系数（默认1.0）
 *   H - 测量系数（默认1.0）
 */
void kalmanFilter1DInit(KalmanFilter1D* kf, double initial_x, double initial_P,
    double Q, double R, double F, double H);

/**
 * 使用默认参数初始化一维卡尔曼滤波器
 * F和H默认设为1.0
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *   initial_x - 初始状态估计值
 *   initial_P - 初始估计误差协方差
 *   Q - 过程噪声协方差
 *   R - 测量噪声协方差
 */
void kalmanFilter1DInitDefault(KalmanFilter1D* kf, double initial_x,
    double initial_P, double Q, double R);

/**
 * 卡尔曼滤波预测步骤
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 */
void kalmanFilter1DPredict(KalmanFilter1D* kf);

/**
 * 卡尔曼滤波更新步骤
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *   z - 新的测量值
 *
 * 返回值:
 *   更新后的状态估计值
 */
double kalmanFilter1DUpdate(KalmanFilter1D* kf, double z);

/**
 * 获取当前的状态估计值
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *
 * 返回值:
 *   当前的状态估计值
 */
double kalmanFilter1DGetState(const KalmanFilter1D* kf);

/**
 * 设置过程噪声协方差
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *   Q - 新的过程噪声协方差值
 */
void kalmanFilter1DSetQ(KalmanFilter1D* kf, double Q);

/**
 * 设置测量噪声协方差
 *
 * 参数:
 *   kf - 卡尔曼滤波器结构体指针
 *   R - 新的测量噪声协方差值
 */
void kalmanFilter1DSetR(KalmanFilter1D* kf, double R);


typedef	struct{

uint32_t	foot_gear;				//脚踏挡位
uint8_t 	foot_flag;				//脚踏使用状态机
uint32_t 	retur;					//转速
uint16_t 	footSpinFlag;			//旋转模式
uint16_t 	EXTI_footFlag;			//挡位开关事件标志位
uint16_t 	EXTI_pumpFlag;			//蠕动泵事件标志位
uint8_t 	foot_patternflag;		//模式状态
uint8_t 	footGearChangeflag;		//挡位状态
uint8_t		footCheckFlag;			//脚踏是否插入状态
uint8_t 	pump_powerflag;			//蠕动泵开关状态
uint8_t 	pumpGearChangeflag;		//蠕动泵挡位状态
uint8_t 	pumpautomaticflag;		//自动改变状态
uint8_t 	pump_tims_global;			//全局蠕动泵默认4档
uint8_t 	pump_tims_global_power;		//全局蠕动泵开关


}FOOTVAR;


//uint32_t	foot_gear = 70000;	//脚踏挡位
//uint8_t 	foot_flag = 0;		//脚踏使用状态机
//uint32_t 	retur = 0;			//电机设定转速
//uint16_t 	footSpinFlag = 0;	//旋转模式
//uint16_t 	EXTI_footFlag = 0;	//挡位开关事件标志位
//uint16_t 	EXTI_pumpFlag = 0;	//蠕动泵事件标志位
//uint8_t     foot_patternflag = 0;		//电机模式状态
//uint8_t     footGearChangeflag = 0;		//电机挡位状态
//uint8_t		footCheckFlag = 0;	//脚踏是否插入状态
//uint8_t     pump_powerflag = 0;			//蠕动泵开关状态
//uint8_t     pumpGearChangeflag = 2;		//默认2挡位蠕动泵状态
//uint8_t     pumpautomaticflag = 0;		//图标自动上传改变的状态
//uint8_t 	pump_tims_global = 2;	//全局蠕动泵默认2档
//uint8_t 	pump_tims_global_power = 1;		//全局蠕动泵开关

extern FOOTVAR	footvar;

void foot_task_init(void);

#endif








