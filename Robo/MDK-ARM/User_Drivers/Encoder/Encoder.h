///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为编码器采集头文件
///////////////////////////////////////
#ifndef __Encoder_H__
#define __Encoder_H__

#include "stm32f4xx_hal.h"

#define	TIM_Cycle_pixel			0.000001f	//圈/频率
#define Length_Of_Each_Circle	32.672563f	//轮子周长，单位cm
#define Encoder_Resolution		48			//编码器分辨率，原始分辨率12，AB两相，上下边沿均记录，故为12*2*2
#define	Motor_Reduction_Ratio	44			//电机减速比
#define Input_Capture_Cycle		0.005f		//输入捕获计算周期

#define Encoder_Filter_Alpha	0.18f		//编码器频率一阶低通滤波参数，1为实际值

extern char Encoder4_B_TIM4_CH4_Data_Pointer;	//编码器4通道B（TIM4_通道4）CCR数据传输伪指针

extern uint16_t CCR_Buffer[8];		//编码器CCR缓存区
extern uint16_t CCR_Input_Pulse[8];	//CCR触发脉冲计数

//编码器结构体
struct Encoder{
	int Pulse;					//脉冲数
	int Pulse_Int;				//脉冲累计
	
	float Actual_Frequiency;	//实际频率
	float Filter_Frequiency;	//滤波后频率
	
	float RMP_S;				//轮子旋转速度,圈每秒
	float RMP_Int;				//轮子旋转圈数累计
	
	float Speed;				//线速度，单位cm/s
	float RUN_Int;				//行进距离累计
};
extern struct Encoder	Encoder1;
extern struct Encoder	Encoder2;
extern struct Encoder	Encoder3;
extern struct Encoder	Encoder4;

void Encoder_Init(void);		//编码器初始化
void Encoder_Compute(void);		//编码器计算
void Encoder_Input_Frequiency_Filter(void);		//编码器输入频率滤波
void Encoder_Speed_Compute(void);				//编码器速度计算


#endif
