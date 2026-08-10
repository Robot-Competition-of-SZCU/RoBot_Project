///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 14th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为灰度ADC采集头文件
///////////////////////////////////////
#ifndef __Grayscale_ADC_H__
#define __Grayscale_ADC_H__

#include "stm32f4xx_hal.h"

#define Calibrate_Sampling_Num	100			//校准时采样次数，采样一次为1ms
#define Grayscale_Num			14			//灰度传感器数量设定
#define Grayscale_Weight_Bias	1			//从中心点开始，每个传感器的偏移权重

//灰度传感器ADC数据缓存区
extern short Grayscale_ADC_Buffer[16];

//灰度传感器ADC参数结构体
struct Grayscale_ADC{
	short Grayscale_ADC_Actual[16];	//灰度传感器ADC实际数据，从左至右依次对应每个传感器
	short Grayscale_ADC_Max[16];	//灰度传感器数据最大极限值
	short Grayscale_ADC_Min[16];	//灰度传感器数据最小极限值
	short Grayscale_ADC_Compute[16];//灰度传感器数据校准值
	float Grayscale_ADC_Compute_Percent[16];	//灰度传感器数据校准后百分比
	float Grayscale_ADC_Trigger_Threshold;		//灰度传感器触发阈值
	char Grayscale_Trigger_State[16];			//灰度传感器触发状态
	char Compute_Map_Mode;						//映射计算模式，0：正向计算(线位置传感器值为高)，1：反向计算(线位置传感器值为低)
	float Grayscale_Map;						//灰度传感器ADC映射一维坐标值
};
extern struct Grayscale_ADC Grayscale;

typedef enum{
	Calibration_Max = 0,
	Calibration_Min
}Calibration_Mode;


void Grayscale_ADC_Init(void);			//灰度传感器ADC采集初始化
HAL_StatusTypeDef Grayscale_ADC_Calibration(Calibration_Mode Mode);	//灰度传感器ADC校准
void Grayscale_ADC_Compute(void);		//灰度传感器ADC解算

#endif
