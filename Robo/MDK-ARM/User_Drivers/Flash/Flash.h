///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 14th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为Flash操作头文件
///////////////////////////////////////
#ifndef __Flash_H__
#define __Flash_H__

#include "stm32f4xx_hal.h"

#define DATA_SECTOR_ADDR	0x080E0000	//数据存储扇区的起始地址，其为STM32F047ZGT6单片机Flash的最后一个扇区的起始地址

//Flash存储数据结构体
struct Flash{
	//灰度传感器数据
	short Grayscale_ADC_Max[16];	//灰度传感器数据最大极限值
	short Grayscale_ADC_Min[16];	//灰度传感器数据最小极限值
	char Compute_Map_Mode;			//映射计算模式，0：正向计算(线位置传感器值为高)，1：反向计算(线位置传感器值为低)
	float Grayscale_ADC_Trigger_Threshold;		//灰度传感器触发阈值
	
	//舵机数据
	char Servo_EN_State[8];			//舵机使能状态
	char Servo_Direction_State[8];	//舵机方向
	float Servo_Angle[8];			//舵机初始角度
	float Servo_Angle_Max_Limit[8];	//舵机最大角度限位
	float Servo_Angle_Min_Limit[8];	//舵机最小角度限位

	//运行数据
	float Base_High_Speed_Set;	//基础速度设置
	float Accelerated_Speed;//加速度
};

HAL_StatusTypeDef Flash_Write(void);		//Flash写数据
void Flash_Read(void);						//Flash读数据

#endif
