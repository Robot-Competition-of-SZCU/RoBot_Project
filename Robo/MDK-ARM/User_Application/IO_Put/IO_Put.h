///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 21th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为IO输入输出头文件
///////////////////////////////////////
#ifndef __IO_Put_H__
#define __IO_Put_H__

#include "stm32f4xx_hal.h"

//输入输出端口宏定义
//需注意，PIN0-PIN7为PF端口，PIN8-PIN15为PE端口
#define Sensor_Middle		GPIO_PIN_7		//中部光电传感器，对应7号端口
#define Sensor_Left			GPIO_PIN_6		//左侧光电传感器，对应6号端口
#define Sensor_Right		GPIO_PIN_5		//右侧光电传感器，对应5号端口
#define Microswitch_Left	GPIO_PIN_4		//左侧微动开关，对应4号端口
#define Microswitch_Right	GPIO_PIN_3		//右侧微动开关，对应3号端口

void GPIO_Trigger_Control(uint32_t GPIO_Pin);		//GPIO中断触发任务


#endif
