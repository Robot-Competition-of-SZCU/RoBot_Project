///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 13th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为控制输出头文件
///////////////////////////////////////
#ifndef __Control_H__
#define __Control_H__

#include "Motor.h"

//电机控制状态结构体
struct Motor_Control{
	Motor_State M1;		//电机1运行方向
	Motor_State M2;		//电机2运行方向
	Motor_State M3;		//电机3运行方向
	Motor_State M4;		//电机4运行方向
	
	float Motor1_Speed;	//电机1目标速度，单位rmp/s
	float Motor2_Speed;	//电机2目标速度，单位rmp/s
	float Motor3_Speed;	//电机3目标速度，单位rmp/s
	float Motor4_Speed;	//电机4目标速度，单位rmp/s

	float Base_Speed;		//基础速度
	float Base_Speed_Set;	//基础速度设置
};
extern struct Motor_Control Motor_Control_Parm;

//运行状态结构体
struct RUN{
	char RUN_State;		//运行状态，0运行，1停止
};
extern struct RUN RUN_Parm;

void RUN_Parm_Init(void);		//运行参数初始化
void RUN_Control(void);			//运行控制



#endif
