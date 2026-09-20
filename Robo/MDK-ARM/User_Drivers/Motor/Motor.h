///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为电机驱动头文件
///////////////////////////////////////
#ifndef __Motor_H__
#define __Motor_H__

//电机编号枚举
typedef enum{
	M1 = 0,
	M2,
	M3,
	M4,
}Motor_Number;

//电机运行状态枚举
typedef enum{
	Retreat = 0,	//前进（正转）
	Advance,		//后退（反转）
}Motor_State;

void Motor_Control_One(Motor_Number Motor_Select,Motor_State RUN_State,float Duty);	//电机独立控制
void Motor_Init(void);		//电机初始化
void Motor_Start(void);		//开启所有电机
void Motor_Stop(void);		//关闭所有电机

#endif
