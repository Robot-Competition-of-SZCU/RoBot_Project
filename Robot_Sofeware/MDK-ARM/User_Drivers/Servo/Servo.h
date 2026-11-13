///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为舵机驱动头文件
///////////////////////////////////////
#ifndef __Servo_H__
#define __Servo_H__

//舵机编号枚举
typedef enum{
	Servo1 = 0,
	Servo2,
	Servo3,
	Servo4,
	Servo5,
	Servo6,
	Servo7,
	Servo8
}Servo_Number;

//控制方向枚举
typedef enum{
	Forward = 0,	//正向
	Inversion		//反向
}Servo_Direction;

//使能枚举
typedef enum{
	Servo_Disable = 0,	//失能
	Servo_Enable		//使能
}Servo_EN_DI;

//舵机状态结构体
struct Servo{
	char Servo_EN_State[8];			//舵机使能状态
	char Servo_Direction_State[8];	//舵机方向
	float Servo_Angle[8];			//舵机角度
	float Servo_Angle_Max_Limit[8];	//舵机最大角度限位
	float Servo_Angle_Min_Limit[8];	//舵机最小角度限位
};
extern struct Servo Servo_State;

void Servo_Init(void);		//舵机初始化
void Servo_Start_One(Servo_Number Servo_Select);	//开启单独舵机
void Servo_Stop_One(Servo_Number Servo_Select);		//关闭单独舵机
void Servo_Control_One(Servo_Number Servo_Select,Servo_Direction Direction,float Angle);	//舵机独立控制
void Servo_Control(void);	//舵机总控制

#endif
