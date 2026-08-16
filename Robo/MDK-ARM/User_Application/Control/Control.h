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
	float Base_Triger_Speed;	//基础目标速度
	float Base_High_Speed_Set;	//基础高速度设置
	float Base_Low_Speed_Set;	//基础低速度设置
	float Accelerated_Speed;//加速度

	float Turn_Speed;			//转向速度
};
extern struct Motor_Control Motor_Control_Parm;

//前方障碍/路口标志位枚举
typedef enum{
	No_Front = 0,		//
	Intersection,		//路口
	Obstacle,			//障碍物	
	Pass_Test,			//通行识别
	Platform,			//平台
	Scenic_Spot,		//景点
	Seesaw,				//跷跷板
	Go_Up_Bridge,		//上升桥梁
	Dead_Zone,			//盲区
}Front_State;

//控制方式枚举
typedef enum{
	Direct_Control = 0,	//直接控制，直接传递电机速度
	ScanLine_Control,	//巡线控制，通过地面线路执行控制
	Angle_Control,		//角度控制，通过自身姿态执行调控
	Turn_Control,
}Control_Choice;

//旋转模式枚举
typedef enum{
	Turn_Left = 0,		//左转
	Turn_Right,			//右转
}Turn_Mode;

//运行状态结构体
struct RUN{
	char RUN_Control;			//运行控制，0停止，1允许运行
	char RUN_State;				//运行状态，0停止，1开始运行

	float Mileage_Int;			//里程累计
	char Path_Switching_Flag;	//路径切换标志位，为1时代表执行路径切换

	float Target_Angle;			//目标角度，用于角度控制时设置目标角度
	Control_Choice Control_State;	//各路径控制状态
	float Mileage_Parm;			//行驶数据存储区，对应行驶状态为直行时，表示行进里程，对应行驶状态为转向时，表示转向角度
	float Travel_State;			//行驶状态存储区
	Front_State	Front;			//障碍/路口标志位存储区
	int Path_Fill_Pointer;		//路径装填指针
	int Path_Pointer;			//路径指针

	int Sensor_Middle;			//中部传感器触发状态
	int Sensor_Left;			//左侧光电传感器触发状态
	int Sensor_Right;			//右侧光电传感器触发状态
	int Microswitch_State;		//微动开关状态

	int Common_Traffic_Sign;	//通行指示牌序号
};
extern struct RUN RUN_Parm;

void RUN_Parm_Init(void);		//运行参数初始化
void RUN_Speed_Control(void);	//运行速度控制

void Mileage_Int_Compute(void);	//里程累计函数

void GPIO_Trigger_Control(uint32_t GPIO_Pin);	//GPIO中断触发任务

void Car_Turn_Control(Turn_Mode Mode,float Turn_Angle);	//转弯控制

void RUN_System_Control(void);			//系统运行控制任务

void Go_Up_Platform(void);				//上平台
void Go_Down_Platform(void);			//下平台

void Platform1_to_Platform2(void);				//平台1至平台2

void Platform2_to_Scenic_Spot2(void);			//平台2至景点2


#endif
