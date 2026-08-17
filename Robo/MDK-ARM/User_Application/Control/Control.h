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

	float Base_Speed;			//基础速度
	float Base_Triger_Speed;	//基础目标速度
	float Base_High_Speed_Set;	//基础高速度设置
	float Base_Low_Speed_Set;	//基础低速度设置
	float Accelerated_Speed;	//加速度

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
	Turn_Control,		//原地转向控制
	Arc_Turn_Control,	//行进中弧线转弯控制，不停顿
}Control_Choice;

//旋转模式枚举
typedef enum{
	Turn_Left = 0,		//左转
	Turn_Right,			//右转
}Turn_Mode;

typedef enum{
	Angle_Control_Advance = 0,	//前进模式
	Angle_Control_Retreat,		//后退模式
}Angle_Control_Mode;

//运行状态结构体
struct RUN{
	char RUN_Control;			//运行控制，0停止，1允许运行
	char RUN_State;				//运行状态，0停止，1开始运行

	float Mileage_Int;			//里程累计
	char Path_Switching_Flag;	//路径切换标志位，为1时代表执行路径切换

	Angle_Control_Mode Angle_Control_Choice;	//角度控制模式

	float Target_Angle;			//目标角度，用于角度控制时设置目标角度
	Control_Choice Control_State;	//各路径控制状态

	float Arc_Turn_Target_Angle;	//弧线转弯目标角度
	float Arc_Turn_Resume_Speed;	//弧线转弯完成后恢复的速度
	float Arc_Turn_End_Threshold;	//弧线转弯完成判定阈值，单位度
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

void RUN_System_Control(void);			//系统运行控制任务


void Car_Turn_Control(Turn_Mode Mode,float Turn_Angle);	//转弯控制
void Car_Arc_Turn_Control(Turn_Mode Mode,float Turn_Angle);//行进中弧线转弯控制，不停顿
void Stop_Control(void);				//停止控制
void Mileage_Arrive_Wait(float Mileage);//等待到达设定里程

void Go_Up_Platform(void);				//上平台
void Go_Down_Platform(void);			//下平台
void Hit_The_Scenic_Spot(void);			//撞景点
void Turn_Around(void);					//原地转向
void Cross_the_Mountain(void);			//翻越山
void Cross_the_Long_Wave_Board(void);	//翻越长波浪板
void Cross_the_Short_Wave_Board(void);	//翻越短波浪板

void Platform1_to_Platform2(void);				//平台1至平台2
void Platform2_to_Scenic_Spot2(void);			//平台2至景点2
void Scenic_Spot2_to_Platform4(void);			//景点2至平台4
void Platform4_to_Scenic_Spot1(void);			//平台4至景点1
void Scenic_Spot1_to_Platform3(void);			//景点1至平台3

void Platform3_to_Test_A(void);					//平台3至A点

void Test_C_to_Scenic_Spot4(void);				//C点至景点4
void Scenic_Spot4_to_Scenic_Spot5(void);		//景点4至景点5
void Scenic_Spot5_to_Test_D(void);				//景点5至D点

void Test_D_to_Platform5(void);					//D点至平台5
void Platform5_to_Platform7(void);				//平台5至平台7
void Platform7_to_Platform8(void);				//平台7至平台8
void Platform8_to_Scenic_Spot3(void);			//平台8至景点3
void Scenic_Spot3_to_Test_C(void);				//景点3至C点

void Aplha_to_Platform1(void);					//Alpha点至平台1

void Test_C_to_Test_A_to_Alpha(void);			//C点至A点至Alpha
void Test_C_to_Test_B_to_Alpha(void);			//C点至B点至Alpha
void Test_D_to_Test_A_to_Alpha(void);			//D点至A点至Alpha
void Test_D_to_Test_B_to_Alpha(void);			//D点至B点至Alpha


#endif
