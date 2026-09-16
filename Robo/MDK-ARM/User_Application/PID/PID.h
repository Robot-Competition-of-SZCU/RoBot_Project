///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 13th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为PID算法头文件
///////////////////////////////////////
#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

//增量式PID结构体
//用于电机转速控制
typedef struct		
{	
	float Kp;				//比例
	float Ki;				//积分
	float Kd;				//微分
	float Kp_Out;			//比例项输出
	float Ki_Out;			//积分项输出
	float Kd_Out;			//微分项输出
	
	float Error_Now;		//本次偏差
	float Error_Last;		//上次偏差
	float Target;			//目标值
	float Actual;			//实际值
	float Actual_Last;		//上次目标
	float Actual_Last_Last;	//上上次偏差
	float Out_Min;			//输出最小值限幅
	float Out_Max;			//输出最大值限幅
	float Out;				//输出值
	float DT;				//调控周期，单位ms
}PID_Incremental;

//电机PID控制结构体
extern PID_Incremental PID_Motor1;
extern PID_Incremental PID_Motor2;
extern PID_Incremental PID_Motor3;
extern PID_Incremental PID_Motor4;

//编码器反馈丢失计数，供诊断显示使用
extern uint16_t Encoder_Lost_Times[4];

//位置式PID结构体
typedef struct		
{	
	float Kp;				//比例
	float Ki;				//积分
	float Kd;				//微分
	float Error_Now;		//本次偏差
	float Error_Last;		//上次偏差
	float ErrorInt;			//偏差积分
	float ErrorInt_Min;		//积分最小值限幅
	float ErrorInt_Max;		//积分最大值限幅
	float Target;			//目标值
	float Out_Min;			//输出最小值限幅
	float Out_Max;			//输出最大值限幅

	float Out;				//输出值
}PID_Positional;

//巡线控制PID结构体
extern PID_Positional Line_Patrol_PID;	

//角度跟随控制PID
extern PID_Positional	Angle_Patrol_PID;

//弧线转弯控制PID
extern PID_Positional	Arc_Turn_PID;


void PID_Parameter_Init(void);							//PID参数初始化
void Motor_Speed_Control(void);							//电机速度控制
void Scan_Line_Control(void);							//巡线控制
void Angle_Patrol_Control(void);						//角度跟随控制
void Angle_Patrol_Retreat_Control(void);				//反向角度跟随控制
void Arc_Turn_Control_Compute(void);					//行进中弧线转弯控制

void PID_Incremental_Compute(PID_Incremental* Pid);		//增量式PID运算函数
void PID_Positional_Compute(PID_Positional*	Pid);		//位置式PID运算函数


#endif

