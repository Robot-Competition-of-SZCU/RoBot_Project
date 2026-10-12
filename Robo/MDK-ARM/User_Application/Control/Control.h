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
#include "Grayscale_ADC.h"

//设定里程100时，其实际行进的距离
#define Set_Mileage_100_Actual	120.0f

// //加减速参数
// #define Advance_Accelerated_Speed	20.0f		//前进加速度
// #define Advance_Deceleration_Speed	16.0f		//前进减速度

// #define Retreat_Accelerated_Speed	16.0f		//后退加速度
// #define Retreat_Deceleration_Speed	20.0f		//后退减速度

// #define Turn_Accelerated_Speed		15.0f		//旋转加速度
// #define Turn_Deceleration_Speed		15.0f		//旋转减速度

//加减速参数结构体
struct Accelerated_Speed{
	float Advance_A_Speed;		//前进加速度
	float Advance_D_Speed;		//前进减速度

	float Retreat_A_Speed;		//后退加速度
	float Retreat_D_Speed;		//后退减速度

	float Turn_A_Speed;			//旋转加速度
	float Turn_D_Speed;			//旋转减速度

	float Turn_Right_Ratio;		//右转时，转速/角速比率，单位：角速度/轮速
	float Turn_Left_Ratio;		//左转时，转速/角速比率，单位：角速度/轮速
};
extern struct Accelerated_Speed AD_S;

//轮速与车体旋转角速度换算系数：角速度(度/s) = 轮速(圈/s) × 该系数
//系数 = 57.2958 × 轮周长(cm) / 旋转半径(cm)，旋转半径 = 左右轮距的一半，需按实车标定
#define Turn_Angle_Speed_Coefficient	250.0f

//弧线转弯时，反侧轮转速比率
#define Turn_Arc_Ratio		0.1f

//目标角度向右修正量，单位度，所有存储目标角度的操作统一使用
#define TARGET_ANGLE_RIGHT_OFFSET	-0.1f

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
	float Base_Triger_Speed;	//目标基础速度
	float Turn_Speed;			//转向速度
};
extern struct Motor_Control Motor_Control_Parm;

typedef enum{
	Car_Advance = 0,	//小车前进
	Car_Retreat,		//小车后退
	Car_Turn_Right,		//小车右转
	Car_Turn_Left,		//小车左转
	Car_Stop,			//小车停止
}Car_Control_Mode;

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

//角度跟随控制模式枚举
typedef enum{
	RUN_Advance = 0,	//前进模式
	RUN_Retreat,		//后退模式
}Car_RUN_Mode;

//运行结束减速模式枚举
typedef enum{
	End_Start_Deceleration = 0,	//终点开始减速
	End_Finish_Deceleration,	//终点完成减速
}RUN_END_Deceleration_Mode;

//角度比较模式枚举
typedef enum{
	Less = 0,	//小于模式
	Greater,	//大于模式
}Angle_Compare_Mode;

//灰度触发模式枚举
typedef enum{
	Trigger = 0,	//触发
	No_Trigger,		//无触发
}Grayscale_Trigger_Mode;

//运行状态结构体
struct RUN{
	char RUN_Control;			//运行控制，0停止，1允许运行
	char RUN_State;				//运行状态，0停止，1开始运行

	float Mileage_Int;			//里程累计
	char Mileage_State;			//里程记录状态，0运行状态，1校准状态

	Car_RUN_Mode Car_RUN_Direction;	//行进方向

	float Target_Angle;			//目标角度，用于角度控制时设置目标角度
	Control_Choice Control_State;	//各路径控制状态

	float Arc_Turn_Target_Angle;	//弧线转弯目标角度
	float Arc_Turn_Resume_Speed;	//弧线转弯完成后恢复的速度
	float Arc_Turn_End_Threshold;	//弧线转弯完成判定阈值，单位度
	float Mileage_Parm;			//行驶数据存储区，对应行驶状态为直行时，表示行进里程，对应行驶状态为转向时，表示转向角度
	float Travel_State;			//行驶状态存储区


	char Round;					//回合
};
extern struct RUN RUN_Parm;

//外部输入信号结构体
struct INPut_State{
	int Sensor_Middle_Int;			//中部传感器触发计数
	int Sensor_Left_Int;			//左侧光电传感器触发计数
	int Sensor_Right_Int;			//右侧光电传感器触发计数
	int Microswitch_State_Int;		//微动开关触发计数
};
extern struct INPut_State INPut_Signal;

//视觉识别数据结构体
struct Visual_Identity_Data{
	char Common_Traffic_Sign;	//通行指示牌序号

	char Colour_Test;			//检测颜色
	char Word_Test;				//检测文字
	char Word_Test_Num;			//文字检测次数

	char Word_Flag;				//视觉识别文字标志位
	char Colour_Flag;			//视觉识别颜色标志位
};
extern struct Visual_Identity_Data Visual_Identity;

void RUN_Parm_Init(void);						//运行参数初始化
void RUN_Speed_Control(float DT);				//运行速度控制
void Car_Mode(Car_Control_Mode Control_Mode);	//小车状态控制

void Turn_Speed_Calibration(void);				//小车旋转速度校准

void Run_Record_Add(uint8_t Code);	//运行记录添加，1-8为平台，9-13为景点

void Mileage_Int_Compute(void);	//里程累计函数
void Scan_Line_RUN_Direct_Control(float RUN_Speed);	//巡线直接运行控制
void Scan_Line_RUN_Control(float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode);	//巡线运行控制
void Angle_Following_RUN_Control(Car_RUN_Mode Mode,float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode);	//角度跟随运行控制

void Mileage_Int_Test(void);		//里程测试函数

void GPIO_Trigger_Control(uint32_t GPIO_Pin);	//GPIO中断触发任务

void Word_Test_Recognition(void);		//文本识别
char Colour_Test_Recognition(void);		//颜色识别

void Get_Now_Z_Angle(float Err);						//获取当前Z轴角度
void Wait_Side_Sensor_Trigger(void);					//等待两侧光电传感器触发
void Wait_Middle_Sensor_Trigger(void);					//等待中部光电传感器触发
void Wait_Microswitch_Trigger(void);					//等待两侧微动开关触发
void Wait_X_Angle(Angle_Compare_Mode Mode,float Angle);	//等待到达设定角度
void Wait_Grayscale_State(Grayscale_Trigger_Mode Mode);	//等待灰度传感器状态
void Break_Line_Scan(Turn_Mode Mode,float Scan_Speed);	//断线扫描
void Wait_Appoint_Intersection(Intersection Intersection_Mode);	//等待到达指定路口
void Wait_Disperse_Intersection(void);					//等待到达分散路口
void Wait_Join_Intersection(void);						//等待到达交汇路口
void Wait_Appoint_Line(char Line_Num);					//等待存在指定线数
void RUN_Speed_Set(float Speed);						//速度设置

void Posture_Protect(void);								//姿态保护

void Car_Turn_Control(Turn_Mode Mode,float Turn_Angle,float Turn_Speed);	//转弯控制
void Car_Arc_Turn_Control(Turn_Mode Mode,float Turn_Angle,float Turn_Speed);	//行进中弧线转弯控制，不停顿
void Mileage_Arrive_Wait(float Mileage);//等待到达设定里程



#endif
