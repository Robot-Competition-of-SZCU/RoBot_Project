///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 13th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为PID算法源文件
///////////////////////////////////////
#include "PID.h"
#include "stm32f4xx_hal.h"

#include "Encoder.h"
#include "Motor.h"
#include "Grayscale_ADC.h"

#include "IMU.h"
#include "Control.h"

//电机PID控制结构体
PID_Incremental PID_Motor1;
PID_Incremental PID_Motor2;
PID_Incremental PID_Motor3;
PID_Incremental PID_Motor4;

//编码器反馈丢失计数，供诊断显示使用
uint16_t Encoder_Lost_Times[4] = {0,0,0,0};

//巡线控制PID结构体
PID_Positional Line_Patrol_PID;	

//角度跟随控制PID
PID_Positional	Angle_Patrol_PID;

//弧线转弯控制PID
PID_Positional	Arc_Turn_PID;

/** @brief	PID参数初始化
  **/
void PID_Parameter_Init(void)
{
	//内环，电机速度控制，增量式PID
	//M1
	PID_Motor1.Kp 		= 43;		//比例系数
	PID_Motor1.Ki 		= 0.8f;		//积分系数
	PID_Motor1.Kd 		= 0;		//微分系数
	PID_Motor1.Out_Max	= 60;		//最大限幅
	PID_Motor1.Out_Min	= 0;		//最小限幅
	PID_Motor1.DT		= 5;		//调控周期，单位ms
	//M2
	PID_Motor2.Kp 		= 43;		//比例系数
	PID_Motor2.Ki 		= 0.8f;		//积分系数
	PID_Motor2.Kd 		= 0;		//微分系数
	PID_Motor2.Out_Max	= 60;		//最大限幅
	PID_Motor2.Out_Min	= 0;		//最小限幅
	PID_Motor2.DT		= 5;		//调控周期，单位ms
	//M3
	PID_Motor3.Kp 		= 43;		//比例系数
	PID_Motor3.Ki 		= 0.8f;		//积分系数
	PID_Motor3.Kd 		= 0;		//微分系数
	PID_Motor3.Out_Max	= 60;		//最大限幅
	PID_Motor3.Out_Min	= 0;		//最小限幅
	PID_Motor3.DT		= 5;		//调控周期，单位ms
	//M4
	PID_Motor4.Kp 		= 43;		//比例系数
	PID_Motor4.Ki 		= 0.8f;		//积分系数
	PID_Motor4.Kd 		= 0;		//微分系数
	PID_Motor4.Out_Max	= 60;		//最大限幅
	PID_Motor4.Out_Min	= 0;		//最小限幅
	PID_Motor4.DT		= 5;		//调控周期，单位ms

	//巡线PID参数设置
	Line_Patrol_PID.Kp = 0.08f;			//比例系数
	// Line_Patrol_PID.Ki = 0.0f;		//积分系数
	// Line_Patrol_PID.Kd = 0.0f;		//微分系数
	Line_Patrol_PID.ErrorInt_Min = -1; 	//积分最小限幅
	Line_Patrol_PID.ErrorInt_Max = +1;	//积分最大限幅
	Line_Patrol_PID.Out_Min = -1;		//输出最小限幅
	Line_Patrol_PID.Out_Max = 1;		//输出最大限幅

	//角度跟随PID参数设置，位置式PID
	Angle_Patrol_PID.Kp 		= 0.025f;	//比例系数
	Angle_Patrol_PID.Ki 		= 0.0f;		//积分系数
	Angle_Patrol_PID.Kd 		= 0;		//微分系数
	Line_Patrol_PID.ErrorInt_Min = -0.5f; 	//积分最小限幅
	Line_Patrol_PID.ErrorInt_Max = +0.5f;	//积分最大限幅
	Angle_Patrol_PID.Out_Min 	= -1;		//输出最小限幅
	Angle_Patrol_PID.Out_Max 	= 1;		//输出最大限幅

	//弧线转弯PID参数设置，位置式PID
	Arc_Turn_PID.Kp 		= 0.35f;	//比例系数
	Arc_Turn_PID.Ki 		= 0.0f;		//积分系数
	Arc_Turn_PID.Kd 		= 0.0f;		//微分系数
	Arc_Turn_PID.ErrorInt_Min = -0.5f; 	//积分最小限幅
	Arc_Turn_PID.ErrorInt_Max = +0.5f;	//积分最大限幅
	Arc_Turn_PID.Out_Min 	= -1;		//输出最小限幅
	Arc_Turn_PID.Out_Max 	= 1;		//输出最大限幅
}

/** @brief	角度跟随控制
  * @note	外环PID解算，输出速度
  **/
void Angle_Patrol_Control(void)
{
	//计算相对角度差
	float Angle = RUN_Parm.Target_Angle - IMU_Data.IMU_Angle_Z;
	if(Angle>180) Angle -= 360;
	if(Angle<-180)Angle +=360;


	//计算差速比
	Angle_Patrol_PID.Error_Last = Angle_Patrol_PID.Error_Now;	//上次误差更新
	Angle_Patrol_PID.Error_Now = Angle;							//本次误差更新
	PID_Positional_Compute(&Angle_Patrol_PID);					//PID计算
	
	//电机速度混合控制
	if(Motor_Control_Parm.Base_Speed)
	{	//基础速度不为0，执行控制
		Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Base_Speed + Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Base_Speed + Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Base_Speed - Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Base_Speed - Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		
		if(Motor_Control_Parm.Motor1_Speed < 0) Motor_Control_Parm.Motor1_Speed = 0;
		if(Motor_Control_Parm.Motor2_Speed < 0) Motor_Control_Parm.Motor2_Speed = 0;
		if(Motor_Control_Parm.Motor3_Speed < 0) Motor_Control_Parm.Motor3_Speed = 0;
		if(Motor_Control_Parm.Motor4_Speed < 0) Motor_Control_Parm.Motor4_Speed = 0;
		
		if(Motor_Control_Parm.Motor1_Speed > 10) Motor_Control_Parm.Motor1_Speed = 10;
		if(Motor_Control_Parm.Motor2_Speed > 10) Motor_Control_Parm.Motor2_Speed = 10;
		if(Motor_Control_Parm.Motor3_Speed > 10) Motor_Control_Parm.Motor3_Speed = 10;
		if(Motor_Control_Parm.Motor4_Speed > 10) Motor_Control_Parm.Motor4_Speed = 10;
		
	}
	else
	{	//基础速度为0，取消控制
		Motor_Control_Parm.Motor1_Speed = 0;
		Motor_Control_Parm.Motor2_Speed = 0;
		Motor_Control_Parm.Motor3_Speed = 0;
		Motor_Control_Parm.Motor4_Speed = 0;
	}
}

/** @brief	反向角度跟随控制
  * @note	外环PID解算，输出速度
  **/
void Angle_Patrol_Retreat_Control(void)
{
	//计算相对角度差
	float Angle = RUN_Parm.Target_Angle - IMU_Data.IMU_Angle_Z;
	if(Angle>180) Angle -= 360;
	if(Angle<-180)Angle +=360;


	//计算差速比
	Angle_Patrol_PID.Error_Last = Angle_Patrol_PID.Error_Now;	//上次误差更新
	Angle_Patrol_PID.Error_Now = Angle;							//本次误差更新
	PID_Positional_Compute(&Angle_Patrol_PID);					//PID计算
	
	//电机速度混合控制
	if(Motor_Control_Parm.Base_Speed)
	{	//基础速度不为0，执行控制
		Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Base_Speed - Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Base_Speed - Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Base_Speed + Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Base_Speed + Angle_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		
		if(Motor_Control_Parm.Motor1_Speed < 0) Motor_Control_Parm.Motor1_Speed = 0;
		if(Motor_Control_Parm.Motor2_Speed < 0) Motor_Control_Parm.Motor2_Speed = 0;
		if(Motor_Control_Parm.Motor3_Speed < 0) Motor_Control_Parm.Motor3_Speed = 0;
		if(Motor_Control_Parm.Motor4_Speed < 0) Motor_Control_Parm.Motor4_Speed = 0;
		
		if(Motor_Control_Parm.Motor1_Speed > 10) Motor_Control_Parm.Motor1_Speed = 10;
		if(Motor_Control_Parm.Motor2_Speed > 10) Motor_Control_Parm.Motor2_Speed = 10;
		if(Motor_Control_Parm.Motor3_Speed > 10) Motor_Control_Parm.Motor3_Speed = 10;
		if(Motor_Control_Parm.Motor4_Speed > 10) Motor_Control_Parm.Motor4_Speed = 10;
		
	}
	else
	{	//基础速度为0，取消控制
		Motor_Control_Parm.Motor1_Speed = 0;
		Motor_Control_Parm.Motor2_Speed = 0;
		Motor_Control_Parm.Motor3_Speed = 0;
		Motor_Control_Parm.Motor4_Speed = 0;
	}
}



/** @brief	巡线控制
  * @note	外环PID解算，输出速度
  **/
void Scan_Line_Control(void)
{
	//计算差速比
	Line_Patrol_PID.Error_Last = Line_Patrol_PID.Error_Now;		//上次误差更新
	Line_Patrol_PID.Error_Now = Grayscale.Grayscale_Map;		//本次误差更新
	PID_Positional_Compute(&Line_Patrol_PID);					//PID计算
	
	//电机速度混合控制
	if(Motor_Control_Parm.Base_Speed)
	{	//基础速度不为0，执行控制
		Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Base_Speed - Line_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Base_Speed - Line_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Base_Speed + Line_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Base_Speed + Line_Patrol_PID.Out * Motor_Control_Parm.Base_Speed;
		
		if(Motor_Control_Parm.Motor1_Speed < 0) Motor_Control_Parm.Motor1_Speed = 0;
		if(Motor_Control_Parm.Motor2_Speed < 0) Motor_Control_Parm.Motor2_Speed = 0;
		if(Motor_Control_Parm.Motor3_Speed < 0) Motor_Control_Parm.Motor3_Speed = 0;
		if(Motor_Control_Parm.Motor4_Speed < 0) Motor_Control_Parm.Motor4_Speed = 0;
		
		if(Motor_Control_Parm.Motor1_Speed > 10) Motor_Control_Parm.Motor1_Speed = 10;
		if(Motor_Control_Parm.Motor2_Speed > 10) Motor_Control_Parm.Motor2_Speed = 10;
		if(Motor_Control_Parm.Motor3_Speed > 10) Motor_Control_Parm.Motor3_Speed = 10;
		if(Motor_Control_Parm.Motor4_Speed > 10) Motor_Control_Parm.Motor4_Speed = 10;
	}
	else
	{	//基础速度为0，取消控制
		Motor_Control_Parm.Motor1_Speed = 0;
		Motor_Control_Parm.Motor2_Speed = 0;
		Motor_Control_Parm.Motor3_Speed = 0;
		Motor_Control_Parm.Motor4_Speed = 0;
	}
}

/** @brief	行进中弧线转弯控制
  * @note	保持前进的同时通过左右差速完成转向，不停顿
  * @note	由周期任务驱动，转弯完成后自动切换回巡线控制
  **/
void Arc_Turn_Control_Compute(void)
{
	//计算剩余角度，归一化至-180~+180
	float Angle = RUN_Parm.Arc_Turn_Target_Angle - IMU_Data.IMU_Angle_Z;
	while(Angle > 180) Angle -= 360;
	while(Angle < -180) Angle += 360;

	//转弯完成，无缝切换回巡线
	if(Angle > -RUN_Parm.Arc_Turn_End_Threshold && Angle < RUN_Parm.Arc_Turn_End_Threshold)
	{
		//恢复基础目标速度
		Motor_Control_Parm.Base_Triger_Speed = RUN_Parm.Arc_Turn_Resume_Speed;
		//复位巡线PID，避免历史误差冲击
		Line_Patrol_PID.Error_Now 	= 0;
		Line_Patrol_PID.Error_Last 	= 0;
		Line_Patrol_PID.ErrorInt 	= 0;
		Line_Patrol_PID.Out 		= 0;
		//切换回巡线控制
		RUN_Parm.Control_State = ScanLine_Control;
		return;
	}

	//角度闭环差速计算
	Arc_Turn_PID.Error_Last = Arc_Turn_PID.Error_Now;	//上次误差更新
	Arc_Turn_PID.Error_Now = Angle;						//本次误差更新
	PID_Positional_Compute(&Arc_Turn_PID);				//PID计算

	//保持前进的同时差速转向，M1M2为右侧，M3M4为左侧
	//Angle为正时目标在左，左侧减速右侧加速，实现左转
	Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Base_Speed + Arc_Turn_PID.Out * Motor_Control_Parm.Base_Speed;
	Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Base_Speed + Arc_Turn_PID.Out * Motor_Control_Parm.Base_Speed;
	Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Base_Speed - Arc_Turn_PID.Out * Motor_Control_Parm.Base_Speed;
	Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Base_Speed - Arc_Turn_PID.Out * Motor_Control_Parm.Base_Speed;

	//速度限幅，最小为0保持前进不停顿
	if(Motor_Control_Parm.Motor1_Speed < 0) Motor_Control_Parm.Motor1_Speed = 0;
	if(Motor_Control_Parm.Motor2_Speed < 0) Motor_Control_Parm.Motor2_Speed = 0;
	if(Motor_Control_Parm.Motor3_Speed < 0) Motor_Control_Parm.Motor3_Speed = 0;
	if(Motor_Control_Parm.Motor4_Speed < 0) Motor_Control_Parm.Motor4_Speed = 0;

	if(Motor_Control_Parm.Motor1_Speed > 10) Motor_Control_Parm.Motor1_Speed = 10;
	if(Motor_Control_Parm.Motor2_Speed > 10) Motor_Control_Parm.Motor2_Speed = 10;
	if(Motor_Control_Parm.Motor3_Speed > 10) Motor_Control_Parm.Motor3_Speed = 10;
	if(Motor_Control_Parm.Motor4_Speed > 10) Motor_Control_Parm.Motor4_Speed = 10;
}

/** @brief	内环编码器反馈异常防护
  * @param	Pid		PID结构体
  * @param	Lost_Times	丢失计数指针
  * @note	运行中输出已建立而反馈突然归零，判定编码器反馈丢失
  * @note	短时丢失：冻结PID输出，防止内环正反馈使输出猛然加大
  * @note	长期丢失：输出逐步衰减归零，防止编码器永久损坏时持续冲车
  **/
static void Encoder_Feedback_Protect(PID_Incremental* Pid,uint16_t* Lost_Times)
{
	//输出已建立(大于死区)、目标不为0、反馈却为0：运行中编码器反馈丢失
	if(Pid->Target != 0 && Pid->Out > 5.0f && Pid->Actual <= 0.05f)
	{
		//累计丢失时间
		(*Lost_Times)++;
		//连续丢失超过100ms，输出逐步衰减至0
		if(*Lost_Times >= 20)
		{
			Pid->Out -= 5.0f;
			if(Pid->Out < 0) Pid->Out = 0;
			*Lost_Times = 20;	//保持饱和，持续衰减
		}
		//短时间丢失：冻结输出，不执行PID计算
	}
	else
	{
		//反馈正常，恢复PID计算
		*Lost_Times = 0;
		PID_Incremental_Compute(Pid);
	}
}

/** @brief	电机速度控制
  * @note	内环PID解算，输出控制
  **/
void Motor_Speed_Control(void)
{
	//传递实际值，传递参数为电机实际转速
	//M1实际值
	PID_Motor1.Actual_Last_Last	= PID_Motor1.Actual_Last;
	PID_Motor1.Actual_Last 		= PID_Motor1.Actual;
	PID_Motor1.Actual 			= Encoder1.RMP_S;
	
	//M2实际值
	PID_Motor2.Actual_Last_Last	= PID_Motor2.Actual_Last;
	PID_Motor2.Actual_Last 		= PID_Motor2.Actual;
	PID_Motor2.Actual 			= Encoder2.RMP_S;
	
	//M3实际值
	PID_Motor3.Actual_Last_Last	= PID_Motor3.Actual_Last;
	PID_Motor3.Actual_Last 		= PID_Motor3.Actual;
	PID_Motor3.Actual 			= Encoder3.RMP_S;
	
	//M4实际值
	PID_Motor4.Actual_Last_Last	= PID_Motor4.Actual_Last;
	PID_Motor4.Actual_Last 		= PID_Motor4.Actual;
	PID_Motor4.Actual 			= Encoder4.RMP_S;
	
	//传递调控值，传递参数为电机目标转速
	PID_Motor1.Target = Motor_Control_Parm.Motor1_Speed;
	PID_Motor2.Target = Motor_Control_Parm.Motor2_Speed;
	PID_Motor3.Target = Motor_Control_Parm.Motor3_Speed;
	PID_Motor4.Target = Motor_Control_Parm.Motor4_Speed;
	
	//进行PID计算，输出参数为PWM占空比，带编码器反馈丢失防护
	Encoder_Feedback_Protect(&PID_Motor1,&Encoder_Lost_Times[0]);
	Encoder_Feedback_Protect(&PID_Motor2,&Encoder_Lost_Times[1]);
	Encoder_Feedback_Protect(&PID_Motor3,&Encoder_Lost_Times[2]);
	Encoder_Feedback_Protect(&PID_Motor4,&Encoder_Lost_Times[3]);
	
	//输出死区控制，目标值为0，则关断输出
	if(PID_Motor1.Target == 0) PID_Motor1.Out = 0;
	if(PID_Motor2.Target == 0) PID_Motor2.Out = 0;
	if(PID_Motor3.Target == 0) PID_Motor3.Out = 0;
	if(PID_Motor4.Target == 0) PID_Motor4.Out = 0;
	
	//执行控制
	Motor_Control_One(M1,Motor_Control_Parm.M1,PID_Motor1.Out);
	Motor_Control_One(M2,Motor_Control_Parm.M2,PID_Motor2.Out);
	Motor_Control_One(M3,Motor_Control_Parm.M3,PID_Motor3.Out);
	Motor_Control_One(M4,Motor_Control_Parm.M4,PID_Motor4.Out);
}

/** @brief	增量式PID运算函数
  * @param	Pid		PID结构体
  * @note	其包含二阶微分先行，调控周期自适应
  **/
void PID_Incremental_Compute(PID_Incremental* Pid)
{
	//获取偏差
	Pid->Error_Last	= Pid->Error_Now;				//计算上次偏差
	Pid->Error_Now	= Pid->Target - Pid->Actual;	//计算当前偏差
	
	//PID计算		
	//Kp_Out = Kp*(当前偏差-上次偏差)
	Pid->Kp_Out = Pid->Kp * (Pid->Error_Now - Pid->Error_Last);
	
	//Ki_Out = Ki*当前偏差*DT
	Pid->Ki_Out = (Pid->Ki *  Pid->Error_Now) * Pid->DT;
	
	//Kd_Out = Kd*(当前反馈 - 2倍上次反馈 + 上上次反馈)/DT
	Pid->Kd_Out = Pid->Kd * (Pid->Actual - 2*Pid->Actual_Last + Pid->Actual_Last_Last) / Pid->DT;
	
	//输出计算
	Pid->Out += (Pid->Kp_Out + Pid->Ki_Out - Pid->Kd_Out);
	
	//输出限幅
	if(Pid->Out > Pid->Out_Max)	Pid->Out = Pid->Out_Max;
	if(Pid->Out < Pid->Out_Min)	Pid->Out = Pid->Out_Min;
}

/** @brief	位置式PID运算函数
  * @param	Pid		PID结构体
  **/
void PID_Positional_Compute(PID_Positional*	Pid)
{
	Pid->ErrorInt+=Pid->Error_Now;		//计算偏差积分
	
	//积分限幅
	if(Pid->ErrorInt > Pid->ErrorInt_Max)	Pid->ErrorInt = Pid->ErrorInt_Max;
	if(Pid->ErrorInt < Pid->ErrorInt_Min)	Pid->ErrorInt = Pid->ErrorInt_Min;
	
	//PID计算		OUT	=	Kp*当前偏差	+	Ki*偏差积分	+	Kd*(当前偏差-上次偏差)
	Pid->Out	=	(Pid->Kp * Pid->Error_Now)
				+	(Pid->Ki * Pid->ErrorInt)
				+	(Pid->Kd * (Pid->Error_Now - Pid->Error_Last));

	//输出限幅
	if(Pid->Out > Pid->Out_Max)	Pid->Out = Pid->Out_Max;
	if(Pid->Out < Pid->Out_Min)	Pid->Out = Pid->Out_Min;

}


