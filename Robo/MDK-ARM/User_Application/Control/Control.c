///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为控制输出源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "Control.h"
#include "cmsis_os.h"
#include <stdlib.h>

#include "OLED.h"
#include "LED.h"
#include "IMU.h"
#include "PID.h"
#include "Encoder.h"
#include "Servo.h"
#include "Grayscale_ADC.h"
#include "UART.h"


//加减速参数结构体
struct Accelerated_Speed AD_S;
//控制状态结构体
struct Motor_Control Motor_Control_Parm;
//运行状态结构体
struct RUN RUN_Parm;
//外部输入信号结构体
struct INPut_State INPut_Signal;
//视觉识别数据结构体
struct Visual_Identity_Data Visual_Identity;


/** @brief	控制参数初始化
  **/
void RUN_Parm_Init(void)
{
	//小车前进设置
	Car_Mode(Car_Advance);
	RUN_Parm.Arc_Turn_End_Threshold = 3.0f;	//弧线转弯完成判定阈值
}

/** @brief	运行速度调节
  * @param  DT 该函数调控周期，单位ms
  **/
void RUN_Speed_Control(float DT)
{
	//运行状态下
    if(RUN_Parm.RUN_State)
    {
		//获取目标基础速度
		float Base_Triger_Speed = Motor_Control_Parm.Base_Triger_Speed;

		//当前速度小于目标速度，加速
		if(Motor_Control_Parm.Base_Speed < Base_Triger_Speed)
		{
			//前进模式
			if(RUN_Parm.Car_RUN_Direction == RUN_Advance)
				Motor_Control_Parm.Base_Speed += AD_S.Advance_A_Speed * DT/1000;	//进行加速
			//后退模式
			else
				Motor_Control_Parm.Base_Speed += AD_S.Retreat_A_Speed * DT/1000;	//进行加速
			//钳位，防止过冲
			if(Motor_Control_Parm.Base_Speed > Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed = Base_Triger_Speed;
		}
		//当前速度大于目标速度，减速
		else if(Motor_Control_Parm.Base_Speed > Base_Triger_Speed)
		{
			//前进模式
			if(RUN_Parm.Car_RUN_Direction == RUN_Advance)
				Motor_Control_Parm.Base_Speed -= AD_S.Advance_D_Speed * DT/1000;	//进行减速
			else
				Motor_Control_Parm.Base_Speed -= AD_S.Retreat_D_Speed * DT/1000;	//进行减速
			//钳位，防止过冲
			if(Motor_Control_Parm.Base_Speed < Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed = Base_Triger_Speed;
		}
    }
	// //非运行状态
    // else
    // {
	// 	//基础速度始终为0
    //     Motor_Control_Parm.Base_Speed = 0;
    // }
	if(RUN_Parm.RUN_Control == 0)
	{
		RUN_Parm.RUN_State = 0;
		//直接控制模式
		//RUN_Parm.Control_State = Direct_Control;
	}	
}

/** @brief	小车状态控制
  * @param	Control_Mode 	状态枚举
  **/
void Car_Mode(Car_Control_Mode Control_Mode)
{
	switch(Control_Mode)
	{
		case Car_Advance:
			Motor_Control_Parm.M1 = Advance;
			Motor_Control_Parm.M2 = Advance;
			Motor_Control_Parm.M3 = Advance;
			Motor_Control_Parm.M4 = Advance;
			break;
		case Car_Retreat:
			Motor_Control_Parm.M1 = Retreat;
			Motor_Control_Parm.M2 = Retreat;
			Motor_Control_Parm.M3 = Retreat;
			Motor_Control_Parm.M4 = Retreat;
			break;
		case Car_Turn_Right:
			Motor_Control_Parm.M1 = Retreat;
			Motor_Control_Parm.M2 = Retreat;
			Motor_Control_Parm.M3 = Advance;
			Motor_Control_Parm.M4 = Advance;
			break;
		case Car_Turn_Left:
			Motor_Control_Parm.M1 = Advance;
			Motor_Control_Parm.M2 = Advance;
			Motor_Control_Parm.M3 = Retreat;
			Motor_Control_Parm.M4 = Retreat;
			break;
		case Car_Stop:
		default:
			//关闭运行基速
			Motor_Control_Parm.Base_Triger_Speed = 0;
			Motor_Control_Parm.Base_Speed = 0;

			//关闭电机输出
			Motor_Control_One(M1,Motor_Control_Parm.M1,0);
			Motor_Control_One(M2,Motor_Control_Parm.M2,0);
			Motor_Control_One(M3,Motor_Control_Parm.M3,0);
			Motor_Control_One(M4,Motor_Control_Parm.M4,0);
			break;
	}
}

/** @brief	巡线直接运行控制
  * @param  RUN_Speed	运行速度
  **/
void Scan_Line_RUN_Direct_Control(float RUN_Speed)
{
	Car_Mode(Car_Advance);											//直行模式

	RUN_Parm.Control_State	= ScanLine_Control;				//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = RUN_Speed;		//速度设置，运行速度
}

/** @brief	巡线运行控制
  * @param  RUN_Speed	运行速度
  * @param  Stop_Speed	到达设定里程后速度
  * @param  Mileage		行进里程
  * @param  End_Mode	终点减速模式
  **/
void Scan_Line_RUN_Control(float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode)
{
	Car_Mode(Car_Advance);											//直行模式

	RUN_Parm.Control_State	= ScanLine_Control;				//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = RUN_Speed;		//速度设置，运行速度

	switch(End_Mode)
	{
		//终点完成减速模式，提前开始减速，到达终点时恰好减速完成
		case End_Finish_Deceleration:
		{
			PD4_LED_Control(LED_ON);								//开启PD4LED
			RUN_Parm.Mileage_Int = 0;						//复位里程计

			//在线等待到达终点，每1ms用当前实际速度判断是否到达减速起点
			//短里程时加速途中即开始减速，自然形成三角形速度曲线，无需平稳运行段
			while(RUN_Parm.Mileage_Int < Mileage)
			{
				//计算当前速度减速至终点速度所需里程：匀减速公式N=(V²-Stop²)/(2*a)
				//再乘轮周长换算为cm，并乘校准系数与里程计刻度对齐
				float Decel_Mileage = (Motor_Control_Parm.Base_Speed*Motor_Control_Parm.Base_Speed
									   - Stop_Speed*Stop_Speed)
									  / (2*AD_S.Advance_D_Speed)
									  * Length_Of_Each_Circle * (Set_Mileage_100_Actual / 100.0f);

				//到达减速起点，开始减速，速度斜坡由RUN_Speed_Control周期任务自动执行
				if(Motor_Control_Parm.Base_Speed > Stop_Speed
				   && RUN_Parm.Mileage_Int >= Mileage - Decel_Mileage)
				{
					Motor_Control_Parm.Base_Triger_Speed = Stop_Speed;
				}
				osDelay(1);
			}
			PD4_LED_Control(LED_OFF);						//关闭PD4LED
			break;
		}
		//终点开始减速模式，到达终点后才开始减速
		case End_Start_Deceleration:
		{
			//等待到达终点
			Mileage_Arrive_Wait(Mileage);

			//到达后开始减速
			Motor_Control_Parm.Base_Triger_Speed = Stop_Speed;		//速度设置，到达后速度
			break;
		}
	}
}

/** @brief	角度跟随运行控制
  * @param 	Mode		行进模式，前进或后退
  * @param  RUN_Speed	运行速度
  * @note	该函数在调用前需设定所跟随的角度
  **/
void Angle_Following_Direct_RUN_Control(Car_RUN_Mode Mode,float RUN_Speed)
{
	if(Mode == RUN_Advance)
	{
		Car_Mode(Car_Advance);		//前进模式
		RUN_Parm.Car_RUN_Direction = RUN_Advance;		//前进模式
	}
	else
	{
		Car_Mode(Car_Retreat);		//后退模式
		RUN_Parm.Car_RUN_Direction = RUN_Retreat;		//后退模式
	}

	RUN_Parm.Control_State	= Angle_Control;				//角度跟随控制
	Motor_Control_Parm.Base_Triger_Speed = RUN_Speed;		//速度设置，运行速度
}

/** @brief	角度跟随运行控制
  * @param 	Mode		行进模式，前进或后退
  * @param  RUN_Speed	运行速度
  * @param  Stop_Speed	到达设定里程后速度
  * @param  Mileage		行进里程
  * @param  End_Mode	终点减速模式
  * @note	该函数在调用前需设定所跟随的角度
  **/
void Angle_Following_RUN_Control(Car_RUN_Mode Mode,float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode)
{
	if(Mode == RUN_Advance)
	{
		Car_Mode(Car_Advance);		//前进模式
		RUN_Parm.Car_RUN_Direction = RUN_Advance;		//前进模式
	}
	else
	{
		Car_Mode(Car_Retreat);		//后退模式
		RUN_Parm.Car_RUN_Direction = RUN_Retreat;		//后退模式
	}

	RUN_Parm.Control_State	= Angle_Control;				//角度跟随控制
	Motor_Control_Parm.Base_Triger_Speed = RUN_Speed;		//速度设置，运行速度

	switch(End_Mode)
	{
		//终点完成减速模式，提前开始减速，到达终点时恰好减速完成
		case End_Finish_Deceleration:
		{
			PD4_LED_Control(LED_ON);						//开启PD4LED
			RUN_Parm.Mileage_Int = 0;						//复位里程计

			//在线等待到达终点，每1ms用当前实际速度判断是否到达减速起点
			//短里程时加速途中即开始减速，自然形成三角形速度曲线，无需平稳运行段
			while(RUN_Parm.Mileage_Int < Mileage)
			{
				//计算当前速度减速至终点速度所需里程：匀减速公式N=(V²-Stop²)/(2*a)
				//再乘轮周长换算为cm，并乘校准系数与里程计刻度对齐
				float Decel_Mileage = (Motor_Control_Parm.Base_Speed*Motor_Control_Parm.Base_Speed
									   - Stop_Speed*Stop_Speed)
									  / (2*AD_S.Advance_D_Speed)
									  * Length_Of_Each_Circle * (Set_Mileage_100_Actual / 100.0f);

				//到达减速起点，开始减速，速度斜坡由RUN_Speed_Control周期任务自动执行
				if(Motor_Control_Parm.Base_Speed > Stop_Speed
				   && RUN_Parm.Mileage_Int >= Mileage - Decel_Mileage)
				{
					Motor_Control_Parm.Base_Triger_Speed = Stop_Speed;
				}
				osDelay(1);
			}
			PD4_LED_Control(LED_OFF);						//关闭PD4LED
			break;
		}
		//终点开始减速模式，到达终点后才开始减速
		case End_Start_Deceleration:
		{
			//等待到达终点
			Mileage_Arrive_Wait(Mileage);

			//到达后开始减速
			Motor_Control_Parm.Base_Triger_Speed = Stop_Speed;		//速度设置，到达后速度
			break;
		}
	}
}

/** @brief	里程测试函数
  * @note	调用该函数将固定以1rmp的速度行进1m后瞬间停止
  * @note	该函数用于测量设定里程与实际里程偏差
  **/
void Mileage_Int_Test(void)
{
	//切换为校准模式
	RUN_Parm.Mileage_State = 1;

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
	Angle_Following_RUN_Control(RUN_Advance,3,0,100,End_Finish_Deceleration);	//进行移动
	Motor_Control_Parm.Base_Speed = 0;		//达到设定后直接停止

	//切换为运行模式
	RUN_Parm.Mileage_State = 0;
}

/** @brief	小车旋转速度校准
  * @param	Mode	校准模式
  * @retval	校准是否完成	
  * @note	通过此函数可获取小车轮速对对应小车整体旋转角速度
  * @note	该函数调用一次即进行一次数据采样
  **/
void Turn_Speed_Calibration(void)
{
	float Speed1_Angle_Int = 0,Speed2_Angle_Int = 0;
	float Speed3_Angle_Int = 0,Speed4_Angle_Int = 0;

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 90;

	//左转控制
	Car_Mode(Car_Turn_Left);

	//执行旋转
	//直接控制
	RUN_Parm.Control_State = Direct_Control;
	Motor_Control_Parm.Base_Speed = 1;

	//等待旋转稳定
	osDelay(1000);

	//进行100次采样
	for(char i=0;i<100;i++)
	{
		Speed1_Angle_Int += IMU_Data.IMU_Gyro_Z;
		osDelay(10);
	}
	Speed1_Angle_Int /= 100.0f;

	//速度2
	Motor_Control_Parm.Base_Speed = 3;
	//等待旋转稳定
	osDelay(1000);

	//进行100次采样
	for(char i=0;i<100;i++)
	{
		Speed2_Angle_Int += IMU_Data.IMU_Gyro_Z;
		osDelay(10);
	}
	Speed2_Angle_Int /= 100.0f;

	//停止
	Car_Mode(Car_Stop);

	//等待旋转稳定
	osDelay(1000);

	//右转控制
	Car_Mode(Car_Turn_Right);

	//速度3
	Motor_Control_Parm.Base_Speed = 1;
	//等待旋转稳定
	osDelay(1000);

	//进行100次采样
	for(char i=0;i<100;i++)
	{
		Speed3_Angle_Int += IMU_Data.IMU_Gyro_Z;
		osDelay(10);
	}
	Speed3_Angle_Int /= 100.0f;

	//速度4
	Motor_Control_Parm.Base_Speed = 3;
	//等待旋转稳定
	osDelay(1000);

	//进行100次采样
	for(char i=0;i<100;i++)
	{
		Speed4_Angle_Int += IMU_Data.IMU_Gyro_Z;
		osDelay(10);
	}
	Speed4_Angle_Int /= 100.0f;

	//停止
	Car_Mode(Car_Stop);

	//计算转向时，轮速对应角速度关系
	AD_S.Turn_Left_Ratio  = (Speed1_Angle_Int + Speed2_Angle_Int) / 4.0f;
	AD_S.Turn_Right_Ratio = (Speed3_Angle_Int + Speed4_Angle_Int) / 4.0f;

	//降下前瞻
	Servo_State.Servo_Angle[Servo2] = 35;
}

/** @brief	角度差回绕归一化
  * @note	将角度差归一化至 -180 ~ +180，始终取最短路径
  **/
static float Angle_Diff_Wrap(float Now,float Last)
{
	float Diff = Now - Last;
	while(Diff > 180) Diff -= 360;
	while(Diff < -180) Diff += 360;
	return Diff;
}

/** @brief	转弯控制
  * @param	Mode:转弯模式
  * @param	Turn_Angle:转弯角度，范围为0-180
  * @param	Turn_Speed	转向速度
  * @note	基于IMU角度增量累计，自动处理-180~+180角度回绕
  **/
void Car_Turn_Control(Turn_Mode Mode,float Turn_Angle,float Turn_Speed)
{
	//转向速度设置
	Motor_Control_Parm.Turn_Speed = Turn_Speed;
	RUN_Parm.Control_State	= Turn_Control;			//转弯控制

	Car_Mode(Car_Stop);									//停止

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 90;
	//等待抬起
	osDelay(50);

	//补偿
	Turn_Angle -= 2;

	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//设置电机转向
	if(Mode == Turn_Left)
	{
		Car_Mode(Car_Turn_Left);
	}
	else if(Mode == Turn_Right)
	{
		Car_Mode(Car_Turn_Right);
	}
	else
		return;

	//执行旋转，带加减速斜坡，终点完成减速
	float Now_Turn_Speed = 0;					//当前转向速度，起点为0
	float Turned_Angle = 0;						//已转角度
	float Last_Angle = IMU_Data.IMU_Angle_Z;		//上一次角度
	while(Turned_Angle < Turn_Angle)
	{
		//读取当前角度
		float Now_Angle = IMU_Data.IMU_Angle_Z;
		//计算角度增量，回绕归一化至-180~+180
		float Delta = Angle_Diff_Wrap(Now_Angle,Last_Angle);
		//取绝对值累计
		if(Delta < 0) Delta = -Delta;
		Turned_Angle += Delta;
		//更新上一次角度
		Last_Angle = Now_Angle;

		//计算剩余角度与当前速度减速到0所需角度
		float Remain_Angle = Turn_Angle - Turned_Angle;
		float Decel_Angle = Turn_Angle_Speed_Coefficient * Now_Turn_Speed * Now_Turn_Speed / (2 * AD_S.Turn_D_Speed);


		//目标速度：剩余角度不足以减速到0时为0，否则为设定转向速度
		float Target_Speed = (Remain_Angle <= Decel_Angle) ? 0 : Turn_Speed;

		//速度斜坡调节
		if(Now_Turn_Speed < Target_Speed)
		{
			Now_Turn_Speed += AD_S.Turn_A_Speed * 0.001f;	//进行加速，调节周期1ms
			if(Now_Turn_Speed > Target_Speed) Now_Turn_Speed = Target_Speed;	//钳位，防止过冲
		}
		else if(Now_Turn_Speed > Target_Speed)
		{
			Now_Turn_Speed -= AD_S.Turn_D_Speed * 0.001f;	//进行减速，调节周期1ms
			if(Now_Turn_Speed < Target_Speed) Now_Turn_Speed = Target_Speed;	//钳位，防止过冲
		}

		//执行旋转
		Motor_Control_Parm.Motor1_Speed = Now_Turn_Speed;
		Motor_Control_Parm.Motor2_Speed = Now_Turn_Speed;
		Motor_Control_Parm.Motor3_Speed = Now_Turn_Speed;
		Motor_Control_Parm.Motor4_Speed = Now_Turn_Speed;

		osDelay(1);
	}
	//停止旋转
	Motor_Control_Parm.Motor1_Speed = 0;
	Motor_Control_Parm.Motor2_Speed = 0;
	Motor_Control_Parm.Motor3_Speed = 0;
	Motor_Control_Parm.Motor4_Speed = 0;

	//降下前瞻
	Servo_State.Servo_Angle[Servo2] = 35;
	//等待降下
	osDelay(50);

	//调整为直行
	Car_Mode(Car_Advance);
}

/** @brief	行进中弧线转弯
  * @param	Mode		转弯模式
  * @param	Turn_Angle	转弯角度，范围为0-180
  * @param	Turn_Speed	转向速度
  * @note	不停顿转弯，保持前进的同时通过左右差速完成转向
  * @note	转弯完成后自动恢复原速度并切换回巡线控制
  **/
void Car_Arc_Turn_Control(Turn_Mode Mode,float Turn_Angle,float Turn_Speed)
{
	//转向速度设置
	Motor_Control_Parm.Turn_Speed = Turn_Speed;
	RUN_Parm.Control_State	= Turn_Control;			//转弯控制

	Car_Mode(Car_Stop);									//停止

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 45;
	//等待抬起
	osDelay(50);

	Turn_Angle += 5;

	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//转向速度保护，速度为0时无法转向，直接退出防止死循环
	if(Turn_Speed <= 0) return;

	//设置电机转向
	if(Mode == Turn_Left)
	{
		Car_Mode(Car_Turn_Left);
	}
	else if(Mode == Turn_Right)
	{
		Car_Mode(Car_Turn_Right);
	}
	else
		return;

	//执行旋转，带加减速斜坡，终点完成减速
	float Now_Turn_Speed = 0;					//当前转向速度，起点为0
	float Turned_Angle = 0;						//已转角度
	float Last_Angle = IMU_Data.IMU_Angle_Z;		//上一次角度
	while(Turned_Angle < (Turn_Angle - 5))
	{
		//读取当前角度
		float Now_Angle = IMU_Data.IMU_Angle_Z;
		//计算角度增量，回绕归一化至-180~+180
		float Delta = Angle_Diff_Wrap(Now_Angle,Last_Angle);
		//取绝对值累计
		if(Delta < 0) Delta = -Delta;
		Turned_Angle += Delta;
		//更新上一次角度
		Last_Angle = Now_Angle;

		//计算剩余角度与当前速度减速到0所需角度
		float Remain_Angle = Turn_Angle - Turned_Angle;
		float Decel_Angle = Turn_Angle_Speed_Coefficient * Now_Turn_Speed * Now_Turn_Speed / (2 * AD_S.Turn_D_Speed);


		//目标速度：剩余角度不足以减速到0时为0，否则为设定转向速度
		float Target_Speed = (Remain_Angle <= Decel_Angle) ? 0 : Turn_Speed;

		//速度斜坡调节
		if(Now_Turn_Speed < Target_Speed)
		{
			// Now_Turn_Speed += AD_S.Turn_A_Speed * 0.001f;	//进行加速，调节周期1ms
			// if(Now_Turn_Speed > Target_Speed) Now_Turn_Speed = Target_Speed;	//钳位，防止过冲

			Now_Turn_Speed = Target_Speed;
		}
		else if(Now_Turn_Speed > Target_Speed)
		{
			Now_Turn_Speed -= AD_S.Turn_D_Speed * 0.001f;	//进行减速，调节周期1ms
			if(Now_Turn_Speed < Target_Speed) Now_Turn_Speed = Target_Speed;	//钳位，防止过冲
		}

		//执行旋转
		if(Mode == Turn_Left)
		{
			Motor_Control_Parm.Motor1_Speed = Now_Turn_Speed;
			Motor_Control_Parm.Motor2_Speed = Now_Turn_Speed;
			//非90度转弯，不进行方向控制
			if(Turn_Angle >= 85)
			{
				Motor_Control_Parm.Motor3_Speed = Now_Turn_Speed * Turn_Arc_Ratio;
				Motor_Control_Parm.Motor4_Speed = Now_Turn_Speed * Turn_Arc_Ratio;
			}
			else
			{
				Motor_Control_Parm.Motor3_Speed = 0;
				Motor_Control_Parm.Motor4_Speed = 0;
			}
		}
		else if(Mode == Turn_Right)
		{
			//非90度转弯，不进行方向控制
			if(Turn_Angle >= 85)
			{
				Motor_Control_Parm.Motor1_Speed = Now_Turn_Speed * Turn_Arc_Ratio;
				Motor_Control_Parm.Motor2_Speed = Now_Turn_Speed * Turn_Arc_Ratio;
			}
			else
			{
				Motor_Control_Parm.Motor1_Speed = 0;
				Motor_Control_Parm.Motor2_Speed = 0;
			}
			Motor_Control_Parm.Motor3_Speed = Now_Turn_Speed;
			Motor_Control_Parm.Motor4_Speed = Now_Turn_Speed;
		}
		
		osDelay(1);
	}
	//停止旋转
	Motor_Control_Parm.Motor1_Speed = 0;
	Motor_Control_Parm.Motor2_Speed = 0;
	Motor_Control_Parm.Motor3_Speed = 0;
	Motor_Control_Parm.Motor4_Speed = 0;

	//降下前瞻
	Servo_State.Servo_Angle[Servo2] = 35;

	//调整为直行
	Car_Mode(Car_Advance);

	// //转向速度设置
	// Motor_Control_Parm.Turn_Speed = Turn_Speed;

	// //抬起前瞻
	// Servo_State.Servo_Angle[Servo2] = 45;

	// //限制转向角度范围
	// if(Turn_Angle > 180) Turn_Angle = 180;
	// if(Turn_Angle < 0) Turn_Angle = 0;

	// //记录转弯完成后的恢复速度
	// RUN_Parm.Arc_Turn_Resume_Speed = Motor_Control_Parm.Base_Triger_Speed;
	// //设置转弯基础速度
	// Motor_Control_Parm.Base_Triger_Speed = Motor_Control_Parm.Turn_Speed;

	// //计算目标角度，处理回绕
	// RUN_Parm.Arc_Turn_Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
	// RUN_Parm.Arc_Turn_Target_Angle += (Mode == Turn_Left) ? Turn_Angle : -Turn_Angle;
	// while(RUN_Parm.Arc_Turn_Target_Angle > 180) RUN_Parm.Arc_Turn_Target_Angle -= 360;
	// while(RUN_Parm.Arc_Turn_Target_Angle < -180) RUN_Parm.Arc_Turn_Target_Angle += 360;

	// //复位弧线转弯PID历史误差
	// Arc_Turn_PID.Error_Now = 0;
	// Arc_Turn_PID.Error_Last = 0;
	// Arc_Turn_PID.ErrorInt = 0;
	// Arc_Turn_PID.Out = 0;

	// //进入弧线转弯状态
	// RUN_Parm.Control_State = Arc_Turn_Control;

	// //阻塞等待转弯完成，转弯状态由周期任务驱动，完成后自动切回巡线
	// //等待期间执行减速控制：接近目标角度时开始减速，仅减速不加速
	// while(RUN_Parm.Control_State == Arc_Turn_Control)
	// {
	// 	//计算剩余角度，归一化至-180~+180
	// 	float Angle = RUN_Parm.Arc_Turn_Target_Angle - IMU_Data.IMU_Angle_Z;
	// 	while(Angle > 180) Angle -= 360;
	// 	while(Angle < -180) Angle += 360;
	// 	float Remain_Angle = (Angle >= 0) ? Angle : -Angle;	//剩余角度绝对值

	// 	//当前行进方向对应的轮速减速度
	// 	float Decel_Speed = (RUN_Parm.Car_RUN_Direction == RUN_Advance) ? AD_S.Advance_D_Speed : AD_S.Retreat_D_Speed;

	// 	//当前速度减速到0所需角度：角速度(度/s)=轮速×换算系数，匀减速公式N=ω²/(2*a)
	// 	float Decel_Angle = (Decel_Speed > 0)
	// 						? Turn_Angle_Speed_Coefficient * Motor_Control_Parm.Base_Speed * Motor_Control_Parm.Base_Speed / (2 * Decel_Speed)
	// 						: 0;

	// 	//到达减速起点，目标速度置0开始减速，仅减速不再加速
	// 	if(Motor_Control_Parm.Base_Triger_Speed > 0 && Remain_Angle <= Decel_Angle)
	// 		Motor_Control_Parm.Base_Triger_Speed = 0;

	// 	osDelay(1);
	// }

	// //降下前瞻
	// Servo_State.Servo_Angle[Servo2] = 35;

	// Car_Mode(Car_Stop);	//停止
}

/** @brief	等待到达设定里程
  * @param	Mileage: 里程设定，单位cm
  **/
void Mileage_Arrive_Wait(float Mileage)
{
	PD4_LED_Control(LED_ON);				//开启PD4LED
	RUN_Parm.Mileage_Int	= 0;			//复位里程计
	RUN_Parm.Mileage_Parm	= Mileage;		//里程设定
	//等待达到设定里程
	while(RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm) osDelay(1);	
	PD4_LED_Control(LED_OFF);				//关闭PD4LED
}

/** @brief	文本识别
  **/
void Word_Test_Recognition(void)
{
	//置文字识别标志位
	Visual_Identity.Word_Flag = 1;

	for(int i=0;i<5000;i++)
	{
		//等待
		if(Visual_Identity.Word_Flag == 0)
		{
			switch(Visual_Identity.Word_Test)
			{
				case 'D':	UART_Audio_Control(9);	//播报到达东岳泰山
						break;
				case 'X':	UART_Audio_Control(10);	//播报到达西岳华山
						break;
				case 'N':	UART_Audio_Control(11);	//播报到达南岳衡山
						break;
				case 'B':	UART_Audio_Control(12);	//播报到达北岳恒山
						break;
				case 'Z':	UART_Audio_Control(13);	//播报到达中岳嵩山
						break;
			}
			//检测次数自增
			Visual_Identity.Word_Test_Num++;
			//复位检测文字
			Visual_Identity.Word_Test = 0;
			break;
		}
		osDelay(1);
	}
	//5秒等待超时退出
}

/** @brief	颜色识别
  * @retval 0:不通过，1通过
  **/
char Colour_Test_Recognition(void)
{
	//置颜色识别标志位
	Visual_Identity.Colour_Flag = 1;

	for(int i=0;i<5000;i++)
	{
		//等待
		if(Visual_Identity.Colour_Flag == 0)
		{
			//检测到绿色
			if(Visual_Identity.Colour_Test == 'G')
				return 1;
			break;	
		}
		osDelay(1);
	}
	//5秒等待超时退出
	return 0;
}

/** @brief	获取当前Z轴角度
  * @param	Err	偏移，左正右负
  **/
void Get_Now_Z_Angle(float Err)
{
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
	//附加偏移
	RUN_Parm.Target_Angle += Err;
	//回绕处理
	if(RUN_Parm.Target_Angle > 180)
		RUN_Parm.Target_Angle -= 360;
	else if(RUN_Parm.Target_Angle < -180)
		RUN_Parm.Target_Angle += 360;
}

/** @brief	等待两侧光电传感器触发
  **/
void Wait_Side_Sensor_Trigger(void)
{
	//清除左右侧光电传感器触发状态
	INPut_Signal.Sensor_Left_Int = 0; INPut_Signal.Sensor_Right_Int = 0;
	//等待左右侧光电传感器触发
	while(!INPut_Signal.Sensor_Left_Int && !INPut_Signal.Sensor_Right_Int) osDelay(1);	
}

/** @brief	等待中部光电传感器触发
  **/
void Wait_Middle_Sensor_Trigger(void)
{
	//清除左右侧光电传感器触发状态
	INPut_Signal.Sensor_Middle_Int = 0;
	//等待左右侧光电传感器触发
	while(!INPut_Signal.Sensor_Middle_Int) osDelay(1);	
}

/** @brief	等待两侧微动开关触发
  **/
void Wait_Microswitch_Trigger(void)
{
	//清除左右侧光电传感器触发状态
	INPut_Signal.Microswitch_State_Int = 0;
	//等待左右侧光电传感器触发
	while(!INPut_Signal.Microswitch_State_Int) osDelay(1);	
}

/** @brief	等待到达设定角度
  * @param	Mode	比较模式
  * @param	Angle	比较角度
  **/
void Wait_X_Angle(Angle_Compare_Mode Mode,float Angle)
{
	//小于比较模式
	if(Mode == Less)
		//等待角度小于比较角度
		while(!(IMU_Data.IMU_Angle_X < Angle))	osDelay(1);
	//大于比较模式
	else if(Mode == Greater)
		//等待角度大于比较角度
		while(!(IMU_Data.IMU_Angle_X > Angle))	osDelay(1);
}

/** @brief	等待灰度传感器状态
  * @param	Mode	等待模式模式
  **/
void Wait_Grayscale_State(Grayscale_Trigger_Mode Mode)
{
	//等待触发
	if(Mode == Trigger)
		//等待角度小于比较角度
		while(!(Grayscale_Out.Grayscale_Trigger_Num))	osDelay(1);
	//等待无触发
	else if(Mode == No_Trigger)
		//等待角度大于比较角度
		while(Grayscale_Out.Grayscale_Trigger_Num)	osDelay(1);
}

/** @brief	等待到达指定路口
  * @param	Intersection_Mode	路口选择
  **/
void Wait_Appoint_Intersection(Intersection Intersection_Mode)
{
	//等待到达左直角路口
	if(Intersection_Mode == Left_Right_angle_Intersection)
		while(!((Grayscale_Out.Grayscale_Trigger_State[0] + Grayscale_Out.Grayscale_Trigger_State[1] +
			Grayscale_Out.Grayscale_Trigger_State[2]+ Grayscale_Out.Grayscale_Trigger_State[3]+
			Grayscale_Out.Grayscale_Trigger_State[4]+ Grayscale_Out.Grayscale_Trigger_State[5]+
			Grayscale_Out.Grayscale_Trigger_State[6]) > 5)) osDelay(1);
	else if(Intersection_Mode == Right_Right_angle_Intersection)
		while(!((Grayscale_Out.Grayscale_Trigger_State[13] + Grayscale_Out.Grayscale_Trigger_State[12] +
			Grayscale_Out.Grayscale_Trigger_State[11]+ Grayscale_Out.Grayscale_Trigger_State[10]+
			Grayscale_Out.Grayscale_Trigger_State[9]+ Grayscale_Out.Grayscale_Trigger_State[8]+
			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);
	else if(Intersection_Mode == Crossroad)
		while(!(Grayscale_Out.Grayscale_Trigger_Num >= 10))	osDelay(1);
}

/** @brief	等待到达分散路口
  **/
void Wait_Disperse_Intersection(void)
{
	//等待灰度触发>=4
	while(!(Grayscale_Out.Grayscale_Trigger_Num >= 4)) osDelay(1);
}

/** @brief	等待到达交汇路口
  **/
void Wait_Join_Intersection(void)
{
	//等待灰度触发>=4
	while(!(Grayscale_Out.Grayscale_Trigger_Num >= 4)) osDelay(1);
	//等待灰度触发<4
	while(!(Grayscale_Out.Grayscale_Trigger_Num < 4)) osDelay(1);
}

/** @brief	等待存在指定线数
  * @param	Line_Num	指定线数
  **/
void Wait_Appoint_Line(char Line_Num)
{
	//等待存在指定线数
	while(!(Grayscale_Out.Number_Of_Discontinuous_Points == Line_Num)) osDelay(1);
}

/** @brief	速度设置
  **/
void RUN_Speed_Set(float Speed)
{
	Motor_Control_Parm.Base_Triger_Speed = Speed;		//速度设置
}



/** @brief	姿态保护
  * @note	当姿态长时间处于非法角度时，关闭所有输出
  **/
void Posture_Protect(void)
{
	static int Err_Int;
	if(IMU_Data.IMU_Angle_X > 40 || IMU_Data.IMU_Angle_X < -40
	|| IMU_Data.IMU_Angle_Y > 40 || IMU_Data.IMU_Angle_Y < -40)
		Err_Int++;
	else
		Err_Int = 0;

	if(Err_Int >= 20)
	{	//执行保护
		RUN_Parm.RUN_Control = 0;
		RUN_Parm.RUN_State = 0;
		Car_Mode(Car_Stop);
	}
}

// /** @brief	C点至D点
//   * @note   起始地点需正对着4号景点
//   * @note   结束地点为正对着1号平台
//   **/
// void Test_C_to_Test_D(void)
// {
// 	Test_C_to_Scenic_Spot4();		//C点至景点4
// 	Word_Test_Recognition();		//文字识别
// 	Hit_The_Scenic_Spot();			//撞景点
// 	osDelay(1000);
// 	Scenic_Spot4_to_Scenic_Spot5();	//景点4至景点5
// 	Word_Test_Recognition();		//文字识别
// 	Hit_The_Scenic_Spot();			//撞景点
// 	osDelay(1000);
// 	Scenic_Spot5_to_Test_D();		//景点5至D点
// }

// /** @brief	D点至E点
//   * @note   起始地点需正对着5号平台
//   * @note   结束地点为正对着D点
//   **/
// void Test_D_to_Test_E(void)
// {
// 	//5号平台
// 	Test_D_to_Platform5();			//D点至平台5
// 	UART_Audio_Control(5);			//播报到达5号平台
// 	//osDelay(2000);

// 	//7号平台
// 	Turn_Around();					//原地转向
// 	Platform5_to_Platform7();		//平台5至平台7
// 	UART_Audio_Control(7);			//播报到达7号平台
// 	//osDelay(2000);

// 	//3号景点
// 	Turn_Around_For_8();					//原地转向
// 	Platform7_to_Scenic_Spot3_Change();		//平台7至景点3，改
// 	Word_Test_Recognition();		//文字识别
// 	Hit_The_Scenic_Spot();			//撞景点
// 	osDelay(1000);

// 	//8号平台
// 	Scenic_Spot3_to_Platform8_Change();	//景点3至平台8，改
// 	UART_Audio_Control(8);			//播报到达8号平台

// 	//至Test_E点
// 	Turn_Around_For_8();			//原地转向
// 	Platform8_to_Test_E_Change();	//平台8至E点，改
// }

// /** @brief	D点至C点
//   * @note   起始地点需正对着5号平台
//   * @note   结束地点为正对着D点
//   **/
// void Test_D_to_Test_C(void)
// {
// 	//5号平台
// 	Test_D_to_Platform5();			//D点至平台5
// 	UART_Audio_Control(5);			//播报到达5号平台
// 	//osDelay(2000);

// 	//7号平台
// 	Turn_Around();					//原地转向
// 	Platform5_to_Platform7();		//平台5至平台7
// 	UART_Audio_Control(7);			//播报到达7号平台
// 	//osDelay(2000);

// 	//8号平台
// 	Turn_Around();					//原地转向
// 	Platform7_to_Platform8();		//平台7至平台8
// 	UART_Audio_Control(8);			//播报到达8号平台
// 	//osDelay(2000);

// 	//3号景点
// 	Turn_Around_For_8();			//原地转向，8号平台特调
// 	Platform8_to_Scenic_Spot3();	//平台8至景点3
// 	Word_Test_Recognition();		//文字识别
// 	Hit_The_Scenic_Spot();			//撞景点

// 	osDelay(1000);

// 	Scenic_Spot3_to_Test_C();		//景点3至C点
// }



















// /** @brief	平台1至平台2,不走桥
//   **/
// void Platform1_to_Platform2_no_Bridge(void)
// {
// 	Car_Mode(Car_Advance);									//小车前进控制

// 	Go_Down_Platform();								//下平台

// 	// Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Left,30);			//行进中左转
// 	Mileage_Arrive_Wait(30);					//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;	//速度设定

// 	Cross_the_Short_Wave_Board();				//翻越短波浪板

// 	Motor_Control_Parm.Base_Triger_Speed = 3;	//速度设定

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	//Mileage_Arrive_Wait(10);					//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,60);				//右转90度


// 	//Car_Arc_Turn_Control(Turn_Right,70);	//行进中右转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;	//速度设定

// 	Mileage_Arrive_Wait(20);					//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;	//速度设定

// 	Cross_the_Mountain();

// 	Motor_Control_Parm.Base_Triger_Speed = 3;	//速度设定

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,30);			//行进中左转

// 	Motor_Control_Parm.Base_Triger_Speed = 3;	//速度设定

// 	Go_Up_Platform();								//上平台
// }







// /** @brief	1点直接通过
//   **/
// void Traffic_Sign_1_Pass_Direct(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,95);	//行进中右转90度

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(150);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[1] + Grayscale_Out.Grayscale_Trigger_State[2] +
// 			Grayscale_Out.Grayscale_Trigger_State[3]+ Grayscale_Out.Grayscale_Trigger_State[4]+
// 			Grayscale_Out.Grayscale_Trigger_State[5]+ Grayscale_Out.Grayscale_Trigger_State[6]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);
	
// }

// /** @brief	2点直接通过
//   **/
// void Traffic_Sign_2_Pass_Direct(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,30);				//右转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(250);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转
// }

// /** @brief	3点直接通过
//   **/
// void Traffic_Sign_3_Pass_Direct(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(200);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[13] + Grayscale_Out.Grayscale_Trigger_State[12] +
// 			Grayscale_Out.Grayscale_Trigger_State[11]+ Grayscale_Out.Grayscale_Trigger_State[10]+
// 			Grayscale_Out.Grayscale_Trigger_State[9]+ Grayscale_Out.Grayscale_Trigger_State[8]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,135);				//右转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(250);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转
// }

// /** @brief	4点直接通过
//   **/
// void Traffic_Sign_4_Pass_Direct(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(200);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[13] + Grayscale_Out.Grayscale_Trigger_State[12] +
// 			Grayscale_Out.Grayscale_Trigger_State[11]+ Grayscale_Out.Grayscale_Trigger_State[10]+
// 			Grayscale_Out.Grayscale_Trigger_State[9]+ Grayscale_Out.Grayscale_Trigger_State[8]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	
// 	Car_Arc_Turn_Control(Turn_Right,90);	//行进中左转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(140);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转
// }










// /** @brief	平台7至景点3，改
//   **/
// void Platform7_to_Scenic_Spot3_Change(void)
// {
// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.15f;
// 	///////////////////////////

// 	Servo_State.Servo_Angle[Servo2] = 0;			//舵机角度设定
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定

// 	Mileage_Arrive_Wait(50);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定

// 	//等待Y轴姿态大于-5
// 	while(!(IMU_Data.IMU_Angle_X > -5))	osDelay(1);	

// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.12f;
// 	///////////////////////////

// 	Mileage_Arrive_Wait(20);							//里程等待

// 	Cross_the_Long_Wave_Board();					//翻越长波浪板

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Left,90);			//行进中左转90

// 	Mileage_Arrive_Wait(10);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Cross_the_Mountain();							//翻越山

// 	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置

// 	Mileage_Arrive_Wait(320);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置

// 	Mileage_Arrive_Wait(120);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 			//微抬起前瞻，以便过桥洞
// 			Servo_State.Servo_Angle[Servo2] = 45;			//舵机角度设定
// 			Mileage_Arrive_Wait(80);						//里程等待，经过桥洞
// 			Servo_State.Servo_Angle[Servo2] = 35;			//降下前瞻

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Mileage_Arrive_Wait(30);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度
// 	osDelay(100);									//等待车身稳定

// 	//Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90度

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//停止
// 	Stop_Control();
// }

// /** @brief	景点3至平台8，改
//   **/
// void Scenic_Spot3_to_Platform8_Change(void)
// {
// 	Car_Mode(Car_Retreat);									//小车后退控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Angle_Control_Choice = RUN_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Mode(Car_Advance);									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = RUN_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(180);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(150);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,130);				//右转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.15f;
// 	///////////////////////////

// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
// 	Mileage_Arrive_Wait(40);						//里程等待

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//到达一阶平台

// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
// 	Mileage_Arrive_Wait(40);						//里程等待
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//到达二阶平台
// 	//清除微动开关触发状态
// 	RUN_Parm.Microswitch_State = 0;
// 	//等待微动开关触发
// 	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

// 	osDelay(100);

// 	Car_Mode(Car_Retreat);											//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = RUN_Retreat;	//后退控制模式
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;				//速度设置
// 	Mileage_Arrive_Wait(8);									//里程等待
// 	RUN_Parm.Angle_Control_Choice = RUN_Advance;	//前进控制模式

// 	//停止
// 	Stop_Control();

// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.12f;
// 	///////////////////////////
// }

// /** @brief	平台8至C点，改
//   **/
// void Platform8_to_Test_E_Change(void)
// {
// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.15f;
// 	///////////////////////////

// 	Servo_State.Servo_Angle[Servo2] = 0;			//舵机角度设定

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);
// 	Mileage_Arrive_Wait(20);						//里程等待	

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);
	
// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	//到达一阶平台

// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
// 	Mileage_Arrive_Wait(40);						//里程等待

// 	//等待Y轴姿态大于-5
// 	while(!(IMU_Data.IMU_Angle_X > -5))	osDelay(1);	
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定


// 	///////////////////////////
// 	//变换巡线PID比例系数
// 	Line_Patrol_PID.Kp = 0.12f;
// 	///////////////////////////

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[13] + Grayscale_Out.Grayscale_Trigger_State[12] +
// 			Grayscale_Out.Grayscale_Trigger_State[11]+ Grayscale_Out.Grayscale_Trigger_State[10]+
// 			Grayscale_Out.Grayscale_Trigger_State[9]+ Grayscale_Out.Grayscale_Trigger_State[8]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定
// }
// /** @brief	E点至A点至Alpha，改
//   **/
// void Test_E_to_Test_A_to_Alpha_Change(void)
// {
// 	Car_Turn_Control(Turn_Left,95);					//左转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(60);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[0] + Grayscale_Out.Grayscale_Trigger_State[1] +
// 			Grayscale_Out.Grayscale_Trigger_State[2]+ Grayscale_Out.Grayscale_Trigger_State[3]+
// 			Grayscale_Out.Grayscale_Trigger_State[4]+ Grayscale_Out.Grayscale_Trigger_State[5]+
// 			Grayscale_Out.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转
// }

// /** @brief	C点至B点至Alpha
//   **/
// void Test_E_to_Test_B_to_Alpha_Change(void)
// {
// 	Car_Turn_Control(Turn_Right,90);				//右转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,130);					//左转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(250);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(5);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,135);				//左转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,135);				//右转
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	Alpha点至平台1
//   **/
// void Aplha_to_Platform1(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	Cross_the_Short_Wave_Board();					//翻越短波浪板

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	
// 	Servo_State.Servo_Angle[Servo2] = 10;			//降下前瞻

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	Mileage_Arrive_Wait(5);						//里程等待

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(30);						//里程等待
// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

// 	Servo_State.Servo_Angle[Servo2] = 10;			//降下前瞻
// 	//停止
// 	Stop_Control();
// }

// /** @brief	C点至A点至Alpha
//   **/
// void Test_C_to_Test_A_to_Alpha(void)
// {
// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(120);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[0] + Grayscale_Out.Grayscale_Trigger_State[1] +
// 			Grayscale_Out.Grayscale_Trigger_State[2]+ Grayscale_Out.Grayscale_Trigger_State[3]+
// 			Grayscale_Out.Grayscale_Trigger_State[4]+ Grayscale_Out.Grayscale_Trigger_State[5]+
// 			Grayscale_Out.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转
// }

// /** @brief	C点至B点至Alpha
//   **/
// void Test_C_to_Test_B_to_Alpha(void)
// {
// 	Car_Arc_Turn_Control(Turn_Left,30);				//行进中左转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(250);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,135);				//左转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,135);				//右转
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	D点至A点至Alpha
//   **/
// void Test_D_to_Test_A_to_Alpha(void)
// {
// 	Car_Arc_Turn_Control(Turn_Left,40);				//行进中左转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(250);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,130);				//右转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转
// }

// /** @brief	D点至B点至Alpha
//   **/
// void Test_D_to_Test_B_to_Alpha(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(120);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[13] + Grayscale_Out.Grayscale_Trigger_State[12] +
// 			Grayscale_Out.Grayscale_Trigger_State[11]+ Grayscale_Out.Grayscale_Trigger_State[10]+
// 			Grayscale_Out.Grayscale_Trigger_State[9]+ Grayscale_Out.Grayscale_Trigger_State[8]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,135);				//右转
// 	osDelay(100);									//等待车身稳定
// }

/** @brief	里程累计函数
  **/
void Mileage_Int_Compute(void)
{
	//运行状态下
	if(RUN_Parm.RUN_State)
	{
		//累计里程
		static float Last_Mileage,Now_Mileage;
		Last_Mileage = Now_Mileage;
		Now_Mileage = Encoder1.RUN_Int + Encoder2.RUN_Int + Encoder3.RUN_Int + Encoder4.RUN_Int;

		//计算平均里程
		if(RUN_Parm.Mileage_State == 0)
			//运行状态，引入偏差修正
			RUN_Parm.Mileage_Int += ((Now_Mileage - Last_Mileage) / 4) * (Set_Mileage_100_Actual / (float)100);
		else
			//校准状态，无修正
			RUN_Parm.Mileage_Int += ((Now_Mileage - Last_Mileage) / 4);
	}
}


















