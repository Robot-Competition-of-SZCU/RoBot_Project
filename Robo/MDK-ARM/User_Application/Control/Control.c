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

#include "OLED.h"
#include "IMU.h"
#include "PID.h"
#include "Encoder.h"
#include "Servo.h"
#include "Grayscale_ADC.h"

//控制状态结构体
struct Motor_Control Motor_Control_Parm;
//运行状态结构体
struct RUN RUN_Parm;

/** @brief	小车前进设置
  **/
void Car_Advance(void)
{
	Motor_Control_Parm.M1 = Advance;
	Motor_Control_Parm.M2 = Advance;
	Motor_Control_Parm.M3 = Advance;
	Motor_Control_Parm.M4 = Advance;
}

/** @brief	小车后退设置
  **/
void Car_Retreat(void)
{
	Motor_Control_Parm.M1 = Retreat;
	Motor_Control_Parm.M2 = Retreat;
	Motor_Control_Parm.M3 = Retreat;
	Motor_Control_Parm.M4 = Retreat;
}

/** @brief	控制参数初始化
  **/
void RUN_Parm_Init(void)
{
	//小车前进设置
	Car_Advance();
	Motor_Control_Parm.Turn_Speed = 1.5;
}

/** @brief	运行速度控制
  **/
void RUN_Speed_Control(void)
{
	//运行状态下
    if(RUN_Parm.RUN_State)
    {
		//基础速度不在目标速度范围内
		float Max_Target_Speed = Motor_Control_Parm.Base_Triger_Speed * 1.05f;
		float Min_Target_Speed = Motor_Control_Parm.Base_Triger_Speed * 0.95f;
		if(Motor_Control_Parm.Base_Speed > Max_Target_Speed || Motor_Control_Parm.Base_Speed < Min_Target_Speed)
		{
			//跟随目标速度
			if(Motor_Control_Parm.Base_Speed < Motor_Control_Parm.Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed += Motor_Control_Parm.Accelerated_Speed * 0.005f;
			else if (Motor_Control_Parm.Base_Speed > Motor_Control_Parm.Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed -= Motor_Control_Parm.Accelerated_Speed * 0.005f;
		}
		//目标速度为0，基础速度同时为0
		if(Motor_Control_Parm.Base_Triger_Speed == 0)
			Motor_Control_Parm.Base_Speed = 0;
    }
    else
    {
        Motor_Control_Parm.Base_Speed = 0;
    }
	if(RUN_Parm.RUN_Control == 0)
	{
		RUN_Parm.RUN_State = 0;
		//直接控制模式
		//RUN_Parm.Control_State = Direct_Control;
	}	
}

/** @brief	IMU姿态获取
  **/
void IMU_Z_Angle_Get(void)
{
	static float Angle_Int;		//角度累计
	static int Int_Time;		//角度累计次数
	//当前为巡线控制模式,且行驶里程未达到设定值
	if(RUN_Parm.Control_State == ScanLine_Control && RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm)
	{
		//行驶里程在20以上
		if(RUN_Parm.Mileage_Int >= 20)
		{
			//累计Z轴姿态
			Angle_Int += IMU_Data.IMU_Angle_Z;
			Int_Time++;
		}
	}
	//若行驶里程达到设定值，且存在角度累计
	else if(RUN_Parm.Mileage_Int >= RUN_Parm.Mileage_Parm && Int_Time )
	{
		//计算角度均值，并设定为目标
		RUN_Parm.Target_Angle = Angle_Int / Int_Time;
		//复位角度累计
		Angle_Int = 0;
		Int_Time = 0;
	}
	else 
	{
		//复位角度累计
		Angle_Int = 0;
		Int_Time = 0;
	}
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
  * @note	基于IMU角度增量累计，自动处理-180~+180角度回绕
  **/
void Car_Turn_Control(Turn_Mode Mode,float Turn_Angle)
{
	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//设置电机转向
	if(Mode == Turn_Left)
	{
		Motor_Control_Parm.M1 = Advance;
		Motor_Control_Parm.M2 = Advance;
		Motor_Control_Parm.M3 = Retreat;
		Motor_Control_Parm.M4 = Retreat;
	}
	else if(Mode == Turn_Right)
	{
		Motor_Control_Parm.M1 = Retreat;
		Motor_Control_Parm.M2 = Retreat;
		Motor_Control_Parm.M3 = Advance;
		Motor_Control_Parm.M4 = Advance;
	}
	else
		return;

	//执行旋转
	Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Turn_Speed;
	Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Turn_Speed;
	Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Turn_Speed;
	Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Turn_Speed;

	//累计转向角度，处理IMU角度回绕
	float Turned_Angle = 0;						//已转角度
	float Last_Angle = IMU_Data.IMU_Angle_Z;	//上一次角度
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
		osDelay(1);
	}
	//停止旋转
	Motor_Control_Parm.Motor1_Speed = 0;
	Motor_Control_Parm.Motor2_Speed = 0;
	Motor_Control_Parm.Motor3_Speed = 0;
	Motor_Control_Parm.Motor4_Speed = 0;

	//调整为直行
	Car_Advance();
}

/** @brief	停止控制
  **/
void Stop_Control(void)
{
	//关闭运行基速
	Motor_Control_Parm.Base_Triger_Speed = 0;
	Motor_Control_Parm.Base_Speed = 0;

	//关闭电机输出
	Motor_Control_One(M1,Motor_Control_Parm.M1,0);
	Motor_Control_One(M2,Motor_Control_Parm.M2,0);
	Motor_Control_One(M3,Motor_Control_Parm.M3,0);
	Motor_Control_One(M4,Motor_Control_Parm.M4,0);
}

/** @brief	等待到达设定里程
  * @param	Mileage: 里程设定，单位cm
  **/
void Mileage_Arrive_Wait(float Mileage)
{
	RUN_Parm.Mileage_Int	= 0;			//复位里程计
	RUN_Parm.Mileage_Parm	= Mileage;		//里程设定
	//等待达到设定里程
	while(RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm) osDelay(1);	
}

/** @brief	系统运行控制任务
  **/
void RUN_System_Control(void)
{
	//运行状态下
    if(RUN_Parm.RUN_State)
    {
		Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设定
		RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		Mileage_Arrive_Wait(100);						//里程等待

		//清除左右侧光电传感器触发状态
		RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
		//等待左右侧光电传感器触发
		while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	
		
		Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定
		Servo_State.Servo_Angle[Servo2] = 80;			//舵机角度设定
		RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
		RUN_Parm.Target_Angle	= 0;					//设定角度为0度
		Mileage_Arrive_Wait(30);						//里程等待
		Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

		//清除左右侧光电传感器触发状态
		RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
		//等待达到设定里程
		RUN_Parm.Mileage_Int	= 0;					//复位里程计
		RUN_Parm.Mileage_Parm	= 120;					//里程设定
		while(RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm) 
		{
			//左侧传感器被触发，且当前端口为低电平
			if(RUN_Parm.Sensor_Left && HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6) == 0)
			{	//角度偏移
				RUN_Parm.Target_Angle -= 10;
				RUN_Parm.Sensor_Left = 0;
			}
			//右侧传感器被触发，且当前端口为低电平
			if(RUN_Parm.Sensor_Right && HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6) == 0)
			{	//角度偏移
				RUN_Parm.Target_Angle += 10;
				RUN_Parm.Sensor_Right = 0;
			}
			osDelay(1);
		}	
		//等待灰度触发数量大于0
		while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	

		RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		Mileage_Arrive_Wait(100);						//里程等待

		//清除左右侧光电传感器触发状态
		RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
		//等待左右侧光电传感器触发
		while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

		RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

		//等待灰度触发数量为0
		while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

		//到达2号平台///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
		RUN_Parm.Control_State	= Angle_Control;		//巡线控制
		Mileage_Arrive_Wait(60);						//里程等待

		Car_Retreat();									//小车后退控制
		Mileage_Arrive_Wait(8);							//里程等待

		//停止
		Stop_Control();

		Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定
		osDelay(200);									//等待200ms

		RUN_Parm.Control_State	= Turn_Control;			//转弯控制
		Car_Turn_Control(Turn_Left,175);				//左转180度

		Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
		osDelay(200);									//等待200ms

		RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		Mileage_Arrive_Wait(50);						//里程等待
		//等待到达路口
		while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);

		RUN_Parm.Control_State	= Turn_Control;			//转弯控制
		Car_Turn_Control(Turn_Right,40);				//右转
		osDelay(100);	

		RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		Mileage_Arrive_Wait(40);						//里程等待	
		//清除左右侧光电传感器触发状态
		RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
		//等待左右侧光电传感器触发
		while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

		Servo_State.Servo_Angle[Servo2] = 60;			//舵机角度设定

		Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
		RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
		Mileage_Arrive_Wait(30);						//里程等待
		Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
		Mileage_Arrive_Wait(30);						//里程等待
		
		RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		Motor_Control_Parm.Base_Triger_Speed = 2.5;
		//等待到达路口
		while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);



		//停止
		RUN_Parm.RUN_Control = 0;
		while(1) osDelay(1);	

	

	}
}

/** @brief	里程累计函数
  **/
void Mileage_Int_Compute(void)
{
	//运行状态下
	if(RUN_Parm.RUN_State)
	{
		//累计里程
		float Mileage = Encoder1.RUN_Int + Encoder2.RUN_Int + Encoder3.RUN_Int + Encoder4.RUN_Int;
		//复位里程
		Encoder1.RUN_Int = 0;
		Encoder2.RUN_Int = 0;
		Encoder3.RUN_Int = 0;
		Encoder4.RUN_Int = 0;
		//计算平均里程
		RUN_Parm.Mileage_Int += Mileage / 4;
	}
}

/** @brief	GPIO中断触发任务
  * @param	GPIO_Pin: 触发引脚
  **/
void GPIO_Trigger_Control(uint32_t GPIO_Pin)
{
	//PF7被触发
	if(GPIO_Pin == GPIO_PIN_7) 
	{
		//当前为允许运行状态
		if(RUN_Parm.RUN_Control == 1)
		{
			//开始运行
			RUN_Parm.RUN_State = 1;	
		}
		//运行状态下
		if(RUN_Parm.RUN_State == 1)
		{
			//达到设定里程
			if(RUN_Parm.Mileage_Int >= RUN_Parm.Mileage_Parm)
			{
				//下一目标为平台
				if(RUN_Parm.Front == Platform)
				{
					//置执行切换标志位
					RUN_Parm.Path_Switching_Flag = 1;
				}
			}
		}
			
	}
	//PF6被触发
	if(GPIO_Pin == GPIO_PIN_6)
		RUN_Parm.Sensor_Left++;
	//PF5被触发
	if(GPIO_Pin == GPIO_PIN_5)
		RUN_Parm.Sensor_Right++;

	// //PF6，PF5被触发
	// if(GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_5)
	// {
	// 	//达到设定里程
	// 	if(RUN_Parm.Mileage_Int >= RUN_Parm.Mileage_Parm)
	// 	{
	// 		//下一目标为障碍
	// 		if(RUN_Parm.Front == RUN_Advance)
	// 		{
	// 			//
	// 		}
	// 		//下一目标为盲区
	// 		if(RUN_Parm.Front == Dead_Zone)
	// 		{
	// 			//置执行切换标志位
	// 			RUN_Parm.Path_Switching_Flag = 1;
	// 		}
	// 	}
	// }
}

