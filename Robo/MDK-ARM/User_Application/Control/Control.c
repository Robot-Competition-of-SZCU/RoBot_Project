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
#include "LED.h"
#include "IMU.h"
#include "PID.h"
#include "Encoder.h"
#include "Servo.h"
#include "Grayscale_ADC.h"
#include "UART.h"

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
	//转向速度设置
	Motor_Control_Parm.Turn_Speed = 1.5f;
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
			if(RUN_Parm.Angle_Control_Choice == Angle_Control_Advance)
				Motor_Control_Parm.Base_Speed += Advance_Accelerated_Speed * DT/1000;	//进行加速
			//后退模式
			else
				Motor_Control_Parm.Base_Speed += Retreat_Accelerated_Speed * DT/1000;	//进行加速
			//钳位，防止过冲
			if(Motor_Control_Parm.Base_Speed > Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed = Base_Triger_Speed;
		}
		//当前速度大于目标速度，减速
		else if(Motor_Control_Parm.Base_Speed > Base_Triger_Speed)
		{
			//前进模式
			if(RUN_Parm.Angle_Control_Choice == Angle_Control_Advance)
				Motor_Control_Parm.Base_Speed -= Advance_Deceleration_Speed * DT/1000;	//进行减速
			else
				Motor_Control_Parm.Base_Speed -= Retreat_Deceleration_Speed * DT/1000;	//进行减速
			//钳位，防止过冲
			if(Motor_Control_Parm.Base_Speed < Base_Triger_Speed)
				Motor_Control_Parm.Base_Speed = Base_Triger_Speed;
		}
    }
	//非运行状态
    else
    {
		//基础速度始终为0
        Motor_Control_Parm.Base_Speed = 0;
    }
	if(RUN_Parm.RUN_Control == 0)
	{
		RUN_Parm.RUN_State = 0;
		//直接控制模式
		//RUN_Parm.Control_State = Direct_Control;
	}	
}

/** @brief	巡线运行控制
  * @param  RUN_Speed	运行速度
  * @param  Stop_Speed	到达设定里程后速度
  * @param  Mileage		行进里程
  * @param  End_Mode	终点减速模式
  **/
void Scan_Line_RUN_Control(float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode)
{
	Car_Advance();											//直行模式

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
									  / (2*Advance_Deceleration_Speed)
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
  * @param  Stop_Speed	到达设定里程后速度
  * @param  Mileage		行进里程
  * @param  End_Mode	终点减速模式
  * @note	该函数在调用前需设定所跟随的角度
  **/
void Angle_Following_RUN_Control(Angle_Control_Mode Mode,float RUN_Speed,float Stop_Speed,float Mileage,RUN_END_Deceleration_Mode End_Mode)
{
	if(Mode == Angle_Control_Advance)
	{
		Car_Advance();		//前进模式
		RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	}
	else
	{
		Car_Retreat();		//后退模式
		RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
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
									  / (2*Advance_Deceleration_Speed)
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
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
	Angle_Following_RUN_Control(Angle_Control_Advance,3,0,100,End_Finish_Deceleration);	//进行移动
	Motor_Control_Parm.Base_Speed = 0;											//达到设定后直接停止
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

	Car_Stop();									//停止

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 60;

	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//转向速度保护，速度为0时无法转向，直接退出防止死循环
	if(Turn_Speed <= 0) return;

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
		float Decel_Angle = Turn_Angle_Speed_Coefficient
							* Now_Turn_Speed * Now_Turn_Speed
							/ (2 * Turn_Deceleration_Speed);
		//目标速度：剩余角度不足以减速到0时为0，否则为设定转向速度
		float Target_Speed = (Remain_Angle <= Decel_Angle) ? 0 : Turn_Speed;

		//速度斜坡调节
		if(Now_Turn_Speed < Target_Speed)
		{
			Now_Turn_Speed += Turn_Accelerated_Speed * 0.001f;	//进行加速，调节周期1ms
			if(Now_Turn_Speed > Target_Speed) Now_Turn_Speed = Target_Speed;	//钳位，防止过冲
		}
		else if(Now_Turn_Speed > Target_Speed)
		{
			Now_Turn_Speed -= Turn_Deceleration_Speed * 0.001f;	//进行减速，调节周期1ms
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

	//调整为直行
	Car_Advance();
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

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 45;

	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//记录转弯完成后的恢复速度
	RUN_Parm.Arc_Turn_Resume_Speed = Motor_Control_Parm.Base_Triger_Speed;
	//设置转弯基础速度
	Motor_Control_Parm.Base_Triger_Speed = Motor_Control_Parm.Turn_Speed;

	//计算目标角度，处理回绕
	RUN_Parm.Arc_Turn_Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
	RUN_Parm.Arc_Turn_Target_Angle += (Mode == Turn_Left) ? Turn_Angle : -Turn_Angle;
	while(RUN_Parm.Arc_Turn_Target_Angle > 180) RUN_Parm.Arc_Turn_Target_Angle -= 360;
	while(RUN_Parm.Arc_Turn_Target_Angle < -180) RUN_Parm.Arc_Turn_Target_Angle += 360;

	//复位弧线转弯PID历史误差
	Arc_Turn_PID.Error_Now = 0;
	Arc_Turn_PID.Error_Last = 0;
	Arc_Turn_PID.ErrorInt = 0;
	Arc_Turn_PID.Out = 0;

	//进入弧线转弯状态
	RUN_Parm.Control_State = Arc_Turn_Control;

	//阻塞等待转弯完成，转弯状态由周期任务驱动，完成后自动切回巡线
	while(RUN_Parm.Control_State == Arc_Turn_Control) osDelay(1);

	//降下前瞻
	Servo_State.Servo_Angle[Servo2] = 35;

	Car_Stop();	//停止
}

/** @brief	停止控制
  **/
void Car_Stop(void)
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
	RUN_Parm.Visual_Identity_Word_Flag = 1;

	for(int i=0;i<5000;i++)
	{
		//等待
		if(RUN_Parm.Visual_Identity_Word_Flag == 0)
		{
			switch(RUN_Parm.Word_Test)
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
			RUN_Parm.Word_Test_Num++;
			//复位检测文字
			RUN_Parm.Word_Test = 0;
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
	RUN_Parm.Visual_Identity_Colour_Flag = 1;

	for(int i=0;i<5000;i++)
	{
		//等待
		if(RUN_Parm.Visual_Identity_Colour_Flag == 0)
		{
			//检测到绿色
			if(RUN_Parm.Colour_Test == 'G')
				return 1;
			break;	
		}
		osDelay(1);
	}
	//5秒等待超时退出
	return 0;
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

/** @brief	系统运行控制任务
  **/
void RUN_System_Control(void)
{
	//运行状态下
    if(RUN_Parm.RUN_Control)
    {
		RUN_Parm.RUN_State = 1;

		// Car_Turn_Control(Turn_Right,180,5);

		// //跟随右侧线
		// Grayscale_Ctrl.Target_Following = Right_Line_Follows;

		// //进行移动,加速至10，行进180，在终点完成减速至3
		// Scan_Line_RUN_Control(3,0,500,End_Finish_Deceleration);	//进行移动


		// osDelay(100000000);


		//进行移动,加速至10，行进180，在终点完成减速至3
		Scan_Line_RUN_Control(10,3,180,End_Finish_Deceleration);	//进行移动

		//等待到达左锐角路口
		while(!(Grayscale_Out.Now_Intersection == Left_Acute_Angle_Intersection)) osDelay(1);

		//左转30度
		Car_Arc_Turn_Control(Turn_Left,30,1.5);

		//跟随左侧线
		Grayscale_Ctrl.Target_Following = Left_Line_Follows;

		Scan_Line_RUN_Control(5,0,100,End_Start_Deceleration);	//进行移动

		//获取当前角度
 		RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

		//等待速度归零
		while(!(Motor_Control_Parm.Base_Speed == 0)) osDelay(1);

		Angle_Following_RUN_Control(Angle_Control_Retreat,5,3,80,End_Finish_Deceleration);	//后退

		//等待存在两段线或触发数量>=4
		while(!(Grayscale_Out.Number_Of_Discontinuous_Points >=2 || Grayscale_Out.Grayscale_Trigger_Num >=4)) osDelay(1);

		Angle_Following_RUN_Control(Angle_Control_Retreat,3,0,10,End_Finish_Deceleration);	
		osDelay(100);
		Scan_Line_RUN_Control(2.5,2.5,5,End_Start_Deceleration);	//进行移动

		//右转30度
		Car_Arc_Turn_Control(Turn_Right,30,1.5);

		//跟随右侧线
		Grayscale_Ctrl.Target_Following = Middle_Line_Follows;

		Scan_Line_RUN_Control(10,3,150,End_Finish_Deceleration);	//进行移动

		//等待到达右直角路口
		while(!(Grayscale_Out.Now_Intersection == Right_Right_angle_Intersection)) osDelay(1);
		Car_Arc_Turn_Control(Turn_Right,80,1.5);
		Scan_Line_RUN_Control(10,3,60,End_Finish_Deceleration);		//进行移动

		//等待到达右锐角路口
		while(!(Grayscale_Out.Now_Intersection == Right_Acute_Angle_Intersection)) osDelay(1);

		Scan_Line_RUN_Control(3,0,20,End_Finish_Deceleration);		//进行移动

		Car_Turn_Control(Turn_Right,140,3);

		Scan_Line_RUN_Control(10,3,100,End_Finish_Deceleration);		//进行移动

		Grayscale_Out.Now_Intersection = No_Crossroad;

		//等待到达右锐角路口
		while(!(Grayscale_Out.Now_Intersection == Right_Acute_Angle_Intersection)) osDelay(1);

		//右转30度
		Car_Arc_Turn_Control(Turn_Left,20,1.5);

		Scan_Line_RUN_Control(10,0,300,End_Finish_Deceleration);		//进行移动

		osDelay(100000000);

		// //举手
		// Servo_State.Servo_Angle[Servo5] = 180;
		// Servo_State.Servo_Angle[Servo6] = 180;
		// UART_Audio_Control(0);			//播报准备完毕
		// osDelay(1000);
		// //放手
		// Servo_State.Servo_Angle[Servo5] = 0;
		// Servo_State.Servo_Angle[Servo6] = 0;
		
		//  	if(RUN_Parm.Round == 0)						//第一回合
		//  		UART_Visual_Identity_Send_Order('2');	//开启视觉识别

		// //2号台
		// Platform1_to_Platform2_on_Bridge();		//平台1至平台2
		// UART_Audio_Control(2);			//播报到达2号平台
		// //osDelay(2000);

		// 	if(RUN_Parm.Round == 0)						//第一回合
		// 		UART_Visual_Identity_Send_Order('2');	//进行文字识别

		// //2号景点
		// Turn_Around();					//原地转向
		// Platform2_to_Scenic_Spot2();	//平台2至景点2
		// Word_Test_Recognition();		//文字识别
		// Hit_The_Scenic_Spot();			//撞景点
		// osDelay(1000);

		// //4号平台
		// Scenic_Spot2_to_Platform4();	//景点2至平台4
		// UART_Audio_Control(4);			//播报到达4号平台
		// //osDelay(2000);

		// //1号景点
		// Turn_Around();					//原地转向
		// Platform4_to_Scenic_Spot1();	//平台4至景点1
		// Word_Test_Recognition();		//文字识别
		// Hit_The_Scenic_Spot();			//撞景点
		// osDelay(1000);

		// 	if(RUN_Parm.Round == 0)						//第一回合
		// 		UART_Visual_Identity_Send_Order('9');	//退出文字识别

		// //3号平台
		// Scenic_Spot1_to_Platform3();	//景点1至平台3
		// UART_Audio_Control(3);			//播报到达3号平台
		// //osDelay(2000);

		// 	if(RUN_Parm.Round == 0)						//第一回合
		//  		UART_Visual_Identity_Send_Order('1');	//进入颜色识别

		//  Turn_Around();					//原地转向

		//  	if(RUN_Parm.Round == 0)						//第一回合
		//  		UART_Visual_Identity_Send_Order('7');	//保存白平衡

		// Platform3_to_Test_A();			//平台3至A点
		
		// //存在通行记录
		// switch(RUN_Parm.Common_Traffic_Sign)
		// {
		// 	case 1: Traffic_Sign_1_Pass_Direct();		//通行指示牌1直接通过
		// 			goto Rounter2;						//前往阶段2
		// 	case 2:	Traffic_Sign_2_Pass_Direct();		//通行指示牌2直接通过
		// 			goto Rounter2;						//前往阶段2
		// 	case 3: Traffic_Sign_2_Pass_Direct();		//通行指示牌3直接通过
		// 			goto Rounter2;						//前往阶段2
		// 	case 4: Traffic_Sign_2_Pass_Direct();		//通行指示牌4直接通过
		// 			goto Rounter2;						//前往阶段2
		// }

		// //无记录
		// Traffic_Sign_1_Test();			//通行指示牌1检测

		// //进行A点检测
		// if(Colour_Test_Recognition() == 1)
		// {	
		// 			UART_Visual_Identity_Send_Order('9');	//退出颜色识别

		// 	Traffic_Sign_1_Pass();				//通行指示牌1通过
		// 	RUN_Parm.Common_Traffic_Sign = 1;	//1号通道可通过
		// 	goto Rounter2;						//前往阶段2
		// }
		// else
		// 	Traffic_Sign_2_Test();			//通行指示牌2检测

		// if(Colour_Test_Recognition() == 1)
		// {	
		// 			UART_Visual_Identity_Send_Order('9');	//退出颜色识别

		// 	Traffic_Sign_2_Pass();				//通行指示牌2通过
		// 	RUN_Parm.Common_Traffic_Sign = 2;	//2号通道可通过
		// 	goto Rounter2;						//前往阶段2
		// }
		// else
		// 	Traffic_Sign_3_Test();			//通行指示牌3检测

		// if(Colour_Test_Recognition() == 1)
		// {	
		// 			UART_Visual_Identity_Send_Order('9');	//退出颜色识别

		// 	Traffic_Sign_3_Pass();				//通行指示牌3通过
		// 	RUN_Parm.Common_Traffic_Sign = 3;	//3号通道可通过
		// 	goto Rounter2;						//前往阶段2
		// }
		// else
		// 	Traffic_Sign_4_Test();
		// 			UART_Visual_Identity_Send_Order('9');	//退出颜色识别

		// 	Traffic_Sign_4_Pass();			//通行指示牌4通过
		// RUN_Parm.Common_Traffic_Sign = 4;	//4号通道可通过

		// //阶段2
		// Rounter2:

		// 		UART_Visual_Identity_Send_Order('2');	//进行文字识别

		// //进入后段路程
		// switch(RUN_Parm.Common_Traffic_Sign)
		// {
		// 	//可通过路口为1
		// 	case 1:	
		// 	case 3:
		// 			Test_C_to_Test_D();				//C点至D点

		// 			Car_Arc_Turn_Control(Turn_Right,95);	//行进中右转90

		// 			Test_D_to_Test_E();				//D点至C点
		// 			break;
		// 	case 2:
		// 	case 4:
		// 			Test_D_to_Test_E();				//D点至C点

		// 			Car_Turn_Control(Turn_Right,85);	//右转90
		// 			RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
		// 			Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
		// 			Mileage_Arrive_Wait(30);						//里程等待

		// 			Test_C_to_Test_D();				//C点至D点
		// 			break;
		// }

		// //回家
		// switch(RUN_Parm.Common_Traffic_Sign)
		// {
		// 	case 1:	Test_E_to_Test_A_to_Alpha_Change();		//C点至A点至Alpha
		// 			break;
		// 	case 2:	Test_D_to_Test_A_to_Alpha();		//C点至B点至Alpha
		// 			break;
		// 	case 3: Test_E_to_Test_B_to_Alpha_Change();		//D点至A点至Alpha
		// 			break;
		// 	case 4:	Test_D_to_Test_B_to_Alpha();		//D点至B点至Alpha
		// 			break;
		// }
		// Aplha_to_Platform1();				//Alpha点至平台1
		// Turn_Around();						//原地转向

		// UART_Audio_Control(1);				//播报到达1号平台
		// //osDelay(2000);

		// //停止
		// //RUN_Parm.RUN_Control = 0;
		// RUN_Parm.RUN_State = 0;
		// RUN_Parm.Round++;					//回合累计
		// //while(1) osDelay(1);	
	}
}


// /** @brief	上平台
//   **/
// void Go_Up_Platform(void)
// {	
// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	Mileage_Arrive_Wait(5);						//里程等待

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(10);						//里程等待
// 	//清除微动开关触发状态
// 	RUN_Parm.Microswitch_State = 0;
// 	//等待微动开关触发
// 	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

// 	osDelay(100);

// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	Mileage_Arrive_Wait(8);							//里程等待
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

// 	//停止
// 	Stop_Control();
// }

// /** @brief	撞景点
//   **/
// void Hit_The_Scenic_Spot(void)
// {
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
	
// 	//清除微动开关触发状态
// 	RUN_Parm.Microswitch_State = 0;
// 	//等待微动开关触发
// 	while(!RUN_Parm.Microswitch_State) osDelay(1);

// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	Mileage_Arrive_Wait(8);						//里程等待
// 	//停止
// 	Stop_Control();

// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	Car_Advance();									//小车前进控制
// }

// /** @brief	下平台
//   **/
// void Go_Down_Platform(void)
// {
// 	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
// 	Mileage_Arrive_Wait(10);							//里程等待
// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// }

// /** @brief	原地转向
//   **/
// void Turn_Around(void)
// {
// 	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

// 	//举手
// 	Servo_State.Servo_Angle[Servo5] = 180;
// 	Servo_State.Servo_Angle[Servo6] = 180;
// 	osDelay(200);									//等待200ms

// 	Car_Turn_Control(Turn_Left,172.5);				//左转180度

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	//放手
// 	Servo_State.Servo_Angle[Servo5] = 0;
// 	Servo_State.Servo_Angle[Servo6] = 0;
// 	osDelay(200);									//等待200ms
// }

// /** @brief	原地转向
//   **/
// void Turn_Around_For_8(void)
// {
// 	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

// 	//举手
// 	Servo_State.Servo_Angle[Servo5] = 180;
// 	Servo_State.Servo_Angle[Servo6] = 180;
// 	osDelay(200);									//等待200ms

// 	Car_Turn_Control(Turn_Right,162);				//左转180度

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	//放手
// 	Servo_State.Servo_Angle[Servo5] = 0;
// 	Servo_State.Servo_Angle[Servo6] = 0;
// 	osDelay(200);									//等待200ms
// }

// /** @brief	翻越山
//   **/
// void Cross_the_Mountain(void)
// {
// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定

// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制

// 	Mileage_Arrive_Wait(30);						//里程等待
// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// }

// /** @brief	翻越长波浪板
//   **/
// void Cross_the_Long_Wave_Board(void)
// {
// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	Servo_State.Servo_Angle[Servo2] = 60;			//舵机角度设定

// 	Motor_Control_Parm.Base_Triger_Speed = 1;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
// 	Mileage_Arrive_Wait(120);						//里程等待
// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// }

// /** @brief	翻越短波浪板
//   **/
// void Cross_the_Short_Wave_Board(void)
// {
// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	Servo_State.Servo_Angle[Servo2] = 60;			//舵机角度设定

// 	Motor_Control_Parm.Base_Triger_Speed = 1;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
// 	Mileage_Arrive_Wait(60);						//里程等待
// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// }

// /** @brief	景点1至平台3
//   **/
// void Scenic_Spot1_to_Platform3(void)
// {
// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	// //等待到达路口中心
// 	// while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1);

// 	Stop_Control();									//停止
// 	osDelay(150);

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	// Mileage_Arrive_Wait(10);							//里程等待

// 	// Stop_Control();									//停止
// 	//osDelay(100);


// 	Car_Arc_Turn_Control(Turn_Left,15);				//行进中左转
	
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(40);							//里程等待

// 	Go_Up_Platform();								//上平台
// }

// /** @brief	平台4至景点1
//   **/
// void Platform4_to_Scenic_Spot1(void)
// {
// 	Go_Down_Platform();								//下平台
// 	Mileage_Arrive_Wait(50);						//里程等待

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
// 	Mileage_Arrive_Wait(400);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[0] + Grayscale_Out.Grayscale_Trigger_State[1] +
// 			Grayscale_Out.Grayscale_Trigger_State[2]+ Grayscale_Out.Grayscale_Trigger_State[3]+
// 			Grayscale_Out.Grayscale_Trigger_State[4]+ Grayscale_Out.Grayscale_Trigger_State[5]+
// 			Grayscale_Out.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Right,15);			//行进中右转

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待

// 	//复位中部传感器触发状态
// 	RUN_Parm.Sensor_Middle = 0;
// 	//等待中部传感器触发
// 	while(!RUN_Parm.Sensor_Middle) osDelay(1);

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待

// 	Stop_Control();									//停止
// }

// /** @brief	景点2至平台4
//   **/
// void Scenic_Spot2_to_Platform4(void)
// {
// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(30);						//里程等待
	
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1);

// 	Stop_Control();									//停止
// 	osDelay(150);

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	Car_Arc_Turn_Control(Turn_Right,30);			//行进中右转

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(40);						//里程等待

// 	Go_Up_Platform();								//上平台
// }

// /** @brief	平台2至景点2
//   **/
// void Platform2_to_Scenic_Spot2(void)
// {
// 	Go_Down_Platform();								//下平台
// 	Mileage_Arrive_Wait(30);						//里程等待
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Right,30);			//行进中右转
	
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(30);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	
// 	Cross_the_Mountain();							//翻越山
	
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Mileage_Arrive_Wait(20);						//里程等待
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	osDelay(100);

// 	Car_Arc_Turn_Control(Turn_Left,5);			//行进中左转

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
// 	Mileage_Arrive_Wait(180);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	osDelay(150);

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Left,30);				//行进中左转

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//复位中部传感器触发状态
// 	RUN_Parm.Sensor_Middle = 0;
// 	//等待中部传感器触发
// 	while(!RUN_Parm.Sensor_Middle) osDelay(1);

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待

// 	Stop_Control();									//停止
// }

// /** @brief	平台1至平台2,走桥
//   **/
// void Platform1_to_Platform2_on_Bridge(void)
// {
// 	Car_Advance();									//小车前进控制

// 	Go_Down_Platform();								//下平台

// 	// Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
// 	Mileage_Arrive_Wait(10);						//里程等待
// 	// Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
// 	// Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定

// 	Mileage_Arrive_Wait(60);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

// 	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z + 4;	//往右修正
	
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
// 	Servo_State.Servo_Angle[Servo2] = 80;			//舵机角度设定
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	Mileage_Arrive_Wait(30);						//里程等待
// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待达到设定里程
// 	RUN_Parm.Mileage_Int	= 0;					//复位里程计
// 	RUN_Parm.Mileage_Parm	= 120;					//里程设定
// 	PD4_LED_Control(LED_ON);						//开启PD4LED
// 	while(RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm) 
// 	{
// 		//左侧传感器端口为低电平
// 		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6) == 0)
// 		{	//角度偏移
// 			RUN_Parm.Target_Angle -= 0.5f;
// 		}
// 		//右侧传感器端口为低电平
// 		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5) == 0)
// 		{	//角度偏移
// 			RUN_Parm.Target_Angle += 0.5f;
// 		}
// 		osDelay(100);
// 	}	
// 	PD4_LED_Control(LED_OFF);				//关闭PD4LED
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
// 	//等待灰度触发数量大于0
// 	while(!Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

// 	Go_Up_Platform();								//上平台
// }

// /** @brief	平台1至平台2,不走桥
//   **/
// void Platform1_to_Platform2_no_Bridge(void)
// {
// 	Car_Advance();									//小车前进控制

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

// /** @brief	平台3至A点
//   **/
// void Platform3_to_Test_A(void)
// {
// 	Go_Down_Platform();								//下平台
// 	Mileage_Arrive_Wait(50);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(150);						//里程等待

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[8] + Grayscale_Out.Grayscale_Trigger_State[9] +
// 			Grayscale_Out.Grayscale_Trigger_State[10]+ Grayscale_Out.Grayscale_Trigger_State[11]+
// 			Grayscale_Out.Grayscale_Trigger_State[12]+ Grayscale_Out.Grayscale_Trigger_State[13]+
// 			Grayscale_Out.Grayscale_Trigger_State[14]) > 5)) osDelay(1);

// 	// Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌1检测
//   **/
// void Traffic_Sign_1_Test(void)
// {
// 	Car_Arc_Turn_Control(Turn_Right,95);	//行进中右转90度

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(200);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);					//右转90度
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌1通过
//   **/
// void Traffic_Sign_1_Pass(void)
// {
// 	Car_Turn_Control(Turn_Left,85);						//左转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[1] + Grayscale_Out.Grayscale_Trigger_State[2] +
// 			Grayscale_Out.Grayscale_Trigger_State[3]+ Grayscale_Out.Grayscale_Trigger_State[4]+
// 			Grayscale_Out.Grayscale_Trigger_State[5]+ Grayscale_Out.Grayscale_Trigger_State[6]+
// 			Grayscale_Out.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	//Stop_Control();								//停止
// 	//osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌2检测
//   **/
// void Traffic_Sign_2_Test(void)
// {
// 	Car_Turn_Control(Turn_Right,80);				//右转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(!((Grayscale_Out.Grayscale_Trigger_State[0] + Grayscale_Out.Grayscale_Trigger_State[1] +
// 			Grayscale_Out.Grayscale_Trigger_State[2]+ Grayscale_Out.Grayscale_Trigger_State[3]+
// 			Grayscale_Out.Grayscale_Trigger_State[4]+ Grayscale_Out.Grayscale_Trigger_State[5]+
// 			Grayscale_Out.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待
// 	Stop_Control();									//停止
// 	osDelay(400);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,128);				//右转
// 	osDelay(200);

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌2通过
//   **/
// void Traffic_Sign_2_Pass(void)
// {
// 	Car_Turn_Control(Turn_Left,85);					//左转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(150);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转

// 	// Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	//Stop_Control();								//停止
// 	//osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌3检测
//   **/
// void Traffic_Sign_3_Test(void)
// {
// 	Car_Turn_Control(Turn_Right,80);				//右转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Stop_Control();									//停止
// 	osDelay(200);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,120);				//右转
// 	osDelay(100);									//等待车身稳定

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
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,135);				//右转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(150);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌3通过
//   **/
// void Traffic_Sign_3_Pass(void)
// {
// 	Car_Turn_Control(Turn_Left,85);					//左转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(150);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转
// }

// /** @brief	通行指示牌4检测
//   **/
// void Traffic_Sign_4_Test(void)
// {
// 	Car_Turn_Control(Turn_Right,80);				//右转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(200);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,110);				//右转
// 	osDelay(100);

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度
// 	osDelay(100);									//等待车身稳定
// }

// /** @brief	通行指示牌4通过
//   **/
// void Traffic_Sign_4_Pass(void)
// {
// 	Car_Turn_Control(Turn_Left,85);					//右转90度
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 

// 	Car_Arc_Turn_Control(Turn_Left,100);				//行进中左转
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

// /** @brief	C点至景点4
//   **/
// void Test_C_to_Scenic_Spot4(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度
// 	osDelay(100);									//等待车身稳定

// 	//Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90度

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//停止
// 	Stop_Control();
// }

// /** @brief	景点4至景点5
//   **/
// void Scenic_Spot4_to_Scenic_Spot5(void)
// {
// 	Car_Retreat();									//小车后退控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90


// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(200);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,85);					//左转90度
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

// /** @brief	景点5至D点
//   **/
// void Scenic_Spot5_to_Test_D(void)
// {
// 	Car_Retreat();									//小车后退控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	// osDelay(100);									//等待车身稳定
// }

// /** @brief	D点至平台5
//   **/
// void Test_D_to_Platform5(void)
// {
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(160);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	Go_Up_Platform();								///上平台
// }

// /** @brief	平台5至平台7
//   **/
// void Platform5_to_Platform7(void)
// {
// 	Go_Down_Platform();								//下平台
// 	Mileage_Arrive_Wait(20);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
// 	Mileage_Arrive_Wait(50);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Left,40);				//行进中左转45

// 	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(150);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,135);				//左转
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(80);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	Cross_the_Mountain();							//翻越第一座山
// 	Mileage_Arrive_Wait(30);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 		//微抬起前瞻，以便过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 45;			//舵机角度设定
// 		Mileage_Arrive_Wait(60);						//里程等待，经过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 35;			//降下前瞻

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	Cross_the_Mountain();							//翻越第二座山
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
// 	Mileage_Arrive_Wait(50);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	Cross_the_Long_Wave_Board();					//翻越长波浪板
// 	//Mileage_Arrive_Wait(10);						//里程等待

// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	
// 	////////////
// 	//上高台
// 	////////////

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	Mileage_Arrive_Wait(50);						//里程等待

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//等待灰度触发数量为0
// 	while(Grayscale_Out.Grayscale_Trigger_Num) osDelay(1);	

// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
// 	//Mileage_Arrive_Wait(10);						//里程等待
// 	//清除微动开关触发状态
// 	RUN_Parm.Microswitch_State = 0;
// 	//等待微动开关触发
// 	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

// 	osDelay(100);

// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	Mileage_Arrive_Wait(8);							//里程等待
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

// 	//停止
// 	Stop_Control();
// }

// /** @brief	平台7至平台8
//   **/
// void Platform7_to_Platform8(void)
// {
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

// 	Mileage_Arrive_Wait(20);							//里程等待

// 	Cross_the_Long_Wave_Board();					//翻越长波浪板

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(40);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	Cross_the_Mountain();							//翻越山
// 	Mileage_Arrive_Wait(30);						//里程等待

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
// 	Mileage_Arrive_Wait(80);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//清除左右侧光电传感器触发状态
// 	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
// 	//等待左右侧光电传感器触发
// 	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

// 	Mileage_Arrive_Wait(50);						//里程等待

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
// 	Mileage_Arrive_Wait(50);						//里程等待
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

// 	Car_Retreat();									//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
// 	Mileage_Arrive_Wait(10);							//里程等待
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式
// 	//停止
// 	Stop_Control();
// 	osDelay(100);
// }

// /** @brief	平台8至景点3
//   **/
// void Platform8_to_Scenic_Spot3(void)
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


// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(15);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,125);				//左转135
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	//等待到达路口中心
// 	while(Grayscale_Out.Grayscale_Trigger_Num > 4) osDelay(1); 
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(150);									//等待车身稳定

// 	Car_Turn_Control(Turn_Left,105);				//左转135
// 	osDelay(100);									//等待车身稳定

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 		//微抬起前瞻，以便过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 45;			//舵机角度设定
// 		Mileage_Arrive_Wait(80);						//里程等待，经过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 35;			//降下前瞻

// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//直角1

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 	//直角2

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Right,90);			//行进中右转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(20);						//里程等待

// 		//微抬起前瞻，以便过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 45;			//舵机角度设定
// 		Mileage_Arrive_Wait(40);						//里程等待，经过桥洞
// 		Servo_State.Servo_Angle[Servo2] = 35;			//降下前瞻
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

// 	//直角3
// 	while(Grayscale_Out.Grayscale_Trigger_Num <= 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Turn_Control(Turn_Right,85);				//右转90度

// 	//Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	Mileage_Arrive_Wait(5);							//里程等待

// 	//获取当前角度
// 	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - TARGET_ANGLE_RIGHT_OFFSET;	//往右修正

// 	//停止
// 	Stop_Control();
// }

// /** @brief	景点3至C点
//   **/
// void Scenic_Spot3_to_Test_C(void)
// {
// 	Car_Retreat();									//小车后退控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

// 	Car_Arc_Turn_Control(Turn_Left,95);				//行进中左转90

// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);

// 	Car_Arc_Turn_Control(Turn_Right,95);			//行进中右转90

// 	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
// 	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
// 	Mileage_Arrive_Wait(180);						//里程等待
// 	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
// 	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
// 	Stop_Control();									//停止
// 	// osDelay(100);									//等待车身稳定
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
// 	Car_Retreat();									//小车后退控制
// 	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
// 	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

// 	//等待到达路口
// 	while(Grayscale_Out.Grayscale_Trigger_Num < 4) osDelay(1);
// 	Stop_Control();									//停止
// 	osDelay(100);									//等待车身稳定

// 	Car_Advance();									//小车前进控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
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

// 	Car_Retreat();											//小车后退控制
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
// 	Motor_Control_Parm.Base_Triger_Speed = 1.2;				//速度设置
// 	Mileage_Arrive_Wait(8);									//里程等待
// 	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

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
		RUN_Parm.Mileage_Int += ((Now_Mileage - Last_Mileage) / 4) * (Set_Mileage_100_Actual / (float)100);
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
			//上升沿触发
			if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7) == 1)
			{
				//开始运行
				RUN_Parm.RUN_State = 1;	
			}
		}
		//运行状态下
		if(RUN_Parm.RUN_State == 1)
		{
			RUN_Parm.Sensor_Middle++;
		}
	}
	//PF6被触发
	if(GPIO_Pin == GPIO_PIN_6)
		RUN_Parm.Sensor_Left++;
	//PF5被触发
	if(GPIO_Pin == GPIO_PIN_5)
		RUN_Parm.Sensor_Right++;

	//PF4,PF3被触发
	if(GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_4)
		RUN_Parm.Microswitch_State++;

}

