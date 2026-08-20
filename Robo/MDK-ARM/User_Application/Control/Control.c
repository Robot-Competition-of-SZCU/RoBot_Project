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
	Motor_Control_Parm.Turn_Speed = 2.5;
	RUN_Parm.Arc_Turn_End_Threshold = 3.0f;	//弧线转弯完成判定阈值
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
	RUN_Parm.Control_State	= Turn_Control;			//转弯控制

	Stop_Control();									//停止

	//抬起前瞻
	Servo_State.Servo_Angle[Servo2] = 60;

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

	//降下前瞻
	Servo_State.Servo_Angle[Servo2] = 35;

	//调整为直行
	Car_Advance();
}

/** @brief	行进中弧线转弯
  * @param	Mode:转弯模式
  * @param	Turn_Angle:转弯角度，范围为0-180
  * @note	不停顿转弯，保持前进的同时通过左右差速完成转向
  * @note	转弯完成后自动恢复原速度并切换回巡线控制
  **/
void Car_Arc_Turn_Control(Turn_Mode Mode,float Turn_Angle)
{
	//限制转向角度范围
	if(Turn_Angle > 180) Turn_Angle = 180;
	if(Turn_Angle < 0) Turn_Angle = 0;

	//记录转弯完成后的恢复速度
	RUN_Parm.Arc_Turn_Resume_Speed = Motor_Control_Parm.Base_Triger_Speed;
	//设置转弯基础速度
	Motor_Control_Parm.Base_Triger_Speed = Motor_Control_Parm.Turn_Speed;

	//计算目标角度，处理回绕
	RUN_Parm.Arc_Turn_Target_Angle = IMU_Data.IMU_Angle_Z;
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
		}
		osDelay(1);
	}
	//5秒等待超时退出
	return 0;
}

/** @brief	C点至D点
  * @note   起始地点需正对着4号景点
  * @note   结束地点为正对着1号平台
  **/
void Test_C_to_Test_D(void)
{
	Test_C_to_Scenic_Spot4();		//C点至景点4
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点
	//osDelay(1000);
	Scenic_Spot4_to_Scenic_Spot5();	//景点4至景点5
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点
	//osDelay(1000);
	Scenic_Spot5_to_Test_D();		//景点5至D点
}

/** @brief	D点至C点
  * @note   起始地点需正对着5号平台
  * @note   结束地点为正对着D点
  **/
void Test_D_to_Test_C(void)
{
	//5号平台
	Test_D_to_Platform5();			//D点至平台5
	UART_Audio_Control(5);			//播报到达5号平台
	//osDelay(2000);

	//7号平台
	Turn_Around();					//原地转向
	Platform5_to_Platform7();		//平台5至平台7
	UART_Audio_Control(7);			//播报到达7号平台
	//osDelay(2000);

	//8号平台
	Turn_Around();					//原地转向
	Platform7_to_Platform8();		//平台7至平台8
	UART_Audio_Control(8);			//播报到达8号平台
	//osDelay(2000);

	//3号景点
	Turn_Around();					//原地转向
	Platform8_to_Scenic_Spot3();	//平台8至景点3
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点

	Scenic_Spot3_to_Test_C();		//景点3至C点
}

/** @brief	系统运行控制任务
  **/
void RUN_System_Control(void)
{
	//运行状态下
    if(RUN_Parm.RUN_State)
    {
		
		/*相机调试*/
		
		// UART_Visual_Identity_Send_Order('2');	//开启视觉识别
		// osDelay(500);
		// UART_Visual_Identity_Send_Order('2');	//进行文字识别
		// while(1) osDelay(1);
		
		
		//举手
		Servo_State.Servo_Angle[Servo5] = 180;
		Servo_State.Servo_Angle[Servo6] = 180;
		UART_Audio_Control(0);			//播报准备完毕
		osDelay(1000);
		//放手
		Servo_State.Servo_Angle[Servo5] = 0;
		Servo_State.Servo_Angle[Servo6] = 0;
		
			if(RUN_Parm.Round == 0)						//第一回合
				UART_Visual_Identity_Send_Order('2');	//开启视觉识别

		//2号台
		Platform1_to_Platform2();		//平台1至平台2
		UART_Audio_Control(2);			//播报到达2号平台
		//osDelay(2000);

			if(RUN_Parm.Round == 0)						//第一回合
				UART_Visual_Identity_Send_Order('2');	//进行文字识别

		//2号景点
		Turn_Around();					//原地转向
		Platform2_to_Scenic_Spot2();	//平台2至景点2
		Word_Test_Recognition();		//文字识别
		Hit_The_Scenic_Spot();			//撞景点

		//4号平台
		Scenic_Spot2_to_Platform4();	//景点2至平台4
		UART_Audio_Control(4);			//播报到达4号平台
		//osDelay(2000);

		//1号景点
		Turn_Around();					//原地转向
		Platform4_to_Scenic_Spot1();	//平台4至景点1
		Word_Test_Recognition();		//文字识别
		Hit_The_Scenic_Spot();			//撞景点

				UART_Visual_Identity_Send_Order('9');	//退出文字识别

		//3号平台
		Scenic_Spot1_to_Platform3();	//景点1至平台3
		UART_Audio_Control(3);			//播报到达3号平台
		//osDelay(2000);

				UART_Visual_Identity_Send_Order('1');	//进入颜色识别

		Turn_Around();					//原地转向

				UART_Visual_Identity_Send_Order('7');	//保存白平衡

		Platform3_to_Test_A();			//平台3至A点

		Traffic_Sign_1_Test();			//通行指示牌1检测
		//进行A点检测
		if(Colour_Test_Recognition() == 1)
		{	
					UART_Visual_Identity_Send_Order('9');	//退出颜色识别

			Traffic_Sign_1_Pass();				//通行指示牌1通过
			RUN_Parm.Common_Traffic_Sign = 1;	//1号通道可通过
			goto Rounter2;						//前往阶段2
		}
		else
			Traffic_Sign_2_Test();			//通行指示牌2检测

		if(Colour_Test_Recognition() == 1)
		{	
					UART_Visual_Identity_Send_Order('9');	//退出颜色识别

			Traffic_Sign_2_Pass();				//通行指示牌2通过
			RUN_Parm.Common_Traffic_Sign = 2;	//2号通道可通过
			goto Rounter2;						//前往阶段2
		}
		else
			Traffic_Sign_3_Test();			//通行指示牌3检测

		if(Colour_Test_Recognition() == 1)
		{	
					UART_Visual_Identity_Send_Order('9');	//退出颜色识别

			Traffic_Sign_3_Pass();				//通行指示牌3通过
			RUN_Parm.Common_Traffic_Sign = 3;	//3号通道可通过
			goto Rounter2;						//前往阶段2
		}
		else
			Traffic_Sign_4_Test();			//通行指示牌4检测

					UART_Visual_Identity_Send_Order('9');	//退出颜色识别

			Traffic_Sign_4_Pass();			//通行指示牌4通过
		RUN_Parm.Common_Traffic_Sign = 4;	//4号通道可通过

		//阶段2
		Rounter2:

				UART_Visual_Identity_Send_Order('2');	//进行文字识别

		//进入后段路程
		switch(RUN_Parm.Common_Traffic_Sign)
		{
			//可通过路口为1
			case 1:	
			case 3:
					Test_C_to_Test_D();				//C点至D点

					Car_Arc_Turn_Control(Turn_Right,85);	//行进中右转90

					Test_D_to_Test_C();				//D点至C点
					break;
			case 2:
			case 4:
					Test_D_to_Test_C();				//D点至C点

					Car_Arc_Turn_Control(Turn_Right,85);	//行进中右转90

					Test_C_to_Test_D();				//C点至D点
					break;
		}

		//回家
		switch(RUN_Parm.Common_Traffic_Sign)
		{
			case 1:	Test_C_to_Test_A_to_Alpha();		//C点至A点至Alpha
					break;
			case 2:	Test_D_to_Test_A_to_Alpha();		//C点至B点至Alpha
					break;
			case 3: Test_C_to_Test_B_to_Alpha();		//D点至A点至Alpha
					break;
			case 4:	Test_D_to_Test_B_to_Alpha();		//D点至B点至Alpha
					break;
		}
		Aplha_to_Platform1();				//Alpha点至平台1
		Turn_Around();						//原地转向

		UART_Audio_Control(1);				//播报到达1号平台
		//osDelay(2000);

		//停止
		//RUN_Parm.RUN_Control = 0;
		RUN_Parm.RUN_State = 0;
		RUN_Parm.Round++;					//回合累计
		//while(1) osDelay(1);	
	}
}


/** @brief	上平台
  **/
void Go_Up_Platform(void)
{	
	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

	Mileage_Arrive_Wait(5);						//里程等待

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(10);						//里程等待
	//清除微动开关触发状态
	RUN_Parm.Microswitch_State = 0;
	//等待微动开关触发
	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

	osDelay(100);

	Car_Retreat();									//小车后退控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	Mileage_Arrive_Wait(8);							//里程等待
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

	//停止
	Stop_Control();
}

/** @brief	撞景点
  **/
void Hit_The_Scenic_Spot(void)
{
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
	
	//清除微动开关触发状态
	RUN_Parm.Microswitch_State = 0;
	//等待微动开关触发
	while(!RUN_Parm.Microswitch_State) osDelay(1);

	Car_Retreat();									//小车后退控制
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	Mileage_Arrive_Wait(8);							//里程等待
	//停止
	Stop_Control();
}

/** @brief	下平台
  **/
void Go_Down_Platform(void)
{
	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
	Mileage_Arrive_Wait(10);							//里程等待
	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Mileage_Arrive_Wait(10);						//里程等待

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
}

/** @brief	原地转向
  **/
void Turn_Around(void)
{
	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

	//举手
	Servo_State.Servo_Angle[Servo5] = 180;
	Servo_State.Servo_Angle[Servo6] = 180;
	osDelay(200);									//等待200ms

	Car_Turn_Control(Turn_Left,172.5);				//左转180度

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	//放手
	Servo_State.Servo_Angle[Servo5] = 0;
	Servo_State.Servo_Angle[Servo6] = 0;
	osDelay(200);									//等待200ms
}

/** @brief	翻越山
  **/
void Cross_the_Mountain(void)
{
	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;	

	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(30);						//里程等待

	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定

	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制

	Mileage_Arrive_Wait(30);						//里程等待
	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
}

/** @brief	翻越长波浪板
  **/
void Cross_the_Long_Wave_Board(void)
{
	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	Servo_State.Servo_Angle[Servo2] = 60;			//舵机角度设定

	Motor_Control_Parm.Base_Triger_Speed = 1;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(10);						//里程等待

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
	Mileage_Arrive_Wait(10);						//里程等待

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
	Mileage_Arrive_Wait(120);						//里程等待
	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
}

/** @brief	翻越短波浪板
  **/
void Cross_the_Short_Wave_Board(void)
{
	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	Servo_State.Servo_Angle[Servo2] = 60;			//舵机角度设定

	Motor_Control_Parm.Base_Triger_Speed = 1;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(10);						//里程等待

	Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定
	Mileage_Arrive_Wait(10);						//里程等待

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
	Mileage_Arrive_Wait(70);						//里程等待
	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
}

/** @brief	景点1至平台3
  **/
void Scenic_Spot1_to_Platform3(void)
{
	Car_Retreat();									//小车后退控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	// //等待到达路口中心
	// while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1);

	Stop_Control();									//停止
	osDelay(100);

	Car_Advance();									//小车前进控制
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	// Mileage_Arrive_Wait(10);							//里程等待

	// Stop_Control();									//停止
	//osDelay(100);


	Car_Arc_Turn_Control(Turn_Left,10);				//行进中左转
	
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	Go_Up_Platform();								//上平台
}

/** @brief	平台4至景点1
  **/
void Platform4_to_Scenic_Spot1(void)
{
	Go_Down_Platform();								//下平台
	Mileage_Arrive_Wait(50);						//里程等待

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
	Mileage_Arrive_Wait(400);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[0] + Grayscale.Grayscale_Trigger_State[1] +
			Grayscale.Grayscale_Trigger_State[2]+ Grayscale.Grayscale_Trigger_State[3]+
			Grayscale.Grayscale_Trigger_State[4]+ Grayscale.Grayscale_Trigger_State[5]+
			Grayscale.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

	Car_Arc_Turn_Control(Turn_Right,15);			//行进中右转

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待

	//复位中部传感器触发状态
	RUN_Parm.Sensor_Middle = 0;
	//等待中部传感器触发
	while(!RUN_Parm.Sensor_Middle) osDelay(1);

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z - 1.5f;	//往右修正1.5
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	Mileage_Arrive_Wait(5);							//里程等待

	Stop_Control();									//停止
}

/** @brief	景点2至平台4
  **/
void Scenic_Spot2_to_Platform4(void)
{
	Car_Retreat();									//小车后退控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(30);						//里程等待
	
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1);

	Stop_Control();									//停止
	osDelay(100);

	Car_Advance();									//小车前进控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	//Mileage_Arrive_Wait(10);						//里程等待

	Car_Arc_Turn_Control(Turn_Right,20);			//行进中右转

	Car_Advance();									//小车前进控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	Go_Up_Platform();								//上平台
}

/** @brief	平台2至景点2
  **/
void Platform2_to_Scenic_Spot2(void)
{
	Go_Down_Platform();								//下平台
	Mileage_Arrive_Wait(30);						//里程等待
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Right,35);			//行进中右转
	
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(30);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	
	Cross_the_Mountain();							//翻越山
	
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Mileage_Arrive_Wait(20);						//里程等待
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	osDelay(100);

	Car_Arc_Turn_Control(Turn_Left,5);			//行进中左转

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
	Mileage_Arrive_Wait(160);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	osDelay(150);

	Car_Arc_Turn_Control(Turn_Left,30);				//行进中左转

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//复位中部传感器触发状态
	RUN_Parm.Sensor_Middle = 0;
	//等待中部传感器触发
	while(!RUN_Parm.Sensor_Middle) osDelay(1);

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	Mileage_Arrive_Wait(5);							//里程等待

	Stop_Control();									//停止
}

/** @brief	平台1至平台2
  **/
void Platform1_to_Platform2(void)
{
	Car_Advance();									//小车前进控制

	Go_Down_Platform();								//下平台

	// Servo_State.Servo_Angle[Servo2] = 10;			//舵机角度设定
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
	Mileage_Arrive_Wait(10);						//里程等待
	// Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
	// Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设定

	Mileage_Arrive_Wait(100);						//里程等待

	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设定
	Servo_State.Servo_Angle[Servo2] = 80;			//舵机角度设定
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(30);						//里程等待
	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待达到设定里程
	RUN_Parm.Mileage_Int	= 0;					//复位里程计
	RUN_Parm.Mileage_Parm	= 120;					//里程设定
	PD4_LED_Control(LED_ON);						//开启PD4LED
	while(RUN_Parm.Mileage_Int < RUN_Parm.Mileage_Parm) 
	{
		//左侧传感器端口为低电平
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6) == 0)
		{	//角度偏移
			RUN_Parm.Target_Angle -= 0.1f;
			RUN_Parm.Sensor_Left = 0;
		}
		//右侧传感器端口为低电平
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5) == 0)
		{	//角度偏移
			RUN_Parm.Target_Angle += 0.1f;
			RUN_Parm.Sensor_Right = 0;
		}
		osDelay(100);
	}	
	PD4_LED_Control(LED_OFF);				//关闭PD4LED
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	
	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Mileage_Arrive_Wait(40);						//里程等待

	Go_Up_Platform();								//上平台
}

/** @brief	平台3至A点
  **/
void Platform3_to_Test_A(void)
{
	Go_Down_Platform();								//下平台
	Mileage_Arrive_Wait(50);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(150);						//里程等待

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[8] + Grayscale.Grayscale_Trigger_State[9] +
			Grayscale.Grayscale_Trigger_State[10]+ Grayscale.Grayscale_Trigger_State[11]+
			Grayscale.Grayscale_Trigger_State[12]+ Grayscale.Grayscale_Trigger_State[13]+
			Grayscale.Grayscale_Trigger_State[14]) > 5)) osDelay(1);

	// Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	// osDelay(100);									//等待车身稳定
}

/** @brief	通行指示牌1检测
  **/
void Traffic_Sign_1_Test(void)
{
	Car_Arc_Turn_Control(Turn_Right,85);	//行进中右转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);					//右转90度
}

/** @brief	通行指示牌1通过
  **/
void Traffic_Sign_1_Pass(void)
{
	Car_Turn_Control(Turn_Left,85);						//左转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[13] + Grayscale.Grayscale_Trigger_State[12] +
			Grayscale.Grayscale_Trigger_State[11]+ Grayscale.Grayscale_Trigger_State[10]+
			Grayscale.Grayscale_Trigger_State[9]+ Grayscale.Grayscale_Trigger_State[8]+
			Grayscale.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	//Stop_Control();								//停止
	//osDelay(100);									//等待车身稳定
}

/** @brief	通行指示牌2检测
  **/
void Traffic_Sign_2_Test(void)
{
	Car_Turn_Control(Turn_Right,85);				//右转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[0] + Grayscale.Grayscale_Trigger_State[1] +
			Grayscale.Grayscale_Trigger_State[2]+ Grayscale.Grayscale_Trigger_State[3]+
			Grayscale.Grayscale_Trigger_State[4]+ Grayscale.Grayscale_Trigger_State[5]+
			Grayscale.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(150);									//等待车身稳定

	Car_Turn_Control(Turn_Right,125);				//右转
	osDelay(100);

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);					//右转90度
}

/** @brief	通行指示牌2通过
  **/
void Traffic_Sign_2_Pass(void)
{
	Car_Turn_Control(Turn_Left,85);						//左转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(150);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 

	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转

	// Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	//Stop_Control();								//停止
	//osDelay(100);									//等待车身稳定
}

/** @brief	通行指示牌3检测
  **/
void Traffic_Sign_3_Test(void)
{
	Car_Turn_Control(Turn_Right,85);				//右转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,140);				//右转

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(200);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[13] + Grayscale.Grayscale_Trigger_State[12] +
			Grayscale.Grayscale_Trigger_State[11]+ Grayscale.Grayscale_Trigger_State[10]+
			Grayscale.Grayscale_Trigger_State[9]+ Grayscale.Grayscale_Trigger_State[8]+
			Grayscale.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,130);				//右转

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);				//右转90度
}

/** @brief	通行指示牌3通过
  **/
void Traffic_Sign_3_Pass(void)
{
	Car_Turn_Control(Turn_Left,85);					//左转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(150);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 

	Car_Arc_Turn_Control(Turn_Left,40);	//行进中左转
}

/** @brief	通行指示牌4检测
  **/
void Traffic_Sign_4_Test(void)
{
	Car_Turn_Control(Turn_Right,85);				//右转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,110);				//右转
	osDelay(100);

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);				//右转90度
}

/** @brief	通行指示牌4通过
  **/
void Traffic_Sign_4_Pass(void)
{
	Car_Turn_Control(Turn_Left,85);					//左转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转
}
/** @brief	C点至景点4
  **/
void Test_C_to_Scenic_Spot4(void)
{
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);				//右转90度
	osDelay(100);									//等待车身稳定

	//Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	Mileage_Arrive_Wait(5);							//里程等待

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	//停止
	Stop_Control();
}

/** @brief	景点4至景点5
  **/
void Scenic_Spot4_to_Scenic_Spot5(void)
{
	Car_Retreat();									//小车后退控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Advance();									//小车前进控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90


	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(200);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Left,85);					//左转90度

	//Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90度

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	Mileage_Arrive_Wait(5);							//里程等待

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	//停止
	Stop_Control();
}

/** @brief	景点5至D点
  **/
void Scenic_Spot5_to_Test_D(void)
{
	Car_Retreat();									//小车后退控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Advance();									//小车前进控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	// osDelay(100);									//等待车身稳定
}

/** @brief	D点至平台5
  **/
void Test_D_to_Platform5(void)
{
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(160);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	Go_Up_Platform();								///上平台
}

/** @brief	平台5至平台7
  **/
void Platform5_to_Platform7(void)
{
	Go_Down_Platform();								//下平台
	Mileage_Arrive_Wait(20);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设定
	Mileage_Arrive_Wait(50);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Left,40);				//行进中左转45

	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(150);									//等待车身稳定

	Car_Turn_Control(Turn_Left,142);				//左转
	osDelay(100);									//等待车身稳定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(80);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

	Cross_the_Mountain();							//翻越第一座山
	Mileage_Arrive_Wait(30);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(60);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

	Cross_the_Mountain();							//翻越第二座山
	Mileage_Arrive_Wait(30);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(70);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(30);						//里程等待

	Cross_the_Long_Wave_Board();					//翻越长波浪板
	//Mileage_Arrive_Wait(10);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	
	////////////
	//上高台
	////////////

	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

	Mileage_Arrive_Wait(50);						//里程等待

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	//Mileage_Arrive_Wait(10);						//里程等待
	//清除微动开关触发状态
	RUN_Parm.Microswitch_State = 0;
	//等待微动开关触发
	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

	osDelay(100);

	Car_Retreat();									//小车后退控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	Mileage_Arrive_Wait(8);							//里程等待
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

	//停止
	Stop_Control();
}

/** @brief	平台7至平台8
  **/
void Platform7_to_Platform8(void)
{
	Servo_State.Servo_Angle[Servo2] = 0;			//舵机角度设定
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定

	Mileage_Arrive_Wait(50);						//里程等待

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定

	//等待Y轴姿态大于-5
	while(!(IMU_Data.IMU_Angle_X > -5))	osDelay(1);	

	Mileage_Arrive_Wait(20);							//里程等待

	Cross_the_Long_Wave_Board();					//翻越长波浪板

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(40);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

	Cross_the_Mountain();							//翻越山
	Mileage_Arrive_Wait(30);						//里程等待

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(100);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

	Mileage_Arrive_Wait(50);						//里程等待

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(20);						//里程等待

	//到达一阶平台

	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待
	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(20);						//里程等待

	//到达二阶平台
	//清除微动开关触发状态
	RUN_Parm.Microswitch_State = 0;
	//等待微动开关触发
	while(!(RUN_Parm.Microswitch_State)) osDelay(1);

	osDelay(100);

	Car_Retreat();									//小车后退控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	Mileage_Arrive_Wait(8);							//里程等待
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式

	//停止
	Stop_Control();
}

/** @brief	平台8至景点3
  **/
void Platform8_to_Scenic_Spot3(void)
{
	///////////////////////////
	//变换巡线PID比例系数
	Line_Patrol_PID.Kp = 0.15f;
	///////////////////////////

	Servo_State.Servo_Angle[Servo2] = 0;			//舵机角度设定

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设定
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定

	Mileage_Arrive_Wait(40);						//里程等待

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);
	
	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	//到达一阶平台

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	Mileage_Arrive_Wait(20);						//里程等待

	//等待灰度触发数量大于0
	while(!Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定
	Mileage_Arrive_Wait(40);						//里程等待

	//等待Y轴姿态大于-5
	while(!(IMU_Data.IMU_Angle_X > -5))	osDelay(1);	
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设定


	///////////////////////////
	//变换巡线PID比例系数
	Line_Patrol_PID.Kp = 0.12f;
	///////////////////////////


	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Left,130);				//左转135
	osDelay(100);									//等待车身稳定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(5);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(150);									//等待车身稳定

	Car_Turn_Control(Turn_Left,120);				//左转135
	osDelay(100);									//等待车身稳定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//直角1

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//直角2

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 3.5;		//速度设置
	Mileage_Arrive_Wait(20);						//里程等待

	//直角3
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,85);				//右转90度

	//Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	Mileage_Arrive_Wait(5);							//里程等待

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;

	//停止
	Stop_Control();
}

/** @brief	景点3至C点
  **/
void Scenic_Spot3_to_Test_C(void)
{
	Car_Retreat();									//小车后退控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;		//后退模式
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Advance();									//小车前进控制
	RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;		//前进模式
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90

	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);

	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 6;		//速度设置
	Mileage_Arrive_Wait(180);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num < 4) osDelay(1);
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	// Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	// osDelay(100);									//等待车身稳定
}

/** @brief	Alpha点至平台1
  **/
void Aplha_to_Platform1(void)
{
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(15);						//里程等待

	Cross_the_Short_Wave_Board();					//翻越短波浪板

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待

	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

	//获取当前角度
	RUN_Parm.Target_Angle = IMU_Data.IMU_Angle_Z;
	Motor_Control_Parm.Base_Triger_Speed = 2;		//速度设置

	Mileage_Arrive_Wait(5);						//里程等待

	//等待灰度触发数量为0
	while(Grayscale.Grayscale_Trigger_Num) osDelay(1);	

	Motor_Control_Parm.Base_Triger_Speed = 1.5;		//速度设置
	RUN_Parm.Control_State	= Angle_Control;		//角度跟随控制
	//Mileage_Arrive_Wait(5);						//里程等待
	//清除左右侧光电传感器触发状态
	RUN_Parm.Sensor_Left = 0; RUN_Parm.Sensor_Right = 0;
	//等待左右侧光电传感器触发
	while(!RUN_Parm.Sensor_Left && !RUN_Parm.Sensor_Right) osDelay(1);	

	// Stop_Control();									//停止控制

	// Car_Retreat();									//小车后退控制
	// RUN_Parm.Angle_Control_Choice = Angle_Control_Retreat;	//后退控制模式
	// Motor_Control_Parm.Base_Triger_Speed = 1.2;		//速度设置
	// Mileage_Arrive_Wait(4);							//里程等待
	// RUN_Parm.Angle_Control_Choice = Angle_Control_Advance;	//前进控制模式
	// Car_Advance();									//小车前进控制

	//停止
	Stop_Control();
}

/** @brief	A点至识别1
  **/
void Test_A_to_recognition1(void)
{
	
}

/** @brief	C点至A点至Alpha
  **/
void Test_C_to_Test_A_to_Alpha(void)
{
	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(120);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[0] + Grayscale.Grayscale_Trigger_State[1] +
			Grayscale.Grayscale_Trigger_State[2]+ Grayscale.Grayscale_Trigger_State[3]+
			Grayscale.Grayscale_Trigger_State[4]+ Grayscale.Grayscale_Trigger_State[5]+
			Grayscale.Grayscale_Trigger_State[6]) > 5)) osDelay(1);

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

	Car_Arc_Turn_Control(Turn_Right,85);			//行进中右转90

	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转
}

/** @brief	C点至B点至Alpha
  **/
void Test_C_to_Test_B_to_Alpha(void)
{
	Car_Arc_Turn_Control(Turn_Left,40);				//行进中左转

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(250);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Left,145);				//左转
	osDelay(100);									//等待车身稳定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,135);				//右转
	osDelay(100);									//等待车身稳定
}

/** @brief	D点至A点至Alpha
  **/
void Test_D_to_Test_A_to_Alpha(void)
{
	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(250);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,135);				//右转
	osDelay(100);									//等待车身稳定

	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Car_Arc_Turn_Control(Turn_Left,45);				//行进中左转
}

/** @brief	D点至B点至Alpha
  **/
void Test_D_to_Test_B_to_Alpha(void)
{
	RUN_Parm.Control_State	= ScanLine_Control;		//巡线控制
	Motor_Control_Parm.Base_Triger_Speed = 5;		//速度设置
	Mileage_Arrive_Wait(120);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(!((Grayscale.Grayscale_Trigger_State[13] + Grayscale.Grayscale_Trigger_State[12] +
			Grayscale.Grayscale_Trigger_State[11]+ Grayscale.Grayscale_Trigger_State[10]+
			Grayscale.Grayscale_Trigger_State[9]+ Grayscale.Grayscale_Trigger_State[8]+
			Grayscale.Grayscale_Trigger_State[7]) > 5)) osDelay(1);

	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置

	Car_Arc_Turn_Control(Turn_Left,85);				//行进中左转90

	Motor_Control_Parm.Base_Triger_Speed = 4;		//速度设置
	Mileage_Arrive_Wait(50);						//里程等待
	Motor_Control_Parm.Base_Triger_Speed = 3;		//速度设置

	//等待到达路口
	while(Grayscale.Grayscale_Trigger_Num <= 4) osDelay(1);
	//等待到达路口中心
	while(Grayscale.Grayscale_Trigger_Num > 4) osDelay(1); 
	Motor_Control_Parm.Base_Triger_Speed = 2.5;		//速度设置
	Mileage_Arrive_Wait(10);						//里程等待，等待车身行驶至路口上方
	Stop_Control();									//停止
	osDelay(100);									//等待车身稳定

	Car_Turn_Control(Turn_Right,135);				//右转
	osDelay(100);									//等待车身稳定
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

