///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为舵机驱动源文件
///////////////////////////////////////
#include "Servo.h"
#include "stm32f4xx_hal.h"

#include "tim.h"

//角度-CCR分辨率
#define Angle_resolution	11.111111f

//舵机状态结构体
struct Servo Servo_State;

/** @brief	舵机初始化
  * @note	关闭所有PWM输出
  **/
void Servo_Init(void)
{
	HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_1 | TIM_CHANNEL_2);	//舵机1/2
	HAL_TIM_PWM_Stop(&htim11,TIM_CHANNEL_1);				//舵机3
	HAL_TIM_PWM_Stop(&htim10,TIM_CHANNEL_1);				//舵机4
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);	//舵机5/6
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1 | TIM_CHANNEL_2);	//舵机7/8
}

/** @brief	开启单独舵机
  * @param	Servo_Select	舵机编号
  **/
void Servo_Start_One(Servo_Number Servo_Select)
{
	//开启单独舵机
	switch(Servo_Select)
	{
		case Servo1:
				HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
			break;
		case Servo2:
				HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
			break;
		case Servo3:
				HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
			break;
		case Servo4:
				HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
			break;
		case Servo5:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
			break;
		case Servo6:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
			break;
		case Servo7:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			break;
		case Servo8:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			break;
	}
}

/** @brief	关闭单独舵机
  * @param	Servo_Select	舵机编号
  **/
void Servo_Stop_One(Servo_Number Servo_Select)
{
	//开启单独舵机
	switch(Servo_Select)
	{
		case Servo1:
				HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_2);
			break;
		case Servo2:
				HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_1);
			break;
		case Servo3:
				HAL_TIM_PWM_Stop(&htim11,TIM_CHANNEL_1);
			break;
		case Servo4:
				HAL_TIM_PWM_Stop(&htim10,TIM_CHANNEL_1);
			break;
		case Servo5:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
			break;
		case Servo6:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			break;
		case Servo7:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
			break;
		case Servo8:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
			break;
	}
}

/** @brief	舵机独立控制
  * @param	Servo_Select	舵机编号
  * @param	Direction		控制方向
  * @param	Angle			控制角度	范围：0~180
  **/
void Servo_Control_One(Servo_Number Servo_Select,Servo_Direction Direction,float Angle)
{
	//存储控制数据
	Servo_State.Servo_Angle[Servo_Select] = Angle;
	Servo_State.Servo_Direction_State[Servo_Select] = Direction;
	
	//角度限位
	if(Angle <= Servo_State.Servo_Angle_Min_Limit[Servo_Select])	Angle = Servo_State.Servo_Angle_Min_Limit[Servo_Select];
	if(Angle >= Servo_State.Servo_Angle_Max_Limit[Servo_Select])	Angle = Servo_State.Servo_Angle_Max_Limit[Servo_Select];
	
	//控制方向设置
	if(Direction) Angle = 180 - Angle;
	
	//计算CCR值
	short CCR = 500 + (Angle_resolution * Angle);
	
	//舵机输出PWM占空比赋值
	switch(Servo_Select)
	{
		case Servo1:
				TIM9 -> CCR2 = CCR;
			break;
		case Servo2:
				TIM9 -> CCR1 = CCR;
			break;
		case Servo3:
				TIM11 -> CCR1 = CCR;
			break;
		case Servo4:
				TIM10 -> CCR1 = CCR;
			break;
		case Servo5:
				TIM3 -> CCR2 = CCR;
			break;
		case Servo6:
				TIM3 -> CCR1 = CCR;
			break;
		case Servo7:
				TIM2 -> CCR2 = CCR;
			break;
		case Servo8:
				TIM2 -> CCR1 = CCR;
			break;
	}
}

/** @brief	舵机总控制
  * @param	Servo_Select	舵机编号
  * @param	Direction		控制方向
  * @param	Angle			控制角度	范围：0~180
  **/
void Servo_Control(void)
{
	static char Last_Servo_EN_State[8],Now_Servo_EN_State[8];
	
	for(char i=0;i<8;i++)
	{
		Last_Servo_EN_State[i] = Now_Servo_EN_State[i];
		Now_Servo_EN_State[i] = Servo_State.Servo_EN_State[i];
		//舵机使能上升沿，开启舵机
		if(Last_Servo_EN_State[i] == Servo_Disable && Now_Servo_EN_State[i] == Servo_Enable)
			Servo_Start_One((Servo_Number)i);
		//舵机使能下降沿，关闭舵机
		else if(Last_Servo_EN_State[i] == Servo_Enable && Now_Servo_EN_State[i] == Servo_Disable)
			Servo_Stop_One((Servo_Number)i);
	}
	//角度限位
	for(char i=0;i<8;i++)
	{
		if(Servo_State.Servo_Angle[i] <= Servo_State.Servo_Angle_Min_Limit[i])
			Servo_State.Servo_Angle[i] = Servo_State.Servo_Angle_Min_Limit[i];
		if(Servo_State.Servo_Angle[i] >= Servo_State.Servo_Angle_Max_Limit[i])
			Servo_State.Servo_Angle[i] = Servo_State.Servo_Angle_Max_Limit[i];
	}
	//执行控制
	Servo_Control_One(Servo1,(Servo_Direction)Servo_State.Servo_Direction_State[0],Servo_State.Servo_Angle[0]);
	Servo_Control_One(Servo2,(Servo_Direction)Servo_State.Servo_Direction_State[1],Servo_State.Servo_Angle[1]);
	Servo_Control_One(Servo3,(Servo_Direction)Servo_State.Servo_Direction_State[2],Servo_State.Servo_Angle[2]);
	Servo_Control_One(Servo4,(Servo_Direction)Servo_State.Servo_Direction_State[3],Servo_State.Servo_Angle[3]);
	Servo_Control_One(Servo5,(Servo_Direction)Servo_State.Servo_Direction_State[4],Servo_State.Servo_Angle[4]);
	Servo_Control_One(Servo6,(Servo_Direction)Servo_State.Servo_Direction_State[5],Servo_State.Servo_Angle[5]);
	Servo_Control_One(Servo7,(Servo_Direction)Servo_State.Servo_Direction_State[6],Servo_State.Servo_Angle[6]);
	Servo_Control_One(Servo8,(Servo_Direction)Servo_State.Servo_Direction_State[7],Servo_State.Servo_Angle[7]);
	
}


