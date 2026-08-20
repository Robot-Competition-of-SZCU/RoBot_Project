///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为设置与显示源文件
///////////////////////////////////////
#include "Set_And_Show.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdio.h"

#include "User_Task.h"
#include "KEY.h"
#include "LED.h"
#include "OLED.h"
#include "Grayscale_ADC.h"
#include "Flash.h"
#include "Servo.h"
#include "Encoder.h"

#include "Control.h"
#include "PID.h"
#include "IMU.h"

//系统菜单结构体
struct Menu Menu_Parm;

/** @brief	系统菜单控制与显示
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void System_Menu_Control_And_Show(void)
{
	//获取按键状态
	KEY_Tigger_State R_KEY1			= Get_Key_State(KEY1);			//按键1
	KEY_Tigger_State R_KEY2			= Get_Key_State(KEY2);			//按键2
	KEY_Tigger_State R_KEY3			= Get_Key_State(KEY3);			//按键3
	KEY_Tigger_State R_KEY4			= Get_Key_State(KEY4);			//按键4
	KEY_Tigger_State R_Rocker_UP	= Get_Key_State(Rocker_UP);		//摇杆_上
	KEY_Tigger_State R_Rocker_Right	= Get_Key_State(Rocker_Right);	//摇杆_右
	KEY_Tigger_State R_Rocker_Down	= Get_Key_State(Rocker_Down);	//摇杆_下
	KEY_Tigger_State R_Rocker_Middle= Get_Key_State(Rocker_Middle);	//摇杆_中
	KEY_Tigger_State R_Rocker_Left	= Get_Key_State(Rocker_Left);	//摇杆_左

	//菜单等级显示
	switch(Menu_Parm.Menu_Level)
	{
		//一级菜单界面
		case Level1:Menu_Level1_Control(R_KEY1,R_Rocker_UP,R_Rocker_Down);
				break;
		//二级菜单界面
		case Level2:
				switch(Menu_Parm.Menu_Interface_L2)
				{
					//运行设置菜单
					case RUN_Set:
							Menu_Level2_RUN_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
							break;
					//系统设置菜单
					case System_Set:
							//二级系统菜单控制与显示
							Menu_Level2_System_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
							break;
					case Scan_Line_Set:
							//二级巡线菜单控制与显示
							Menu_Level2_Scan_Line_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);								
							break;
					case Servo_Set:
							//二级舵机设置菜单控制与显示
							Menu_Level2_Servo_Set(R_KEY1,R_KEY2);
							break;
					case Task_Show:
							//二级任务设置窗口控制与显示
							Menu_Level2_Task_Show(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
							break;
				}
				break;
		//三级菜单界面
		case Level3:
				switch(Menu_Parm.Menu_Interface_L2)
				{
					case RUN_Set:
							//二级运行菜单
							switch(Menu_Parm.Menu_Interface_L3)
							{
								case RUN_Parm_Set:
										//三级运行参数设置控制与显示
										Menu_RUN_Parm_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
										break;
								case RUN_View:
										//三级运行记录显示
										Menu_RUN_View(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
										break;
							}
							break;
					//系统设置菜单
					case System_Set:
							//二级系统菜单
							switch(Menu_Parm.Menu_Interface_L3)
							{
								case Speed_Set:
										//三级速度设置菜单控制与显示
										Menu_Level3_Speed_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
										break;
								case PID_Set:
										//三级PID设置菜单控制与显示
										Menu_Level3_PID_Set(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
										break;
								case Encoder_View:
										//三级编码器参数显示
										Menu_Level3_Encoder_View(R_KEY2,R_Rocker_Right,R_Rocker_Left,R_Rocker_UP,R_Rocker_Down);
										break;
								case IMU_View:
										Menu_Level3_IMU_View(R_KEY2);
										break;
							}
							break;
					case Scan_Line_Set:
							//二级系统菜单
							switch(Menu_Parm.Menu_Interface_L3)
							{
								case Gray_View:
										//三级灰度参数查看控制与显示
										Menu_Level3_Gray_View(R_KEY2,R_Rocker_Right,R_Rocker_Left);
										break;
								case Gray_Calib:
										//三级灰度校准控制与显示
										Menu_Level3_Gray_Calib(R_KEY1,R_KEY2);
										break;
							}							
							break;
					case Servo_Set:
							//二级舵机设置菜单
							//三级舵机控制与显示
							Menu_Level3_Servo_Control(R_KEY1,R_KEY2,R_Rocker_UP,R_Rocker_Down);
							break;
					case Task_Show:
							//二级任务设置窗口
							switch(Menu_Parm.Menu_Interface_L3)
							{
								case Task_LED:
										//三级运行指示灯任务显示
										Menu_Level3_Task_LED(R_KEY2);
										break;
								case Task_KEY_Sacn:
										//三级按键扫描任务显示
										Menu_Level3_Task_KEY_Sacn(R_KEY2);
										break;
								case Task_Show_Set:
										//三级设置与显示任务显示
										Menu_Level3_Task_Show(R_KEY2);
										break;
								case Task_UART_Debug:
										//三级串口调试任务显示
										Menu_Level3_Task_UART_Debug(R_KEY2);
										break;
								case Task_Slow_Com:
										//三级低速计算任务显示
										Menu_Level3_Task_Slow_Com(R_KEY2);
										break;
								case Task_High_Com:
										//三级高速计算任务显示
										Menu_Level3_Task_High_Com(R_KEY2);
										break;
							}
							break;
				}	
				break;
		//四级菜单界面
		case Level4:
			switch(Menu_Parm.Menu_Interface_L2)
				{
					//系统设置菜单
					case System_Set:
							//二级系统菜单
							switch(Menu_Parm.Menu_Interface_L3)
							{
								case Speed_Set:
										//三级速度设置菜单控制与显示
										switch(Menu_Parm.Menu_Interface_L4)
										{
											case Base_Speed_Set:
												//四级基础速度设置
												Menu_Level4_Speed_Set(R_KEY2,R_Rocker_UP,R_Rocker_Down,R_Rocker_Right,R_Rocker_Left);
												break;
											case Accelerated_Speed_Set:
												//四级加速度设置
												Menu_Level4_Accelerated_Speed_Set(R_KEY2,R_Rocker_UP,R_Rocker_Down,R_Rocker_Right,R_Rocker_Left);
												break;
										}
										break;
								case PID_Set:
										//三级PID设置菜单控制与显示
										switch(Menu_Parm.Menu_Interface_L4)
										{
											case Motor_Speed_PID:
												//电机转速PID查看
												Menu_Level4_Motor_Speed_PID(R_KEY2);
												break;
											case Line_P_PID:
												//巡线PID查看
												Menu_Level4_Line_Patrol_PID(R_KEY2);
												break;
										}
										break;
							}
				break;
				}
		//五级菜单界面
		case Level5:
				break;
	}
	//OLED刷新
	OLED_Update();
	//OLED复位
	OLED_Clear();
	//复位所有键码状态
	Key_State_Clear();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///* 一级菜单显示 *///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief 一级主菜单控制与显示
  * @param	KEY1		按键1
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level1_Control(KEY_Tigger_State KEY1,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;	//菜单指针
	static int Menu_Y_Shift=0;		//显示Y轴偏移
	//摇杆下按,下翻选项
	if(Rocker_Down)
	{	Menu_Pointer++;							//菜单指针自增
		if(Menu_Pointer == 5) Menu_Pointer = 0;	//超过长度，回到顶端
		//显示Y轴偏移
		if(Menu_Pointer >= 3) Menu_Y_Shift++;	//偏移
		if(Menu_Y_Shift >= 1) Menu_Y_Shift = 1;	//限位
		if(Menu_Pointer == 0) Menu_Y_Shift = 0;	//复位
	}
	//摇杆上按，上翻选项
	if(Rocker_UP)
	{	Menu_Pointer--;							//菜单指针自减
		if(Menu_Pointer < 0) Menu_Pointer = 4;	//超过长度，回到底端
		//显示Y轴偏移
		if(Menu_Pointer <= 1) Menu_Y_Shift--;	//偏移
		if(Menu_Y_Shift <= 0) Menu_Y_Shift = 0;	//限位
		if(Menu_Pointer == 4) Menu_Y_Shift = 1;	//复位
	}
	//按键1按下，进入所选菜单
	if(KEY1)
	{	//菜单等级+1
		Menu_Parm.Menu_Level++;
		//二级菜单的界面为当前菜单指针所指向的界面
		Menu_Parm.Menu_Interface_L2 = Menu_Pointer;
	}

	//菜单显示
	OLED_ShowString(0,0-Menu_Y_Shift*16,"运行设置",OLED_8X16);
	OLED_ShowString(0,16-Menu_Y_Shift*16,"系统设置",OLED_8X16);
	OLED_ShowString(0,32-Menu_Y_Shift*16,"巡线设置",OLED_8X16);
	OLED_ShowString(0,48-Menu_Y_Shift*16,"舵机设置",OLED_8X16);
	OLED_ShowString(0,64-Menu_Y_Shift*16,"任务查看",OLED_8X16);

	//菜单指针指向部分高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0-Menu_Y_Shift*16,64,16);
				break;
		case 1:	OLED_ReverseArea(0,16-Menu_Y_Shift*16,64,16);
				break;
		case 2:	OLED_ReverseArea(0,32-Menu_Y_Shift*16,64,16);
				break;
		case 3:	OLED_ReverseArea(0,48-Menu_Y_Shift*16,64,16);
				break;
		case 4:	OLED_ReverseArea(0,64-Menu_Y_Shift*16,64,16);
				break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///* 二级菜单显示 *///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief	二级运行设置菜单控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level2_RUN_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;	//菜单指针
	//摇杆下按，下翻选项
	if(Rocker_Down)
	{	Menu_Pointer++;							//菜单指针自增
		if(Menu_Pointer == 3) Menu_Pointer = 0;	//超过长度，回到顶端
	}
	//摇杆上按，上翻选项
	if(Rocker_UP)
	{	Menu_Pointer--;							//菜单指针自减
		if(Menu_Pointer < 0) Menu_Pointer = 2;	//超过长度，回到底端
	}
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//退出当前菜单
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//按键1按下，进入所选菜单
	if(KEY1)
	{
		//当前菜单指针为0
		if(Menu_Pointer == 0)
			//切换控制状态
			RUN_Parm.RUN_Control = !RUN_Parm.RUN_Control;
		//其他状态
		else
		{
			//菜单等级+1
			Menu_Parm.Menu_Level++;
			//三级菜单的界面为当前菜单指针所指向的界面
			Menu_Parm.Menu_Interface_L3 = Menu_Pointer - 1;
		}
	}

	//菜单显示
	if(RUN_Parm.RUN_Control)
		OLED_ShowString(0,0,"正在运行",OLED_8X16);
	else
		OLED_ShowString(0,0,"开始运行",OLED_8X16);
	OLED_ShowString(0,16,"运行设置",OLED_8X16);
	OLED_ShowString(0,32,"运行查看",OLED_8X16);
	
	//控制指针指向部分高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,64,16);
				break;
		case 1:	OLED_ReverseArea(0,16,64,16);
				break;
		case 2:	OLED_ReverseArea(0,32,64,16);
				break;
	}
}

/** @brief	二级系统菜单控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level2_System_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;	//菜单指针
	//摇杆下按，下翻选项
	if(Rocker_Down)
	{	Menu_Pointer++;							//菜单指针自增
		if(Menu_Pointer == 4) Menu_Pointer = 0;	//超过长度，回到顶端
	}
	//摇杆上按，上翻选项
	if(Rocker_UP)
	{	Menu_Pointer--;							//菜单指针自减
		if(Menu_Pointer < 0) Menu_Pointer = 3;	//超过长度，回到底端
	}
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//退出当前菜单
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//按键1按下，进入所选菜单
	if(KEY1)
	{	//菜单等级+1
		Menu_Parm.Menu_Level++;
		//三级菜单的界面为当前菜单指针所指向的界面
		Menu_Parm.Menu_Interface_L3 = Menu_Pointer;
	}

	//菜单显示
	OLED_ShowString(0,0,"速度设置",OLED_8X16);
	OLED_ShowString(0,16,"PID设置",OLED_8X16);
	OLED_ShowString(0,32,"编码器查看",OLED_8X16);
	OLED_ShowString(0,48,"IMU数据查看",OLED_8X16);
	
	//控制指针指向部分高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,64,16);
				break;
		case 1:	OLED_ReverseArea(0,16,64,16);
				break;
		case 2:	OLED_ReverseArea(0,32,80,16);
				break;
		case 3:	OLED_ReverseArea(0,48,88,16);
				break;
	}
}

/** @brief	二级巡线菜单控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level2_Scan_Line_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	static signed char Menu_Y_Shift=0;	//显示Y轴偏移
	//摇杆下按，下翻选项
	if(Rocker_Down)
	{	//菜单指针无偏移
		if(Menu_Pointer < 10)
		{
			Menu_Pointer++;							//菜单指针自增
			if(Menu_Pointer == 5) Menu_Pointer = 0;	//超过长度，回到顶端
			//显示Y轴偏移
			if(Menu_Pointer >= 3) Menu_Y_Shift++;	//偏移
			if(Menu_Y_Shift >= 1) Menu_Y_Shift = 1;	//限位
			if(Menu_Pointer == 0) Menu_Y_Shift = 0;	//复位
		}
		//指针偏移为13，增加触发阈值
		else if(Menu_Pointer == 13)
		{
			Grayscale.Grayscale_ADC_Trigger_Threshold += 1;
			if(Grayscale.Grayscale_ADC_Trigger_Threshold >= 100)
				Grayscale.Grayscale_ADC_Trigger_Threshold =100;
		}
	}
	//摇杆上按，上翻选项
	if(Rocker_UP)
	{	//指针无偏移
		if(Menu_Pointer < 10)
		{
			Menu_Pointer--;							//菜单控制指针自减
			if(Menu_Pointer < 0) Menu_Pointer = 4;	//超过长度，回到底端
			//显示Y轴偏移
			if(Menu_Pointer <= 1) Menu_Y_Shift--;	//偏移
			if(Menu_Y_Shift <= 0) Menu_Y_Shift = 0;	//限位
			if(Menu_Pointer == 4) Menu_Y_Shift = 1;	//复位
		}
		//指针偏移为13，增加触发阈值
		else if(Menu_Pointer == 13)
		{
			Grayscale.Grayscale_ADC_Trigger_Threshold -= 1;
			if(Grayscale.Grayscale_ADC_Trigger_Threshold <= 0)
				Grayscale.Grayscale_ADC_Trigger_Threshold =0;
		}
	}
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//控制指针存在偏移
		if(Menu_Pointer >= 10)
		//消除偏移
			Menu_Pointer -=10;
		else
		{	//菜单等级-1
			Menu_Parm.Menu_Level--;
		}	
	}
	//按键1按下，复合操作
	if(KEY1)
	{
		switch(Menu_Pointer)
		{
			//控制指针在第一二栏，进入当前所选择的三级级菜单
			case 0:
			case 1:
					//菜单等级+1
					Menu_Parm.Menu_Level++;
					//三级菜单的界面为当前控制指针所指向的界面
					Menu_Parm.Menu_Interface_L3 = Menu_Pointer;
					break;
			//控制指针在第三栏，操作为翻转灰度计算模式
			case 2:
					//计算模式设置
					Grayscale.Compute_Map_Mode =! Grayscale.Compute_Map_Mode;
					break;
			//控制指针在第四栏，操作为进入调整触发阈值
			case 3:
					//指针偏移10
					Menu_Pointer += 10;
					break;
			//控制指针在第五栏，操作为存储当前数据
			case 4:
					//进行Flash写数据操作
					OLED_Show_And_Set_Write_Flash();
					//退出本次循环
					return;
		}
	}

	//菜单显示
	OLED_ShowString(0,0-Menu_Y_Shift*16,"灰度参数查看",OLED_8X16);
	OLED_ShowString(0,16-Menu_Y_Shift*16,"校准设置",OLED_8X16);
	OLED_ShowString(0,32-Menu_Y_Shift*16,"计算模式:",OLED_8X16);
	OLED_ShowString(0,48-Menu_Y_Shift*16,"触发阈值:",OLED_8X16);
	OLED_ShowString(0,64-Menu_Y_Shift*16,"是否存储参数→",OLED_8X16);

	OLED_Printf(90,48-Menu_Y_Shift*16,OLED_8X16,"%3.0f",Grayscale.Grayscale_ADC_Trigger_Threshold);
	
	if(Grayscale.Compute_Map_Mode == 0)
		OLED_ShowString(80,32-Menu_Y_Shift*16,"正向",OLED_8X16);
	else
		OLED_ShowString(80,32-Menu_Y_Shift*16,"反向",OLED_8X16);
	
	//控制指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0-Menu_Y_Shift*16,96,16);
				break;
		case 1:	OLED_ReverseArea(0,16-Menu_Y_Shift*16,64,16);
				break;
		case 2:	OLED_ReverseArea(0,32-Menu_Y_Shift*16,64,16);
				break;
		case 3:	OLED_ReverseArea(0,48-Menu_Y_Shift*16,64,16);
				break;
		case 4:	OLED_ReverseArea(0,64-Menu_Y_Shift*16,112,16);
				break;
		case 13:OLED_ReverseArea(90,48-Menu_Y_Shift*16,24,16);
				break;		
	}
}
/** @brief	二级舵机设置菜单控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level2_Servo_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2)
{
	static int Menu_Pointer = 0;		//菜单指针
	//按键1按下，进入当前所选择的三级级菜单
	if(KEY1)
	{	//菜单等级+1
		Menu_Parm.Menu_Level++;
		//三级菜单的界面为当前控制指针所指向的界面
		Menu_Parm.Menu_Interface_L3 = Menu_Pointer;
	}
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	
	//菜单显示
	OLED_ShowString(0,0,"舵机直接控制",OLED_8X16);
	//OLED_ShowString(0,16,"舵机初始设置",OLED_8X16);
	
	
	//伪指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,96,16);
				break;
		case 1:	OLED_ReverseArea(0,16,96,16);
				break;
	}
}

/** @brief	二级任务设置窗口控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level2_Task_Show(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	static signed char Menu_Y_Shift=0;	//显示Y轴偏移
	//摇杆下按
	if(Rocker_Down)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer == 6) Menu_Pointer = 0;	
		//菜单偏移
		if(Menu_Pointer >= 3) Menu_Y_Shift++;		//偏移
		if(Menu_Y_Shift >= 2)	Menu_Y_Shift = 2;	//限位
		if(Menu_Pointer == 0) Menu_Y_Shift = 0;		//复位
	}
	//摇杆上按
	if(Rocker_UP)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer < 0) Menu_Pointer = 5;
		//菜单偏移
		if(Menu_Pointer <= 2) Menu_Y_Shift--;		//偏移
		if(Menu_Y_Shift <= 0)	Menu_Y_Shift = 0;	//限位		
		if(Menu_Pointer == 5) Menu_Y_Shift = 2;		//复位
	}
	//按键1按下，进入当前所选择的三级级菜单
	if(KEY1)
	{	//菜单等级+1
		Menu_Parm.Menu_Level++;
		//三级菜单的界面为当前控制指针所指向的界面
		Menu_Parm.Menu_Interface_L3 = Menu_Pointer;
	}
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_ShowString(0,0-Menu_Y_Shift*16,"运行指示灯任务",OLED_8X16);
	OLED_ShowString(0,16-Menu_Y_Shift*16,"按键扫描任务",OLED_8X16);
	OLED_ShowString(0,32-Menu_Y_Shift*16,"设置与显示任务",OLED_8X16);
	OLED_ShowString(0,48-Menu_Y_Shift*16,"串口调试任务",OLED_8X16);
	OLED_ShowString(0,64-Menu_Y_Shift*16,"低速计算任务",OLED_8X16);
	OLED_ShowString(0,80-Menu_Y_Shift*16,"高速计算任务",OLED_8X16);

	//控制指针指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0-Menu_Y_Shift*16,112,16);
				break;
		case 1:	OLED_ReverseArea(0,16-Menu_Y_Shift*16,96,16);
				break;
		case 2:	OLED_ReverseArea(0,32-Menu_Y_Shift*16,112,16);
				break;
		case 3:	OLED_ReverseArea(0,48-Menu_Y_Shift*16,96,16);
				break;
		case 4:	OLED_ReverseArea(0,64-Menu_Y_Shift*16,96,16);
				break;
		case 5:OLED_ReverseArea(0,80-Menu_Y_Shift*16,96,16);
				break;		
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///* 三级菜单显示 *///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief	三级运行参数设置控制与显示
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_RUN_Parm_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	//按键2按下，退出
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//显示
	OLED_ShowString(0,0,"无需设置",OLED_8X16);
}

/** @brief	三级运行记录显示
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_RUN_View(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	static signed char Menu_Y_Shift=0;	//显示Y轴偏移
	//摇杆向下按
	if(Rocker_Down)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer == 7) Menu_Pointer = 0;	
		//菜单偏移
		if(Menu_Pointer >= 3) Menu_Y_Shift++;		//偏移
		if(Menu_Y_Shift >= 3) Menu_Y_Shift = 3;		//限位
		if(Menu_Pointer == 0) Menu_Y_Shift = 0;		//复位	
	}
	//摇杆向上按
	if(Rocker_UP)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer < 0) Menu_Pointer = 6;
		//菜单偏移
		if(Menu_Pointer <= 3) Menu_Y_Shift --;		//偏移	
		if(Menu_Y_Shift <= 0) Menu_Y_Shift = 0;		//限位
		if(Menu_Pointer == 6) Menu_Y_Shift = 3;	//复位
	}

	//按键2按下，退出
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//显示
	OLED_Printf(0,0 - Menu_Y_Shift*16,OLED_8X16,"直立景点%d个",RUN_Parm.Word_Test_Num);
	OLED_Printf(0,16 - Menu_Y_Shift*16,OLED_8X16,"1-5号平台%d个",5 * RUN_Parm.Round);
	OLED_Printf(0,32 - Menu_Y_Shift*16,OLED_8X16,"6号平台0个");
	OLED_Printf(0,48 - Menu_Y_Shift*16,OLED_8X16,"7号平台%d个",RUN_Parm.Round);
	OLED_Printf(0,64 - Menu_Y_Shift*16,OLED_8X16,"8号平台%d个",RUN_Parm.Round);
	OLED_Printf(0,80 - Menu_Y_Shift*16,OLED_8X16,"回家%d次",RUN_Parm.Round);
	OLED_Printf(0,96 - Menu_Y_Shift*16,OLED_8X16,"总分%4d分",((RUN_Parm.Word_Test_Num * 11) + 390) * 1.2 - 30);

	//控制指针指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0 - Menu_Y_Shift*16,80,16);
				break;
		case 1:	OLED_ReverseArea(0,16 - Menu_Y_Shift*16,80,16);
				break;
		case 2:	OLED_ReverseArea(0,32 - Menu_Y_Shift*16,80,16);
				break;
		case 3:	OLED_ReverseArea(0,48 - Menu_Y_Shift*16,80,16);
				break;
		case 4:	OLED_ReverseArea(0,64 - Menu_Y_Shift*16,80,16);
				break;
		case 5:	OLED_ReverseArea(0,80 - Menu_Y_Shift*16,80,16);
				break;
		case 6:	OLED_ReverseArea(0,96 - Menu_Y_Shift*16,80,16);
				break;
	}
}

/** @brief	三级速度设置菜单控制与显示
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Speed_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	//摇杆下按
	if(Rocker_Down)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer == 4) Menu_Pointer = 0;	
	}
	//摇杆上按
	if(Rocker_UP)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer < 0) Menu_Pointer = 3;
	}
	//按键2按下
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//按键1按下，进入所选菜单
	if(KEY1)
	{	
		if(Menu_Pointer == 3)
		{
			//flash写数据
			OLED_Show_And_Set_Write_Flash();
			return;
		}

		//菜单等级+1
		Menu_Parm.Menu_Level++;
		//四级菜单的界面为当前菜单指针所指向的界面
		Menu_Parm.Menu_Interface_L4 = Menu_Pointer;
	}

	//菜单显示
	OLED_ShowString(0,0,"基础速度",OLED_8X16);
	OLED_ShowString(0,16,"加速度",OLED_8X16);
	OLED_ShowString(0,32,"转向速度",OLED_8X16);
	OLED_ShowString(0,48,"是否存储参数→",OLED_8X16);

	//控制指针指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,64,16);
				break;
		case 1:	OLED_ReverseArea(0,16,48,16);
				break;
		case 2:	OLED_ReverseArea(0,32,64,16);
				break;
		case 3:	OLED_ReverseArea(0,48,112,16);
		break;
	}
}

/** @brief	三级PID设置菜单控制与显示
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_PID_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	//摇杆下按
	if(Rocker_Down)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer == 2) Menu_Pointer = 0;	
	}
	//摇杆上按
	if(Rocker_UP)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer < 0) Menu_Pointer = 1;
	}
	//按键2按下
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//按键1按下，进入所选菜单
	if(KEY1)
	{	//菜单等级+1
		Menu_Parm.Menu_Level++;
		//四级菜单的界面为当前菜单指针所指向的界面
		Menu_Parm.Menu_Interface_L4 = Menu_Pointer;
	}

	//菜单显示
	OLED_ShowString(0,0,"转速环",OLED_8X16);
	OLED_ShowString(0,16,"巡线环",OLED_8X16);
	
	//控制指针指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,48,16);
				break;
		case 1:	OLED_ReverseArea(0,16,48,16);
				break;
	}
}

/** @brief	三级编码器参数显示
  * @param	KEY2		按键2
  * @param	Rocker_Right	摇杆上
  * @param	Rocker_Left	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Encoder_View(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	//按键2按下
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//摇杆右按
	if(Rocker_Right)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer > 5)
			{if(Menu_Pointer == 13) Menu_Pointer = 10;}
		else
			if(Menu_Pointer == 3) Menu_Pointer = 0;	
	}
	//摇杆左按
	if(Rocker_Left)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer > 5)
			{if(Menu_Pointer < 10) Menu_Pointer = 12;}
		else
			if(Menu_Pointer < 0) Menu_Pointer = 2;
	}
	//摇杆下按
	if(Rocker_Down)
	{
		//切换显示
		if(Menu_Pointer < 5)
			Menu_Pointer += 10;
	}
	if(Rocker_UP)
	{
		if(Menu_Pointer > 5)
			Menu_Pointer -= 10;
	}

	//菜单显示
	OLED_ShowString(0,0,"脉冲",OLED_8X16);
	OLED_ShowString(48,0,"旋转",OLED_8X16);
	OLED_ShowString(96,0,"线速",OLED_8X16);
	OLED_ShowString(0,16,"累计",OLED_8X16);
	OLED_ShowString(48,16,"累计",OLED_8X16);
	OLED_ShowString(96,16,"累计",OLED_8X16);

	OLED_ShowChar(0,32,'M',OLED_6X8);
	OLED_ShowChar(0,40,'1',OLED_6X8);
	OLED_ShowChar(0,48,'M',OLED_6X8);
	OLED_ShowChar(0,56,'2',OLED_6X8);
	OLED_ShowChar(8,32,':',OLED_8X16);
	OLED_ShowChar(8,48,':',OLED_8X16);

	OLED_ShowChar(64,32,'M',OLED_6X8);
	OLED_ShowChar(64,40,'3',OLED_6X8);
	OLED_ShowChar(64,48,'M',OLED_6X8);
	OLED_ShowChar(64,56,'4',OLED_6X8);
	OLED_ShowChar(72,32,':',OLED_8X16);
	OLED_ShowChar(72,48,':',OLED_8X16);

	//显示不同参数
	switch(Menu_Pointer)
	{
		//实时脉冲
		case 0:	OLED_Printf(16,32,OLED_8X16,"%5d",Encoder1.Pulse);
				OLED_Printf(16,48,OLED_8X16,"%5d",Encoder2.Pulse);
				OLED_Printf(80,32,OLED_8X16,"%5d",Encoder3.Pulse);
				OLED_Printf(80,48,OLED_8X16,"%5d",Encoder4.Pulse);
				break;
		//实时旋转圈数
		case 1:	OLED_Printf(16,32,OLED_8X16,"%5.2f",Encoder1.RMP_S);
				OLED_Printf(16,48,OLED_8X16,"%5.2f",Encoder2.RMP_S);
				OLED_Printf(80,32,OLED_8X16,"%5.2f",Encoder3.RMP_S);
				OLED_Printf(80,48,OLED_8X16,"%5.2f",Encoder4.RMP_S);
				break;
		//实时线速度
		case 2:	OLED_Printf(16,32,OLED_8X16,"%5.1f",Encoder1.Speed);
				OLED_Printf(16,48,OLED_8X16,"%5.1f",Encoder2.Speed);
				OLED_Printf(80,32,OLED_8X16,"%5.1f",Encoder3.Speed);
				OLED_Printf(80,48,OLED_8X16,"%5.1f",Encoder4.Speed);
				break;

		//脉冲累计
		case 10:OLED_Printf(16,32,OLED_8X16,"%5d",Encoder1.Pulse_Int);
				OLED_Printf(16,48,OLED_8X16,"%5d",Encoder2.Pulse_Int);
				OLED_Printf(80,32,OLED_8X16,"%5d",Encoder3.Pulse_Int);
				OLED_Printf(80,48,OLED_8X16,"%5d",Encoder4.Pulse_Int);
				break;
		//旋转圈数累计
		case 11:OLED_Printf(16,32,OLED_8X16,"%5.1f",Encoder1.RMP_Int);
				OLED_Printf(16,48,OLED_8X16,"%5.1f",Encoder2.RMP_Int);
				OLED_Printf(80,32,OLED_8X16,"%5.1f",Encoder3.RMP_Int);
				OLED_Printf(80,48,OLED_8X16,"%5.1f",Encoder4.RMP_Int);
				break;
		//行进距离累计
		case 12:OLED_Printf(16,32,OLED_8X16,"%5.1f",Encoder1.RUN_Int);
				OLED_Printf(16,48,OLED_8X16,"%5.1f",Encoder2.RUN_Int);
				OLED_Printf(80,32,OLED_8X16,"%5.1f",Encoder3.RUN_Int);
				OLED_Printf(80,48,OLED_8X16,"%5.1f",Encoder4.RUN_Int);
				break;
	}

	//控制指针指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,32,16);
				break;
		case 1:	OLED_ReverseArea(48,0,32,16);
				break;
		case 2:	OLED_ReverseArea(96,0,32,16);
				break;

		case 10:OLED_ReverseArea(0,16,32,16);
				break;
		case 11:OLED_ReverseArea(48,16,32,16);
				break;
		case 12:OLED_ReverseArea(96,16,32,16);
				break;
	}
}

/** @brief	三级IMU数据显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_IMU_View(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//参数显示
	OLED_Printf(0,0,OLED_8X16,"IMU_X: %6.2f",IMU_Data.IMU_Angle_X);
	OLED_Printf(0,16,OLED_8X16,"IMU_Y: %6.2f",IMU_Data.IMU_Angle_Y);
	OLED_Printf(0,32,OLED_8X16,"IMU_Z: %6.2f",IMU_Data.IMU_Angle_Z);

}

/** @brief	三级灰度参数查看控制与显示
  * @param	KEY2		按键2
  * @param	Rocker_Right	摇杆上
  * @param	Rocker_Left	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Gray_View(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left)
{
	static int Menu_Pointer = 0;		//菜单指针
	//摇杆下按
	if(Rocker_Right)
	{	//选择下翻
		Menu_Pointer++;
		//超过长度，回到顶端
		if(Menu_Pointer == 2) Menu_Pointer = 0;	
	}
	//摇杆上按
	if(Rocker_Left)
	{	//选择上翻
		Menu_Pointer--;
		//超过长度，回到底端
		if(Menu_Pointer < 0) Menu_Pointer = 1;
	}
	//按键2按下
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_ShowString(0,0,"校准值    实际值",OLED_8X16);
	
	//参数显示
	if(Menu_Pointer == 0)
	{	//显示校准值
		OLED_Printf(0,16,OLED_6X8,"1:%3.0f 7:%3.0f 13:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[0],Grayscale.Grayscale_ADC_Compute_Percent[6],Grayscale.Grayscale_ADC_Compute_Percent[12]);
		OLED_Printf(0,24,OLED_6X8,"2:%3.0f 8:%3.0f 14:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[1],Grayscale.Grayscale_ADC_Compute_Percent[7],Grayscale.Grayscale_ADC_Compute_Percent[13]);
		OLED_Printf(0,32,OLED_6X8,"3:%3.0f 9:%3.0f 15:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[2],Grayscale.Grayscale_ADC_Compute_Percent[8],Grayscale.Grayscale_ADC_Compute_Percent[14]);
		OLED_Printf(0,40,OLED_6X8,"4:%3.0f 10:%3.0f 16:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[3],Grayscale.Grayscale_ADC_Compute_Percent[9],Grayscale.Grayscale_ADC_Compute_Percent[15]);
		OLED_Printf(0,48,OLED_6X8,"5:%3.0f 11:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[4],Grayscale.Grayscale_ADC_Compute_Percent[10]);
		OLED_Printf(0,56,OLED_6X8,"6:%3.0f 12:%3.0f",Grayscale.Grayscale_ADC_Compute_Percent[5],Grayscale.Grayscale_ADC_Compute_Percent[11]);
	}
	else
	{	//显示实际值
		OLED_Printf(0,16,OLED_6X8,"1:%4d 7:%4d 13:%4d",Grayscale.Grayscale_ADC_Actual[0],Grayscale.Grayscale_ADC_Actual[6],Grayscale.Grayscale_ADC_Actual[12]);
		OLED_Printf(0,24,OLED_6X8,"2:%4d 8:%4d 14:%4d",Grayscale.Grayscale_ADC_Actual[1],Grayscale.Grayscale_ADC_Actual[7],Grayscale.Grayscale_ADC_Actual[13]);
		OLED_Printf(0,32,OLED_6X8,"3:%4d 9:%4d 15:%4d",Grayscale.Grayscale_ADC_Actual[2],Grayscale.Grayscale_ADC_Actual[8],Grayscale.Grayscale_ADC_Actual[14]);
		OLED_Printf(0,40,OLED_6X8,"4:%4d 10:%4d16:%4d",Grayscale.Grayscale_ADC_Actual[3],Grayscale.Grayscale_ADC_Actual[9],Grayscale.Grayscale_ADC_Actual[15]);
		OLED_Printf(0,48,OLED_6X8,"5:%4d 11:%4d",Grayscale.Grayscale_ADC_Actual[4],Grayscale.Grayscale_ADC_Actual[10]);
		OLED_Printf(0,56,OLED_6X8,"6:%4d 12:%4d",Grayscale.Grayscale_ADC_Actual[5],Grayscale.Grayscale_ADC_Actual[11]);
	}
	
	//控制指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0,48,16);
				break;
		case 1:	OLED_ReverseArea(80,0,64,16);
				break;
	}
}

/** @brief	三级灰度校准控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Gray_Calib(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2)
{
	static int Menu_Pointer = 0;		//菜单指针
	//按键2按下，退出当前菜单
	if(KEY2)
	{	//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//按键1长按，开始校准
	if(KEY1 == long_Tigger)
	{
		//最大极限值校准
		if(Menu_Pointer == 0)
		{	
			//清除显示缓存区
			OLED_Clear();
			OLED_ShowString(24,8,"正在校准",OLED_8X16);
			OLED_ShowString(16,24,"最大极限值",OLED_8X16);
			OLED_Update();		//刷新屏幕
			for(;;)
			{	//校准最大极限值，校准完退出循环
				if(Grayscale_ADC_Calibration(Calibration_Max) == HAL_OK)
					break;
				//延时1ms
				osDelay(1);
			}
			//延时1000ms，留影
			osDelay(1000);
			//清除显示缓存区
			OLED_Clear();
			OLED_ShowString(24,40,"校准完成",OLED_8X16);
			OLED_ShowString(16,24,"最大极限值",OLED_8X16);
			OLED_Update();		//刷新屏幕
			//延时1500ms，留影
				osDelay(1500);
			//控制指针指向选择+1
			Menu_Pointer++;
			//退出
			goto out;
		}
		//最小极限值校准
		if(Menu_Pointer == 1)
		{	
			//清除显示缓存区
			OLED_Clear();
			OLED_ShowString(24,8,"正在校准",OLED_8X16);
			OLED_ShowString(16,24,"最小极限值",OLED_8X16);
			OLED_Update();		//刷新屏幕
			for(;;)
			{	//校准最大极限值，校准完退出循环
				if(Grayscale_ADC_Calibration(Calibration_Min) == HAL_OK)
					break;
				//延时1ms
				osDelay(1);
			}
			//延时1000ms，留影
			osDelay(1000);
			//清除显示缓存区
			OLED_Clear();
			OLED_ShowString(24,40,"校准完成",OLED_8X16);
			OLED_ShowString(16,24,"最小极限值",OLED_8X16);
			OLED_Update();		//刷新屏幕
			//延时1500ms，留影
				osDelay(1500);
			//控制指针指向选择-1
			Menu_Pointer--;
		}
	}
	out:

	//菜单显示
	if(Menu_Pointer == 0)
		OLED_ShowString(0,0,"当前校准  最大值",OLED_8X16);
	else if(Menu_Pointer == 1)
		OLED_ShowString(0,0,"当前校准  最小值",OLED_8X16);
	
	//显示实际值
	OLED_Printf(0,16,OLED_6X8,"1:%4d 7:%4d 13:%4d",Grayscale.Grayscale_ADC_Actual[0],Grayscale.Grayscale_ADC_Actual[6],Grayscale.Grayscale_ADC_Actual[12]);
	OLED_Printf(0,24,OLED_6X8,"2:%4d 8:%4d 14:%4d",Grayscale.Grayscale_ADC_Actual[1],Grayscale.Grayscale_ADC_Actual[7],Grayscale.Grayscale_ADC_Actual[13]);
	OLED_Printf(0,32,OLED_6X8,"3:%4d 9:%4d 15:%4d",Grayscale.Grayscale_ADC_Actual[2],Grayscale.Grayscale_ADC_Actual[8],Grayscale.Grayscale_ADC_Actual[14]);
	OLED_Printf(0,40,OLED_6X8,"4:%4d 10:%4d16:%4d",Grayscale.Grayscale_ADC_Actual[3],Grayscale.Grayscale_ADC_Actual[9],Grayscale.Grayscale_ADC_Actual[15]);
	OLED_Printf(0,48,OLED_6X8,"5:%4d 11:%4d",Grayscale.Grayscale_ADC_Actual[4],Grayscale.Grayscale_ADC_Actual[10]);
	OLED_Printf(0,56,OLED_6X8,"6:%4d 12:%4d",Grayscale.Grayscale_ADC_Actual[5],Grayscale.Grayscale_ADC_Actual[11]);
	
	//高亮显示
	OLED_ReverseArea(80,0,48,16);
}

/** @brief	三级舵机控制与显示
  * @param	KEY1		按键1
  * @param	KEY2		按键2
  * @param	Rocker_UP	摇杆上
  * @param	Rocker_Down	摇杆下
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Servo_Control(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down)
{
	static int Menu_Pointer = 0;		//菜单指针
	static signed char Servo_Select=0;	//舵机选择
	static signed char Menu_Y_Shift=0;	//显示Y轴偏移
	//摇杆向下按
	if(Rocker_Down)
	{	//指针在10以内，操作为下翻一级控制指针指向选择
		if(Menu_Pointer < 10)
		{	//选择下翻
			Menu_Pointer++;
			//超过长度，回到顶端
			if(Menu_Pointer == 7) Menu_Pointer = 0;	
			//菜单偏移
			if(Menu_Pointer >= 3) Menu_Y_Shift++;		//偏移
			if(Menu_Y_Shift >= 3) Menu_Y_Shift = 3;		//限位
			if(Menu_Pointer == 0) Menu_Y_Shift = 0;		//复位
		}
		//指针为10，操作为切换控制的舵机
		else if(Menu_Pointer == 10)
		{
			Servo_Select++;
			Servo_Select%=8;
		}
		//指针为11，操作为切换所控制的舵机的状态
		else if(Menu_Pointer == 11)
			Servo_State.Servo_EN_State[Servo_Select] = !Servo_State.Servo_EN_State[Servo_Select];
		//指针为12，操作为切换所控制的舵机的状态
		else if(Menu_Pointer == 12)
			Servo_State.Servo_Direction_State[Servo_Select] = !Servo_State.Servo_Direction_State[Servo_Select];
		//指针为13，当前控制的舵机角度增加
		else if(Menu_Pointer == 13)
		{
			Servo_State.Servo_Angle[Servo_Select]+=1;
			if(Servo_State.Servo_Angle[Servo_Select] >= Servo_State.Servo_Angle_Max_Limit[Servo_Select])
				Servo_State.Servo_Angle[Servo_Select] = Servo_State.Servo_Angle_Max_Limit[Servo_Select];
		}
		//指针为14，当前控制的舵机最大极限角度增加
		else if(Menu_Pointer == 14)
		{
			Servo_State.Servo_Angle_Max_Limit[Servo_Select]+=1;
			if(Servo_State.Servo_Angle_Max_Limit[Servo_Select] >= 180)
				Servo_State.Servo_Angle_Max_Limit[Servo_Select] = 180;
		}
		//指针为15，当前控制的舵机最小极限角度增加
		else if(Menu_Pointer == 15)
		{
			Servo_State.Servo_Angle_Min_Limit[Servo_Select]+=1;
			if(Servo_State.Servo_Angle_Min_Limit[Servo_Select] >= 180)
				Servo_State.Servo_Angle_Min_Limit[Servo_Select] = 180;
		}
			
	}
	//摇杆向上按
	if(Rocker_UP)
	{	//指针在10以内，操作为上翻一级控制指针指向选择
		if(Menu_Pointer < 10)
		{	//选择上翻
			Menu_Pointer--;
			//超过长度，回到底端
			if(Menu_Pointer < 0) Menu_Pointer = 6;
			//菜单偏移
			if(Menu_Pointer <= 3) Menu_Y_Shift --;		//偏移	
			if(Menu_Y_Shift <= 0) Menu_Y_Shift = 0;		//限位
			if(Menu_Pointer == 6) Menu_Y_Shift = 3;	//复位
		}
		//指针为10，操作为切换控制的舵机
		else if(Menu_Pointer == 10)
		{
			Servo_Select--;
			if(Servo_Select < 0) Servo_Select = 7;
		}
		//指针为11，操作为切换所控制的舵机的状态
		else if(Menu_Pointer == 11)
			Servo_State.Servo_EN_State[Servo_Select] = !Servo_State.Servo_EN_State[Servo_Select];
		//指针为12，操作为切换所控制的舵机的状态
		else if(Menu_Pointer == 12)
			Servo_State.Servo_Direction_State[Servo_Select] = !Servo_State.Servo_Direction_State[Servo_Select];
		//指针为13，当前控制的舵机角度减小
		else if(Menu_Pointer == 13)
		{
			Servo_State.Servo_Angle[Servo_Select]-=1;
			if(Servo_State.Servo_Angle[Servo_Select] <= Servo_State.Servo_Angle_Min_Limit[Servo_Select])
				Servo_State.Servo_Angle[Servo_Select] = Servo_State.Servo_Angle_Min_Limit[Servo_Select];
		}
		//指针为14，当前控制的舵机最大极限角度减小
		else if(Menu_Pointer == 14)
		{
			Servo_State.Servo_Angle_Max_Limit[Servo_Select]-=1;
			if(Servo_State.Servo_Angle_Max_Limit[Servo_Select] <= 0)
				Servo_State.Servo_Angle_Max_Limit[Servo_Select] = 0;
		}
		//指针为15，当前控制的舵机最小极限角度减小
		else if(Menu_Pointer == 15)
		{
			Servo_State.Servo_Angle_Min_Limit[Servo_Select]-=1;
			if(Servo_State.Servo_Angle_Min_Limit[Servo_Select] <= 0)
				Servo_State.Servo_Angle_Min_Limit[Servo_Select] = 0;
		}
	}
	//按键2按下
	if(KEY2)
	{	//指针在10以内，操作为退出当前菜单
		if(Menu_Pointer < 10)
		{	//菜单等级-1
			Menu_Parm.Menu_Level--;
		}
		//指针不在10以内，操作指针内偏移10
		else 
			Menu_Pointer -= 10;
	}
	
	//按键1按下，且指针在10以内，操作为进入数据调整
	if(KEY1 && Menu_Pointer < 10)
	{	
		//指针外偏移10
		Menu_Pointer += 10;
		//若偏移为16，则为指向数据存储命令
		if(Menu_Pointer == 16)
		{	
			//进行Flash写数据操作
			OLED_Show_And_Set_Write_Flash();
			//操作指针内偏移10
			Menu_Pointer -= 10;
			//退出本次循环
			return;
		}
	}

	//菜单显示
	OLED_ShowString(0,0 - Menu_Y_Shift*16,"舵机编号  舵机",OLED_8X16);
	OLED_ShowString(0,16 - Menu_Y_Shift*16,"舵机状态",OLED_8X16);
	OLED_ShowString(0,32 - Menu_Y_Shift*16,"舵机方向",OLED_8X16);
	OLED_ShowString(0,48 - Menu_Y_Shift*16,"舵机角度",OLED_8X16);
	OLED_ShowString(0,64 - Menu_Y_Shift*16,"最大角度",OLED_8X16);
	OLED_ShowString(0,80 - Menu_Y_Shift*16,"最小角度",OLED_8X16);
	OLED_ShowString(0,96 - Menu_Y_Shift*16,"是否存储参数→",OLED_8X16);
	
	//参数显示
	//舵机编号
	OLED_ShowNum(112,0 - Menu_Y_Shift*16,Servo_Select+1,1,OLED_8X16);
	//舵机状态
	if(Servo_State.Servo_EN_State[Servo_Select] == Servo_Disable)
		OLED_ShowString(80,16 - Menu_Y_Shift*16,"失能",OLED_8X16);
	else
		OLED_ShowString(80,16 - Menu_Y_Shift*16,"使能",OLED_8X16);
	//舵机方向
	if(Servo_State.Servo_Direction_State[Servo_Select] == Forward)
		OLED_ShowString(80,32 - Menu_Y_Shift*16,"正向",OLED_8X16);
	else
		OLED_ShowString(80,32 - Menu_Y_Shift*16,"反向",OLED_8X16);
	//舵机角度
	OLED_Printf(80,48 - Menu_Y_Shift*16,OLED_8X16,"%3.0f",Servo_State.Servo_Angle[Servo_Select]);
	//最大角度
	OLED_Printf(80,64 - Menu_Y_Shift*16,OLED_8X16,"%3.0f",Servo_State.Servo_Angle_Max_Limit[Servo_Select]);
	//最小角度
	OLED_Printf(80,80 - Menu_Y_Shift*16,OLED_8X16,"%3.0f",Servo_State.Servo_Angle_Min_Limit[Servo_Select]);
	
	//控制指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(0,0 - Menu_Y_Shift*16,64,16);
				break;
		case 1:	OLED_ReverseArea(0,16 - Menu_Y_Shift*16,64,16);
				break;
		case 2:	OLED_ReverseArea(0,32 - Menu_Y_Shift*16,64,16);
				break;
		case 3:	OLED_ReverseArea(0,48 - Menu_Y_Shift*16,64,16);
				break;
		case 4:	OLED_ReverseArea(0,64 - Menu_Y_Shift*16,64,16);
				break;
		case 5:	OLED_ReverseArea(0,80 - Menu_Y_Shift*16,64,16);
				break;
		case 6:	OLED_ReverseArea(0,96 - Menu_Y_Shift*16,112,16);
				break;
		case 10:OLED_ReverseArea(80,0 - Menu_Y_Shift*16,40,16);
				break;
		case 11:OLED_ReverseArea(80,16 - Menu_Y_Shift*16,32,16);
				break;
		case 12:OLED_ReverseArea(80,32 - Menu_Y_Shift*16,32,16);
				break;
		case 13:OLED_ReverseArea(80,48 - Menu_Y_Shift*16,24,16);
				break;
		case 14:OLED_ReverseArea(80,64 - Menu_Y_Shift*16,24,16);
				break;
		case 15:OLED_ReverseArea(80,80 - Menu_Y_Shift*16,24,16);
				break;
	}
}

/** @brief	三级运行指示灯任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_LED(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"RUN_LED_Flash",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_RUN_LED_Flash.usStackHighWaterMark);
}

/** @brief	三级按键扫描任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_KEY_Sacn(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"KEY_Scan",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_KEY_Scan.usStackHighWaterMark);
}

/** @brief	三级设置与显示任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_Show(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"Set_And_Show",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_Set_And_Show.usStackHighWaterMark);
}

/** @brief	三级串口调试任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_UART_Debug(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"UART_Debug",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_UART_Debug.usStackHighWaterMark);
}

/** @brief	三级低速计算任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_Slow_Com(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"Slow_Compute",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_Slow_Compute.usStackHighWaterMark);
}

/** @brief	三级高速计算任务显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level3_Task_High_Com(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}
	//菜单显示
	OLED_ShowString(0,0,"任务名称",OLED_8X16);
	OLED_ShowString(0,16,"High_Compute",OLED_8X16);
	OLED_ShowString(0,32,"最小剩余栈",OLED_8X16);
	OLED_Printf(0,48,OLED_8X16,"%d",xTaskDetails_High_Compute.usStackHighWaterMark);
}

/** @brief	四级基础速度设置与显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level4_Speed_Set(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left)
{
	char Show_Num[8];
	//字符化参数
	sprintf(Show_Num,"%05.2f",Motor_Control_Parm.Base_High_Speed_Set);

	 static int Menu_Pointer = 0;		//菜单指针
	//摇杆左按
	if(Rocker_Left)
	{	//选择左移
		Menu_Pointer--;
		//超过长度，回到顶端
	 	if(Menu_Pointer < 0) Menu_Pointer = 3;	
	}
	//摇杆右按
	 if(Rocker_Right)
	 {	//选择右移
		Menu_Pointer++;
		//超过长度，回到顶端
	 	if(Menu_Pointer >= 4) Menu_Pointer = 0;	
	 }
	//摇杆下按
	if(Rocker_Down)
	{	//所选参数加
		if(Menu_Pointer<=1)
		{
			Show_Num[Menu_Pointer]++;
			if(Show_Num[Menu_Pointer] > '9') Show_Num[Menu_Pointer] = '0';
		}
		else
		{
			Show_Num[Menu_Pointer + 1]++;
			if(Show_Num[Menu_Pointer + 1] > '9') Show_Num[Menu_Pointer + 1] = '0';
		}
	}
	//摇杆上按
	if(Rocker_UP)
	{	//所选参数减
		if(Menu_Pointer<=1)
		{
			Show_Num[Menu_Pointer]--;
			if(Show_Num[Menu_Pointer] < '0') Show_Num[Menu_Pointer] = '9';
		}
		else
		{
			Show_Num[Menu_Pointer + 1]--;
			if(Show_Num[Menu_Pointer + 1] < '0') Show_Num[Menu_Pointer + 1] = '9';
		}
	}
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_ShowString(16,0,"基础速度",OLED_8X16);
	OLED_ShowString(24,32,"RMP/S",OLED_8X16);
	OLED_ShowString(24,16,Show_Num,OLED_8X16);
	
	//控制指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(24,16,8,16);
				break;
		case 1:	OLED_ReverseArea(32,16,8,16);
				break;
		case 2:	OLED_ReverseArea(48,16,8,16);
				break;
		case 3:	OLED_ReverseArea(56,16,8,16);
				break;
	}

	//参数化字符
	sscanf(Show_Num,"%f",&Motor_Control_Parm.Base_High_Speed_Set);
}

/** @brief	四级加速度设置与显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level4_Accelerated_Speed_Set(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left)
{
	char Show_Num[8];
	//字符化参数
	sprintf(Show_Num,"%05.2f",Motor_Control_Parm.Accelerated_Speed);

	 static int Menu_Pointer = 0;		//菜单指针
	//摇杆左按
	if(Rocker_Left)
	{	//选择左移
		Menu_Pointer--;
		//超过长度，回到顶端
	 	if(Menu_Pointer < 0) Menu_Pointer = 3;	
	}
	//摇杆右按
	 if(Rocker_Right)
	 {	//选择右移
		Menu_Pointer++;
		//超过长度，回到顶端
	 	if(Menu_Pointer >= 4) Menu_Pointer = 0;	
	 }
	//摇杆下按
	if(Rocker_Down)
	{	//所选参数加
		if(Menu_Pointer<=1)
		{
			Show_Num[Menu_Pointer]++;
			if(Show_Num[Menu_Pointer] > '9') Show_Num[Menu_Pointer] = '0';
		}
		else
		{
			Show_Num[Menu_Pointer + 1]++;
			if(Show_Num[Menu_Pointer + 1] > '9') Show_Num[Menu_Pointer + 1] = '0';
		}
	}
	//摇杆上按
	if(Rocker_UP)
	{	//所选参数减
		if(Menu_Pointer<=1)
		{
			Show_Num[Menu_Pointer]--;
			if(Show_Num[Menu_Pointer] < '0') Show_Num[Menu_Pointer] = '9';
		}
		else
		{
			Show_Num[Menu_Pointer + 1]--;
			if(Show_Num[Menu_Pointer + 1] < '0') Show_Num[Menu_Pointer + 1] = '9';
		}
	}
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_ShowString(16,0,"加速度",OLED_8X16);
	OLED_ShowString(24,32,"RMP/S",OLED_8X16);
	OLED_ShowString(24,16,Show_Num,OLED_8X16);
	
	//控制指针指向显示,指向的参数高亮显示
	switch(Menu_Pointer)
	{	
		case 0:	OLED_ReverseArea(24,16,8,16);
				break;
		case 1:	OLED_ReverseArea(32,16,8,16);
				break;
		case 2:	OLED_ReverseArea(48,16,8,16);
				break;
		case 3:	OLED_ReverseArea(56,16,8,16);
				break;
	}

	//参数化字符
	sscanf(Show_Num,"%f",&Motor_Control_Parm.Accelerated_Speed);
}

/** @brief	四级电机转速PID显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level4_Motor_Speed_PID(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_Printf(0,0,OLED_8X16,"Kp: %.2f",PID_Motor1.Kp);
	OLED_Printf(0,16,OLED_8X16,"Ki: %.2f",PID_Motor1.Ki);
	OLED_Printf(0,32,OLED_8X16,"Kd: %.2f",PID_Motor1.Kd);
}

/** @brief	四级巡线PID显示
  * @param	KEY2		按键2
  * @note	通过该函数，用户可使用板载按键与OLED屏幕实现查看系统部分参数以及对系统执行部分控制
  **/
void Menu_Level4_Line_Patrol_PID(KEY_Tigger_State KEY2)
{
	//按键2按下
	if(KEY2)
	{
		//菜单等级-1
		Menu_Parm.Menu_Level--;
	}

	//菜单显示
	OLED_Printf(0,0,OLED_8X16,"Kp: %.2f",Line_Patrol_PID.Kp);
	OLED_Printf(0,16,OLED_8X16,"Ki: %.2f",Line_Patrol_PID.Ki);
	OLED_Printf(0,32,OLED_8X16,"Kd: %.2f",Line_Patrol_PID.Kd);
}

/** @brief	Flash写数据显示与操作
  * @note	调用该函数后，将对flash进行写入操作，并在屏幕上同步显示状态
  **/
void OLED_Show_And_Set_Write_Flash(void)
{
	//清除显示缓存区
	OLED_Clear();
	//显示信息
	OLED_ShowString(0,16,"存储参数中。。。",OLED_8X16);
	//刷新屏幕
	OLED_Update();	
	//清除显示缓存区
	OLED_Clear();
	//延时500ms
	osDelay(500);
	
	//写入数据至flash中
	if(Flash_Write() == HAL_OK)
		OLED_ShowString(0,16,"Flash写入成功",OLED_8X16);
	else
		OLED_ShowString(0,16,"Flash写入失败",OLED_8X16);
	//刷新屏幕
	OLED_Update();	
	//清除显示缓存区
	OLED_Clear();
	//延时1500ms
	osDelay(1500);
}


/** @brief	OLED硬件故障报错显示
  * @note	若出现硬件故障，在屏幕上进行显示，并同步开启PD3指示灯
  * @note	该函数在"stm32f4xx_it.c"文件中，由HardFault_Handler中断函数调用
  **/
void OLED_HardFault_Error_Message_Show(void)
{
	PD3_LED_Control(LED_ON);							//开启PD3
	OLED_ShowString(1,1,"HardFault_Error",OLED_8X16);	//输出报错故障码
	OLED_Update();										//刷新屏幕
}
