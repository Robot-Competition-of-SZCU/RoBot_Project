///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为设置与显示头文件
///////////////////////////////////////
#ifndef __Set_And_Show_H__
#define __Set_And_Show_H__

#include "KEY.h"
/*
设置/显示菜单定义，文字为菜单名称，数字为菜单界面码

一级菜单+-二级运行设置菜单--+-开始运行
		|					|
		|					+-运行设置
		|
		+-二级系统设置菜单--+-三级速度设置菜单--+-基础速度设置
		|					|					+-加速度设置
		|					|					+-转向速度设置
		|					|					+-参数存储
		|					|						
		|					+-三级PID设置菜单---+-轮子转速环PID
		|					|					+-巡线调控环PID
		|					|
		|					+-三级编码器参数查看菜单--+-实时脉冲数
		|					|						+-脉冲累计
		|					|						+-实时旋转速度
		|					|						+-旋转速度累计
		|					|						+-实时线速度
		|					|						+-线速度累计
		|					|
		|					+-三级IMU数据查看
		|
		+-二级巡线设置菜单--+-三级灰度参数查看菜单--+-校准值显示
		|					|						+-实际值显示
		|					|							
		|					+-三级校准设置菜单--+-最大极限值校准
		|					|					+-最小极限值校准
		|					|
		|					+-计算模式设置
		|					+-触发阈值设置
		|					+-参数存储
		|						
		+-二级舵机设置菜单--+-三级舵机直接控制菜单--+-舵机选择
		|											+-舵机状态设置
		|											+-舵机方向设置
		|											+-舵机角度设置
		|											+-舵机最大极限角度设置
		|											+-舵机最小极限角度设置
		|											+-参数存储
		|
		+-二级任务设置窗口--+-三级运行指示灯任务查看(30)
							+-三级按键扫描任务查看(31)
							+-三级设置与显示任务查看(32)
							+-三级串口调试任务查看(33)
							+-三级低速计算任务查看(34)
							+-三级高速计算任务查看(35)
*/

//菜单等级枚举
typedef enum{
	Level1 = 0,
	Level2,
	Level3,
	Level4,
	Level5,
hghghfghfghfghfghfghfghfghfhfghfhfghfgh}Menu_Level_Set;

//二级菜单界面码枚举
typedef enum{
	RUN_Set = 0,	//运行设置
	System_Set,		//系统设置菜单
	Scan_Line_Set,	//巡线参数设置菜单
	Servo_Set,		//舵机设置菜单
	Task_Show,		//任务查看菜单	
}Menu_IF_L2;

//三级系统设置菜单
typedef enum{
	Speed_Set = 0,	//速度设置
	PID_Set,		//PID设置
	Encoder_View,	//编码器查看
	IMU_View,		//IMU数据查看
}Menu_System_Set_L3;

//三级巡线参数设置菜单
typedef enum{
	Gray_View = 0,	//灰度参数查看
	Gray_Calib,		//校准设置
}Menu_Scan_Line_Set_L3;

//三级舵机设置菜单
typedef enum{
	Servo_Set_ = 0,	//舵机控制
}Menu_Servo_Set_L3;

//三级任务查看菜单
typedef enum{
	Task_LED = 0,	//运行指示灯任务
	Task_KEY_Sacn,	//按键扫描任务
	Task_Show_Set,	//设置与显示任务
	Task_UART_Debug,//串口调试任务
	Task_Slow_Com,	//低速计算任务
	Task_High_Com,	//高速计算任务	
}Menu_Task_Show_L3;

typedef enum{
	Base_Speed_Set = 0,		//基础速度设置
	Accelerated_Speed_Set,	//加速度设置
	Turn_Speed_Set			//转向速度设置
}Menu_Speed_Set_L4;

//四级PID设置菜单
typedef enum{
	Motor_Speed_PID = 0,//电机转速PID
	Line_P_PID			//巡线PID
}Menu_PID_Set_L4;

//系统菜单结构体
struct Menu{
	Menu_Level_Set Menu_Level;		//菜单当前等级
	int Menu_Control_Pointer;		//菜单控制指针
	int Menu_Interface_L2;			//二级菜单界面码
	int Menu_Interface_L3;			//三级菜单界面码
	int Menu_Interface_L4;			//四级菜单界面码
};
extern struct Menu Menu_Parm;


void System_Menu_Control_And_Show(void);		//系统菜单控制与显示

//一级主菜单控制与显示
void Menu_Level1_Control(KEY_Tigger_State KEY1,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);

//二级运行设置菜单控制与显示
void Menu_Level2_RUN_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//二级系统菜单控制与显示
void Menu_Level2_System_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//二级巡线菜单控制与显示
void Menu_Level2_Scan_Line_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//二级舵机设置菜单控制与显示
void Menu_Level2_Servo_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2);
//二级任务设置窗口控制与显示
void Menu_Level2_Task_Show(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);

//三级速度设置菜单控制与显示
void Menu_Level3_Speed_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//三级PID设置菜单控制与显示
void Menu_Level3_PID_Set(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//三级编码器参数显示
void Menu_Level3_Encoder_View(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);
//三级IMU数据显示
void Menu_Level3_IMU_View(KEY_Tigger_State KEY2);
//三级灰度参数查看控制与显示
void Menu_Level3_Gray_View(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left);
//三级灰度校准控制与显示
void Menu_Level3_Gray_Calib(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2);
//三级舵机控制与显示
void Menu_Level3_Servo_Control(KEY_Tigger_State KEY1,KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down);

void Menu_Level3_Task_LED(KEY_Tigger_State KEY2);			//三级运行指示灯任务显示
void Menu_Level3_Task_KEY_Sacn(KEY_Tigger_State KEY2);		//三级按键扫描任务显示
void Menu_Level3_Task_Show(KEY_Tigger_State KEY2);			//三级设置与显示任务显示
void Menu_Level3_Task_UART_Debug(KEY_Tigger_State KEY2);	//三级串口调试任务显示
void Menu_Level3_Task_Slow_Com(KEY_Tigger_State KEY2);		//三级低速计算任务显示
void Menu_Level3_Task_High_Com(KEY_Tigger_State KEY2);		//三级高速计算任务显示

//四级基础速度设置与显示
void Menu_Level4_Speed_Set(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left);			
//四级加速度设置与显示
void Menu_Level4_Accelerated_Speed_Set(KEY_Tigger_State KEY2,KEY_Tigger_State Rocker_UP,KEY_Tigger_State Rocker_Down,KEY_Tigger_State Rocker_Right,KEY_Tigger_State Rocker_Left);
void Menu_Level4_Motor_Speed_PID(KEY_Tigger_State KEY2);	//四级电机转速PID显示
void Menu_Level4_Line_Patrol_PID(KEY_Tigger_State KEY2);	//四级巡线PID显示

void OLED_Show_And_Set_Write_Flash(void);			//Flash写数据显示与操作
void OLED_HardFault_Error_Message_Show(void);		//OLED硬件故障报错显示

#endif

