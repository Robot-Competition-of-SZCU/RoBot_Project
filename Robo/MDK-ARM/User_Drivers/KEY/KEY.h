///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 8th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为按键控制读取底层驱动头文件
///////////////////////////////////////
#ifndef __KEY_H__
#define __KEY_H__

//按键状态
typedef enum
{
	NO_Tigger = 0,	//无触发
	short_Tigger,	//短按触发
	long_Tigger		//长按触发
}KEY_Tigger_State;

//按键键码
typedef enum
{
	KEY1 = 0,		//按键1
	KEY2,			//按键2
	KEY3,			//按键3
	KEY4,			//按键4
	Rocker_UP,		//摇杆_上
	Rocker_Right,	//摇杆_右
	Rocker_Down,	//摇杆_下
	Rocker_Middle,	//摇杆_中
	Rocker_Left		//摇杆_左
}KEY_Encode;

void KEY_Polling_Function(void);					//按键轮询函数，轮询间隔为20ms
KEY_Tigger_State Get_Key_State(KEY_Encode KEY_X);	//获取按键状态
void Key_State_Clear(void);							//复位所有键码状态


#endif
