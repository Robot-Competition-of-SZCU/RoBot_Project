///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为任务设置头文件
///////////////////////////////////////
#ifndef __User_Task_H__
#define __User_Task_H__

#include "cmsis_os.h"

//各任务信息存储区
extern TaskStatus_t xTaskDetails_defaultTask;
extern TaskStatus_t xTaskDetails_RUN_LED_Flash;
extern TaskStatus_t xTaskDetails_KEY_Scan;
extern TaskStatus_t xTaskDetails_State_LED_Flash;
extern TaskStatus_t xTaskDetails_Set_And_Show;
extern TaskStatus_t xTaskDetails_Slow_Compute;
extern TaskStatus_t xTaskDetails_High_Compute;
extern TaskStatus_t xTaskDetails_UART_Debug;


void User_Init(void);					//总初始化函数

void RUN_LED_Flash_Task(void *argument);		//运行指示灯闪烁任务
void State_LED_Flash_Task(void *argument);
void KEY_Scan_Task(void *argument);				//按键扫描任务
void OLED_Show_Task(void *argument);			//OLED显示任务

#endif
