///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为中断任务头文件
///////////////////////////////////////
#ifndef __Interrupt_H__
#define __Interrupt_H__

#include "cmsis_os.h"

/* GPIO中断触发消息队列句柄(在freertos.c中创建) */
extern osMessageQueueId_t GPIO_Tigger_StateHandle;

void USART6_IDLE_Interrupt_Callback(void);	//串口空闲中断回调，由USART6中断服务函数调用

#endif

