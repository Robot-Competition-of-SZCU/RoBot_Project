///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为串口通讯底层驱动头文件
///////////////////////////////////////
#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx_hal.h"

extern char Debug_Receive_Buffer[66];	//接收数据缓存区


void UART_Receive_Init(void);									//串口接收数据初始化
void UART_Debug_Send_Date(char* Data_Address,short Length);		//调试串口发送数据

void UART_Audio_Loud_Control(uint8_t Audio_Loud);		//音频串口音量控制
void UART_Audio_Control(uint16_t Audio_Select);			//音频串口播放控制
void UART_Audio_Mode_Control(void);						//音频串口播放模式


#endif
