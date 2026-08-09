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

extern char Debug_Receive_Buffer[66];	//接收数据缓存区

void Jinqu_Play(int num);
void UART_Receive_Init(void);									//串口接收数据初始化
void UART_Debug_Send_Date(char* Data_Address,short Length);		//调试串口发送数据

#endif
