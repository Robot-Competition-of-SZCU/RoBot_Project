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
#include "cmsis_os.h"

//视觉识别单包数据最大长度
#define VISUAL_IDENTITY_MAX_LEN	64

//视觉识别数据包结构体
typedef struct{
	uint16_t Length;		//数据长度
	uint8_t Data[VISUAL_IDENTITY_MAX_LEN];	//接收数据
}Visual_Identity_Packet;

extern char Debug_Receive_Buffer[66];	//接收数据缓存区
extern uint8_t Visual_Identity_RX_Buffer[VISUAL_IDENTITY_MAX_LEN];	//视觉识别接收缓冲区
extern osMessageQueueId_t Visual_Identity_QueueHandle;	//视觉识别消息队列
extern Visual_Identity_Packet Packet;				//视觉识别数据结构体


void UART_Receive_Init(void);									//串口接收数据初始化
void UART_Debug_Send_Date(char* Data_Address,short Length);		//调试串口发送数据

void UART_Visual_Identity_Init(void);					//视觉识别串口接收初始化
void UART_Visual_Identity_IDLE_Handle(void);			//视觉识别串口空闲中断处理
void Visual_Identity_Data_Analysis(Visual_Identity_Packet* Packet);	//视觉识别数据解析
void UART_Visual_Identity_Send_Order(char Order);		//视觉识别发送命令


void UART_Audio_Loud_Control(uint8_t Audio_Loud);		//音频串口音量控制
void UART_Audio_Control(uint16_t Audio_Select);			//音频串口播放控制
void UART_Audio_Mode_Control(void);						//音频串口播放模式


#endif
