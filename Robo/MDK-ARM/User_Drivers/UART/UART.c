///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为串口通讯底层驱动源文件
///////////////////////////////////////
#include "UART.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"

#include "usart.h"
#include "dma.h"

char Debug_Send_Buffer[66];		//发送数据缓存区	
char Debug_Receive_Buffer[66];	//接收数据缓存区


//景区播放发送数据，五岳顺序（东南西北中）
const char Jinqu_Buffer[5][14] ={
{FD,00,0B,01,01,B6,AB,D4,C0,CC,A9,C9,BD,EE},
{FD,00,0B,01,01,CE,F7,D4,C0,BB,AA,C9,BD,BE},
{FD,00,0B,01,01,C4,CF,D4,C0,BA,E2,C9,BD,C5},
{FD,00,0B,01,01,B1,B1,D4,C0,BA,E3,C9,BD,CF},
{FD,00,0B,01,01,D6,D0,D4,C0,CB,C9,C9,BD,92}   
};

/** @brief	景区五岳播放
  **/
void Jinqu_Play(int num)
{
	short i;

	//等待UART外设空闲,若100ms内未等到，则发送失败，返回
	for(char i=0;i<100;i++)
	{
		if(__HAL_DMA_GET_COUNTER(&hdma_usart3_tx) == 0)
			goto Send;
		osDelay(1);
	}
	return;
	
		Send:
	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)Jinqu_Buffer[num],14);
};

/** @brief	串口接收数据初始化
  **/
void UART_Receive_Init(void)
{
	//清空接收数据缓存区
	for(char i=0;i<66;i++)
		Debug_Receive_Buffer[i] = 0;
	//开启UART3接收与DMA传输
	HAL_UART_Receive_DMA(&huart3,(uint8_t*)Debug_Receive_Buffer,19);
}

/** @brief	调试串口接收数据解析
  **/
void UART_Debug_Receive_Analysis(void)
{
	
}

/** @brief	调试串口发送数据
  * @param	Data_Address	数据起始地址
  * @retval	Length			数据长度
  * @note	该函数在原本数据上添加包头0XA5，包尾0X5A，后以数据包格式发送
  * @note	所发送的数据长度不应超过64字节
  **/
void UART_Debug_Send_Date(char* Data_Address,short Length)
{
	short i;
	
	if(Length>=64)	Length=64;	//长度限位
	
	//等待UART外设空闲,若100ms内未等到，则发送失败，返回
	for(char i=0;i<100;i++)
	{
		if(__HAL_DMA_GET_COUNTER(&hdma_usart3_tx) == 0)
			goto Send;
		osDelay(1);
	}
	return;
	
		Send:
		
	Debug_Send_Buffer[0] = 0XA5;					//包头数据
	
	for(i=0;i<(Length);i++)
		Debug_Send_Buffer[i+1] = Data_Address[i];	//实际数据
	
	Debug_Send_Buffer[i+1] = 0X5A;					//包尾数据
	
	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)Debug_Send_Buffer,Length+2);
}

