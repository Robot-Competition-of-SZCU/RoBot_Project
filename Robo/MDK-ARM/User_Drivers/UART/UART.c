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
#include <string.h>

#include "usart.h"
#include "dma.h"

#include "IMU.h"

char Debug_Send_Buffer[66];		//发送数据缓存区	
char Debug_Receive_Buffer[66];	//接收数据缓存区

char Audio_Control_Send_Buffer[64];	//音频控制发送数据缓存区

/** @brief	串口接收数据初始化
  **/
void UART_Receive_Init(void)
{
	// //清空接收数据缓存区
	// for(char i=0;i<66;i++)
	// 	Debug_Receive_Buffer[i] = 0;
	//开启UART3接收与DMA传输
	HAL_UART_Receive_DMA(&huart3,(uint8_t*)Debug_Receive_Buffer,19);
	//开启IMU姿态数据解析
	HAL_UART_Receive_DMA(&huart4,(uint8_t*)IMU_data_Buffer,IMU_Buffer_Length);
	
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

/** @brief	音频串口音量控制
  * @param	Audio_Loud	音量大小，范围0-30
  **/
void UART_Audio_Loud_Control(uint8_t Audio_Loud)
{
	uint8_t CRC_ = 0;	//和校验
	
	Audio_Control_Send_Buffer[0] = 0X7E;	//起始码
	Audio_Control_Send_Buffer[1] = 0X08;	//帧长度
	Audio_Control_Send_Buffer[2] = 0XFF;	//设备地址(高字)
	Audio_Control_Send_Buffer[3] = 0XFF;	//设备地址(低字)
	Audio_Control_Send_Buffer[4] = 0X13;	//指令码
	
	//音量大小
	memcpy(&Audio_Control_Send_Buffer[5],&Audio_Loud,sizeof(Audio_Loud));
	
	//和校验计算
	for(char i=0;i<6;i++)
		CRC_ += Audio_Control_Send_Buffer[i];
	
	Audio_Control_Send_Buffer[6] = CRC_;	//和校验装填
	Audio_Control_Send_Buffer[7] = 0XEF;	//结束码
	
	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)Audio_Control_Send_Buffer,8);
}

/** @brief	音频串口播放控制
  * @param	Audio_Select	音频选择
  **/
void UART_Audio_Control(uint16_t Audio_Select)
{
	uint8_t CRC_ = 0;	//和校验
	
	Audio_Control_Send_Buffer[0] = 0X7E;	//起始码
	Audio_Control_Send_Buffer[1] = 0X09;	//帧长度
	Audio_Control_Send_Buffer[2] = 0XFF;	//设备地址(高字)
	Audio_Control_Send_Buffer[3] = 0XFF;	//设备地址(低字)
	Audio_Control_Send_Buffer[4] = 0X07;	//指令码
	
	//曲目名称
	Audio_Control_Send_Buffer[5] = Audio_Select>>8;
	Audio_Control_Send_Buffer[6] = Audio_Select;
	
	//和校验计算
	for(char i=0;i<7;i++)
		CRC_ += Audio_Control_Send_Buffer[i];
	
	Audio_Control_Send_Buffer[7] = CRC_;	//和校验装填
	Audio_Control_Send_Buffer[8] = 0XEF;	//结束码
	
	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)Audio_Control_Send_Buffer,9);
}

/** @brief	音频串口播放模式
  **/
void UART_Audio_Mode_Control(void)
{
	uint8_t CRC_ = 0;	//和校验
	
	Audio_Control_Send_Buffer[0] = 0X7E;	//起始码
	Audio_Control_Send_Buffer[1] = 0X0A;	//帧长度
	Audio_Control_Send_Buffer[2] = 0XFF;	//设备地址(高字)
	Audio_Control_Send_Buffer[3] = 0XFF;	//设备地址(低字)
	Audio_Control_Send_Buffer[4] = 0X18;	//指令码
	
	Audio_Control_Send_Buffer[5] = 0X00;	//无循环模式
	Audio_Control_Send_Buffer[6] = 0X00;	//无循环次数
	Audio_Control_Send_Buffer[7] = 0X00;	//无循环次数
	
	//和校验计算
	for(char i=0;i<8;i++)
		CRC_ += Audio_Control_Send_Buffer[i];
	
	Audio_Control_Send_Buffer[8] = CRC_;	//和校验装填
	Audio_Control_Send_Buffer[9] = 0XEF;	//结束码
	
	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart2,(uint8_t*)Audio_Control_Send_Buffer,10);
}

