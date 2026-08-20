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
#include "Control.h"

char Debug_Send_Buffer[66];		//发送数据缓存区	
char Debug_Receive_Buffer[66];	//接收数据缓存区

char Audio_Control_Send_Buffer[64];	//音频控制发送数据缓存区

uint8_t Visual_Identity_RX_Buffer[VISUAL_IDENTITY_MAX_LEN];	//视觉识别串口接收缓冲区
static uint8_t UART6_RX_Started = 0;		//视觉识别串口接收启动标志
Visual_Identity_Packet Packet;				//视觉识别数据结构体

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
	//开启视觉识别串口DMA接收
	UART_Visual_Identity_Init();
	
}

/** @brief	视觉识别串口接收初始化
  * @note	启动USART6的DMA接收，单包最大长度为32字节
  **/
void UART_Visual_Identity_Init(void)
{
	//清空接收缓冲区
	memset(Visual_Identity_RX_Buffer,0,VISUAL_IDENTITY_MAX_LEN);
	//清除串口空闲标志
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	//开启USART6接收与DMA传输
	HAL_UART_Receive_DMA(&huart6,Visual_Identity_RX_Buffer,VISUAL_IDENTITY_MAX_LEN);
	//使能串口空闲中断，用于判断一帧数据接收完成
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	//置接收启动标志
	UART6_RX_Started = 1;
}

/** @brief	视觉识别串口空闲中断处理
  * @note	串口接收完一帧数据后触发空闲中断，在此计算包长并发送消息队列
  * @note	数据包长度 = 缓冲区长度 - DMA剩余搬运次数
  **/
void UART_Visual_Identity_IDLE_Handle(void)
{
	//接收未启动时不处理
	if(UART6_RX_Started == 0)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		return;
	}

	//停止DMA与串口DMA请求，锁定当前接收进度
	HAL_UART_DMAStop(&huart6);

	//计算本次接收的数据包长度
	uint16_t Length = VISUAL_IDENTITY_MAX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

	//长度合法判断
	if(Length > 0 && Length <= VISUAL_IDENTITY_MAX_LEN)
	{
		//数据打包
		Visual_Identity_Packet Packet;
		Packet.Length = Length;
		memcpy(Packet.Data,Visual_Identity_RX_Buffer,Length);
		//发送至消息队列，唤醒数据解析任务，中断中调用超时参数必须为0
		osMessageQueuePut(Visual_Identity_QueueHandle,&Packet,0U,0U);
	}

	//重启DMA接收，不影响下一次接收
	HAL_UART_Receive_DMA(&huart6,Visual_Identity_RX_Buffer,VISUAL_IDENTITY_MAX_LEN);

	//清除串口空闲标志
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
}

/** @brief	视觉识别数据解析
  * @param	Packet	数据包
  * @note	在视觉识别任务中调用，在此按协议解析数据包内容
  **/
void Visual_Identity_Data_Analysis(Visual_Identity_Packet* Packet)
{
	//数据长度合法判断
	if(Packet->Length == 0) return;

	static char Test_Buffer[3] = {0,1,2},Test_Buffer_Pointer = 0;

	//文字识别状态
	if(RUN_Parm.Visual_Identity_Word_Flag == 1)
	{
		//存储一次当前数据
		Test_Buffer[Test_Buffer_Pointer] = Packet->Data[6];
		Test_Buffer_Pointer++;
		Test_Buffer_Pointer%=3;

		//连续三次识别数据一致，判定为有效数据
		if(Test_Buffer[0] == Test_Buffer[1] && Test_Buffer[1] == Test_Buffer[2])
		{
			//传递数据
			RUN_Parm.Word_Test = Test_Buffer[0];
			//复位缓存区
			Test_Buffer[0] = 0;
			Test_Buffer[1] = 1;
			Test_Buffer[2] = 2;
			//识别结束
			RUN_Parm.Visual_Identity_Word_Flag = 0;
		}
	}
	//颜色识别状态
	else if(RUN_Parm.Visual_Identity_Colour_Flag == 1)
	{
		//存储一次当前数据
		Test_Buffer[Test_Buffer_Pointer] = Packet->Data[6];
		Test_Buffer_Pointer++;
		Test_Buffer_Pointer%=3;

		//连续三次识别数据一致，判定为有效数据
		if(Test_Buffer[0] == Test_Buffer[1] && Test_Buffer[1] == Test_Buffer[2])
		{
			//传递数据
			RUN_Parm.Colour_Test = Test_Buffer[0];
			//复位缓存区
			Test_Buffer[0] = 0;
			Test_Buffer[1] = 1;
			Test_Buffer[2] = 2;
			//识别结束
			RUN_Parm.Visual_Identity_Colour_Flag = 0;
		}
	}
}

/** @brief	视觉识别发送命令
  * @param	Order	视觉控制命令
  **/
void UART_Visual_Identity_Send_Order(char Order)
{
	static char Order_Buffer;
	Order_Buffer = Order;

	//调用DMA进行数据发送
	HAL_UART_Transmit_DMA(&huart6,(uint8_t*)&Order_Buffer,1);
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

