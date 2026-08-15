///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为编码器采集源文件
///////////////////////////////////////
#include "Encoder.h"
#include "stm32f4xx_hal.h"

#include "tim.h"
#include "dma.h"
#include "main.h"

#include "PID.h"


struct Encoder	Encoder1;
struct Encoder	Encoder2;
struct Encoder	Encoder3;
struct Encoder	Encoder4;

char Encoder4_B_TIM4_CH4_Data_Pointer;	//编码器4通道B（TIM4_通道4）CCR数据传输伪指针

uint16_t CCR_Buffer[8];		//编码器CCR缓存区
uint16_t CCR_Input_Pulse[8];//CCR触发脉冲计数

/** @brief	编码器初始化
  * @note	开启输入捕获与DMA传输
  * @note	由于STM32F407硬件上不支持TIM4_CH4输入捕获链接DMA传输，故该通道输入捕获改为中断读取，具体详见"stm32f4xx.it.c"，中断函数TIM4_IRQHandler
  **/
void Encoder_Init(void)
{
	HAL_TIM_Base_Start(&htim4);		//开启TIM4
	HAL_TIM_Base_Start(&htim8);		//开启TIM8
	
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);	//开启TIM4_CH4通道输入捕获中断
	
	HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t*)&CCR_Buffer[4],65535);	//开启TIM4_CH1通道DMA传输
	HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_2,(uint32_t*)&CCR_Buffer[5],65535);	//开启TIM4_CH2通道DMA传输
	HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_3,(uint32_t*)&CCR_Buffer[6],65535);	//开启TIM4_CH3通道DMA传输
	
	//因STM32F407硬件上不支持TIM4_CH4输入捕获链接DMA传输，故该项无效
	//HAL_TIM_IC_Start_DMA(&htim4,TIM_CHANNEL_4,(uint32_t*)&CCR_Buffer[7],65535);	//开启TIM4_CH4通道DMA传输
	
	HAL_TIM_IC_Start_DMA(&htim8,TIM_CHANNEL_1,(uint32_t*)&CCR_Buffer[0],65535);	//开启TIM8_CH1通道DMA传输
	HAL_TIM_IC_Start_DMA(&htim8,TIM_CHANNEL_2,(uint32_t*)&CCR_Buffer[1],65535);	//开启TIM8_CH2通道DMA传输
	HAL_TIM_IC_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t*)&CCR_Buffer[2],65535);	//开启TIM8_CH3通道DMA传输
	HAL_TIM_IC_Start_DMA(&htim8,TIM_CHANNEL_4,(uint32_t*)&CCR_Buffer[3],65535);	//开启TIM8_CH4通道DMA传输
}

/** @brief	编码器输入频率计算
  * @note	采样M法测速，即在一个时间窗格内记录输入的脉冲数量
  * @note	该函数调控周期为5ms
  **/
void Encoder_Compute(void)
{
	char i=0;
	float Frequiency[8];
	
	static uint16_t Last_DMA_CNDTR[8],Now_DMA_CNDTR[8];
	
	//转移上一次DMA剩余传输记录器值
	for(i=0;i<8;i++)
		Last_DMA_CNDTR[i] = Now_DMA_CNDTR[i];
	
	//获取当前DMA剩余传输记录器值
	Now_DMA_CNDTR[0] = __HAL_DMA_GET_COUNTER(&hdma_tim8_ch1);
	Now_DMA_CNDTR[1] = __HAL_DMA_GET_COUNTER(&hdma_tim8_ch2);
	Now_DMA_CNDTR[2] = __HAL_DMA_GET_COUNTER(&hdma_tim8_ch3);
	Now_DMA_CNDTR[3] = __HAL_DMA_GET_COUNTER(&hdma_tim8_ch4_trig_com);
	
	Now_DMA_CNDTR[4] = __HAL_DMA_GET_COUNTER(&hdma_tim4_ch1);
	Now_DMA_CNDTR[5] = __HAL_DMA_GET_COUNTER(&hdma_tim4_ch2);
	Now_DMA_CNDTR[6] = __HAL_DMA_GET_COUNTER(&hdma_tim4_ch3);
	//因STM32F407硬件上不支持TIM4_CH4输入捕获链接DMA传输，故该项无效
	//Now_DMA_CNDTR[7] = __HAL_DMA_GET_COUNTER(&hdma_tim4_ch4);
	
	//计算差值，可得到5ms内DMA传输次数，即输入捕获触发次数
	for(i=0;i<7;i++)
		CCR_Input_Pulse[i] = Last_DMA_CNDTR[i] - Now_DMA_CNDTR[i];
	//TIM4_CH4输入捕获计数已在中断中单独计算
		
	//计算频率值
	for(i=0;i<8;i++)
		Frequiency[i] = CCR_Input_Pulse[i] / Input_Capture_Cycle;
		
	//复位差值
	CCR_Input_Pulse[7] = 0;
		
	//装载触发数
	Encoder1.Pulse = CCR_Input_Pulse[0] + CCR_Input_Pulse[1];
	Encoder2.Pulse = CCR_Input_Pulse[2] + CCR_Input_Pulse[3];
	Encoder3.Pulse = CCR_Input_Pulse[4] + CCR_Input_Pulse[5];
	Encoder4.Pulse = CCR_Input_Pulse[6] + CCR_Input_Pulse[7];
	
	//累计触发数
	Encoder1.Pulse_Int += Encoder1.Pulse;
	Encoder2.Pulse_Int += Encoder2.Pulse;
	Encoder3.Pulse_Int += Encoder3.Pulse;
	Encoder4.Pulse_Int += Encoder4.Pulse;
	
	//装载频率
	Encoder1.Actual_Frequiency = Frequiency[0] + Frequiency[1];
	Encoder2.Actual_Frequiency = Frequiency[2] + Frequiency[3];
	Encoder3.Actual_Frequiency = Frequiency[4] + Frequiency[5];
	Encoder4.Actual_Frequiency = Frequiency[6] + Frequiency[7];
	
	//静态偏差消除
	//无输出或弱输出时，编码器会因轮子抖动引入偏差，通过以下判断进行消除
	if(PID_Motor1.Out <= 5)
		Encoder1.Actual_Frequiency = 0;
	if(PID_Motor2.Out <= 5)
		Encoder2.Actual_Frequiency = 0;
	if(PID_Motor3.Out <= 5)
		Encoder3.Actual_Frequiency = 0;
	if(PID_Motor4.Out <= 5)
		Encoder4.Actual_Frequiency = 0;
}


/** @brief	编码器输入频率滤波，使用一阶低通滤波
  * @note	滤波值 = 滤波系数 * 实际值 + （1-滤波系数） * 上一滤波值
  **/
void Encoder_Input_Frequiency_Filter(void)
{
	Encoder1.Filter_Frequiency = Encoder_Filter_Alpha * Encoder1.Actual_Frequiency + (1-Encoder_Filter_Alpha) * Encoder1.Filter_Frequiency;
	Encoder2.Filter_Frequiency = Encoder_Filter_Alpha * Encoder2.Actual_Frequiency + (1-Encoder_Filter_Alpha) * Encoder2.Filter_Frequiency;
	Encoder3.Filter_Frequiency = Encoder_Filter_Alpha * Encoder3.Actual_Frequiency + (1-Encoder_Filter_Alpha) * Encoder3.Filter_Frequiency;
	Encoder4.Filter_Frequiency = Encoder_Filter_Alpha * Encoder4.Actual_Frequiency + (1-Encoder_Filter_Alpha) * Encoder4.Filter_Frequiency;
}

/** @brief	编码器速度计算
  * @note	滤波值
  **/
void Encoder_Speed_Compute(void)
{
	//转速计算：转速 = 编码器滤波后频率值 / 编码器分辨率 / 电机减速比
	Encoder1.RMP_S = Encoder1.Filter_Frequiency / Encoder_Resolution / Motor_Reduction_Ratio;
	Encoder2.RMP_S = Encoder2.Filter_Frequiency / Encoder_Resolution / Motor_Reduction_Ratio;
	Encoder3.RMP_S = Encoder3.Filter_Frequiency / Encoder_Resolution / Motor_Reduction_Ratio;
	Encoder4.RMP_S = Encoder4.Filter_Frequiency / Encoder_Resolution / Motor_Reduction_Ratio;
	
	//线速度计算：线速度 = 转速 * 轮子周长
	Encoder1.Speed = Encoder1.RMP_S * Length_Of_Each_Circle;
	Encoder2.Speed = Encoder2.RMP_S * Length_Of_Each_Circle;
	Encoder3.Speed = Encoder3.RMP_S * Length_Of_Each_Circle;
	Encoder4.Speed = Encoder4.RMP_S * Length_Of_Each_Circle;
	
	//累计旋转圈数
	Encoder1.RMP_Int += Encoder1.RMP_S * Input_Capture_Cycle;
	Encoder2.RMP_Int += Encoder2.RMP_S * Input_Capture_Cycle;
	Encoder3.RMP_Int += Encoder3.RMP_S * Input_Capture_Cycle;
	Encoder4.RMP_Int += Encoder4.RMP_S * Input_Capture_Cycle;
	
	//累计行进距离
	Encoder1.RUN_Int += Encoder1.Speed * Input_Capture_Cycle;
	Encoder2.RUN_Int += Encoder2.Speed * Input_Capture_Cycle;
	Encoder3.RUN_Int += Encoder3.Speed * Input_Capture_Cycle;
	Encoder4.RUN_Int += Encoder4.Speed * Input_Capture_Cycle;
}


