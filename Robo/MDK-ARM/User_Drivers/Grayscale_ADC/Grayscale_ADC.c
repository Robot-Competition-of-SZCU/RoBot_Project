///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 14th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为灰度ADC采集源文件
///////////////////////////////////////
#include "Grayscale_ADC.h"
#include "stm32f4xx_hal.h"

#include "adc.h"
#include "tim.h"

#include "Control.h"

//灰度传感器ADC数据缓存区
short Grayscale_ADC_Buffer[16];

//灰度传感器ADC参数结构体
struct Grayscale_ADC Grayscale;


/** @brief	灰度传感器ADC采集初始化
  * @note	ADC1与ADC2同步采集,TIM5CH1通道上升沿信号作为ADC1外部触发时钟
  **/
void Grayscale_ADC_Init(void)
{
	//开启ADC1与ADC2
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc1); 
	//启动ADC双通道同步采集
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)Grayscale_ADC_Buffer, 8);
	
	//开启TIM5作为ADC触发时钟
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
}

/** @brief	灰度传感器ADC校准，使用均值校准
  * @param	Mode	校准模式
  * @retval	校准是否完成	
  **/
HAL_StatusTypeDef Grayscale_ADC_Calibration(Calibration_Mode Mode)
{
	static int Calibration_Buffer[Grayscale_Num];	//数据累计缓存区
	static int Sampling_Num = 0;		//采样次数
	
	if(Mode == Calibration_Max)
	{
		if(Sampling_Num < Calibrate_Sampling_Num)
		{
			//累加数据
			for(char i=0;i<Grayscale_Num;i++)
				Calibration_Buffer[i] += Grayscale_ADC_Buffer[i];
			//采样次数+1
			Sampling_Num++;
		}
		//达到设定采样次数
		else
		{
			//求均值并搬运
			for(char i=0;i<Grayscale_Num;i++)
				Grayscale.Grayscale_ADC_Max[i] = Calibration_Buffer[i] / Calibrate_Sampling_Num;
			//完成校准
			goto Finish;
		}
	}
	else if(Mode == Calibration_Min)
	{
		if(Sampling_Num < Calibrate_Sampling_Num)
		{
			//累加数据
			for(char i=0;i<Grayscale_Num;i++)
				Calibration_Buffer[i] += Grayscale_ADC_Buffer[i];
			//采样次数+1
			Sampling_Num++;
		}
		//达到设定采样次数
		else
		{
			//求均值并搬运
			for(char i=0;i<Grayscale_Num;i++)
				Grayscale.Grayscale_ADC_Min[i] = Calibration_Buffer[i] / Calibrate_Sampling_Num;
			//完成校准
			goto Finish;
		}
	}
	
	//返回忙
		return HAL_BUSY;
	
	Finish:
	//复位数据缓存及状态
	for(char i=0;i<Grayscale_Num;i++)
		Calibration_Buffer[i] = 0;
	Sampling_Num = 0;
	
	//返回完成
		return HAL_OK;
}


/** @brief	灰度传感器ADC解算
  **/
void Grayscale_ADC_Compute(void)
{
	//数据搬运
	for(char i=0;i<Grayscale_Num;i++)
		Grayscale.Grayscale_ADC_Actual[i] = Grayscale_ADC_Buffer[i];
	
	//计算灰度百分比
	//正向计算模式
	if(Grayscale.Compute_Map_Mode == 0)
	{	//计算校准值
		for(char i=0;i<Grayscale_Num;i++)
		{	//校准值 = 实际值 - 最小极限值
			Grayscale.Grayscale_ADC_Compute[i] = Grayscale.Grayscale_ADC_Actual[i] - Grayscale.Grayscale_ADC_Min[i];
			//防溢出处理
			if(Grayscale.Grayscale_ADC_Compute[i] < 0) Grayscale.Grayscale_ADC_Compute[i] = 0;
			if(Grayscale.Grayscale_ADC_Compute[i] > (Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i]))
				Grayscale.Grayscale_ADC_Compute[i] = Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i];
		}
		//计算校准后参数百分比
		for(char i=0;i<Grayscale_Num;i++)
		{	//参数百分比 = (实际值 / (最大极限值 - 最小极限值)) * 100
			Grayscale.Grayscale_ADC_Compute_Percent[i] = (float)(Grayscale.Grayscale_ADC_Compute[i]
														/(float)(Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i]))
														* 100.0f;
		}
	}
	//反向计算模式
	else if(Grayscale.Compute_Map_Mode == 1)
	{	//计算校准值
		for(char i=0;i<Grayscale_Num;i++)
		{	//校准值 =最大极限值 - 实际值
			Grayscale.Grayscale_ADC_Compute[i] = Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Actual[i];
			//防溢出处理
			if(Grayscale.Grayscale_ADC_Compute[i] < 0) Grayscale.Grayscale_ADC_Compute[i] = 0;
			if(Grayscale.Grayscale_ADC_Compute[i] > (Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i]))
				Grayscale.Grayscale_ADC_Compute[i] = Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i];
		}
		//计算校准后参数百分比
		for(char i=0;i<Grayscale_Num;i++)
		{	//参数百分比 = (实际值 / (最大极限值 - 最小极限值)) * 100
			Grayscale.Grayscale_ADC_Compute_Percent[i] = ((float)Grayscale.Grayscale_ADC_Compute[i]
														/(float)(Grayscale.Grayscale_ADC_Max[i] - Grayscale.Grayscale_ADC_Min[i]))
														* 100.0f;
		}
	}

	//计算触发状态
	for(char i=0;i<Grayscale_Num;i++)
		if(Grayscale.Grayscale_ADC_Compute_Percent[i] >= Grayscale.Grayscale_ADC_Trigger_Threshold)	
			Grayscale.Grayscale_Trigger_State[i] = 1;
		else
			Grayscale.Grayscale_Trigger_State[i] = 0;

	//计算映射一维坐标值
	float Grayscale_Weight=0;	//灰度权重值
	float Grayscale_Int=0;		//灰度总值

	//根据Grayscale_Num自适应计算各传感器偏移权重并累计
	//传感器权重 = (传感器索引 - 中心位置) * Grayscale_Weight_Bias，中心位置 = (Grayscale_Num-1)/2
	//仅需修改Grayscale_Num或Grayscale_Weight_Bias宏即可适配不同数量的灰度传感器，无需改动本函数
	for(char i=0;i<Grayscale_Num;i++)
	{
		Grayscale_Weight += Grayscale.Grayscale_ADC_Compute_Percent[i]
							* ((float)i - (Grayscale_Num - 1) / 2.0f)
							* Grayscale_Weight_Bias;
		//累计总值
		Grayscale_Int  += Grayscale.Grayscale_ADC_Compute_Percent[i];
	}

	//计算映射值（防止除零）
	if(Grayscale_Int != 0)
		Grayscale.Grayscale_Map = Grayscale_Weight / Grayscale_Int;
	else
		Grayscale.Grayscale_Map = 0;

	//运行状态下
	if(RUN_Parm.RUN_State)
	{
		char Trigger_Int = 0;
		for(char i=0;i<Grayscale_Num;i++)
		Trigger_Int += Grayscale.Grayscale_Trigger_State[i];

		if(Trigger_Int == 0 || Trigger_Int >= 5)
			RUN_Parm.RUN_State = 0;
	}

}



