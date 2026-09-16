///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 9rd, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为灰度采集计算及巡线解析源文件
///////////////////////////////////////
#include "Grayscale_ADC.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#include "adc.h"
#include "tim.h"


//灰度传感器ADC数据缓存区
short Grayscale_ADC_Buffer[16];

//灰度传感器数据结构体
struct Grayscale_Parm Grayscale;
//灰度传感器控制结构体
struct Grayscale_Control Grayscale_Ctrl;
//灰度传感器输出结构体
struct Grayscale_OutPut Grayscale_Out;


/** @brief	灰度传感器采集初始化
  * @note	ADC1与ADC2同步采集,TIM5CH1通道上升沿信号作为ADC1外部触发时钟
  * @note	输入数据流为数字量时，则无需调用该初始化函数
  **/
void Grayscale_Init(void)
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
  * @note	输入数据流为数字量时，则无需调用该校准函数
  * @note	该函数调用一次即进行一次数据采样
  **/
Calibration_State Grayscale_ADC_Calibration(Calibration_Mode Mode)
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
		return Calibration_BUSY;
	
	Finish:
	//复位数据缓存及状态
	for(char i=0;i<Grayscale_Num;i++)
		Calibration_Buffer[i] = 0;
	Sampling_Num = 0;
	
	//返回完成
		return Calibration_OK;
}

/** @brief	灰度数据流处理
  **/
void Grayscale_Data_Stream_Processing(void)
{
	Grayscale_ADC_Compute();		//灰度传感器ADC解算
	Scan_Line_Segment_Compute();	//巡线不连续触发段统计计算
	Grayscale_ADC_Map_Compute();	//灰度传感器ADC映射计算
	Intersection_Recognition();		//路口识别
}

/** @brief	灰度传感器ADC解算
  * @note	根据计算模式计算其百分比值与触发状态
  **/
void Grayscale_ADC_Compute(void)
{
	//数据搬运
	memcpy(Grayscale.Grayscale_ADC_Actual, Grayscale_ADC_Buffer, sizeof(Grayscale_ADC_Buffer));

	//输入数据流为数字量
	if(Grayscale_Data_Stream_Type == 1)
	{
		//正向计算模式
		if(Grayscale_Ctrl.Compute_Map_Mode == 0)
		{	
			//计算参数百分比
			for(char i=0;i<Grayscale_Num;i++)
			{	//参数百分比 = 实际值 * 100
				Grayscale.Grayscale_ADC_Compute_Percent[i] = Grayscale.Grayscale_ADC_Actual[i] * 100.0f;
			}
		}
		//反向计算模式
		else if(Grayscale_Ctrl.Compute_Map_Mode == 1)
		{	
			//计算参数百分比
			for(char i=0;i<Grayscale_Num;i++)
			{	//参数百分比 = 实际值 * 100
				if(Grayscale.Grayscale_ADC_Actual[i])
					Grayscale.Grayscale_ADC_Compute_Percent[i] = 0;
				else
					Grayscale.Grayscale_ADC_Compute_Percent[i] = 100.0f;
			}
		}
	}
	//输入数据流为模拟量
	else if(Grayscale_Data_Stream_Type == 0)
	{
		//计算灰度百分比
		//正向计算模式
		if(Grayscale_Ctrl.Compute_Map_Mode == 0)
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
		else if(Grayscale_Ctrl.Compute_Map_Mode == 1)
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
	}

	//计算触发状态
	for(char i=0;i<Grayscale_Num;i++)
		if(Grayscale.Grayscale_ADC_Compute_Percent[i] >= Grayscale_Ctrl.Grayscale_ADC_Trigger_Threshold)	
			Grayscale_Out.Grayscale_Trigger_State[i] = 1;
		else
			Grayscale_Out.Grayscale_Trigger_State[i] = 0;

	//计算传感器总共被触发的数量
	Grayscale_Out.Grayscale_Trigger_Num = 0;
	for(char i=0;i<Grayscale_Num;i++)
		Grayscale_Out.Grayscale_Trigger_Num += Grayscale_Out.Grayscale_Trigger_State[i];
}

//不连续触发段数量统计，并记录各线段起止索引
char Segment_Start[Grayscale_Num];	//各线段起始传感器索引
char Segment_End[Grayscale_Num];	//各线段结束传感器索引
float Segment_Map[Grayscale_Num];	//各线段的映射坐标值

/** @brief	巡线不连续触发段统计计算
  **/
void Scan_Line_Segment_Compute(void)
{
	//复位线段数量统计
	Grayscale_Out.Number_Of_Discontinuous_Points = 0;

	char Trigger_Continuous_State = 0;			//触发连续状态，0不连续，1连续

	//线段检测：以触发状态上升沿作为新线段开始，下降沿作为线段结束
	for(char i=0;i<Grayscale_Num;i++)
	{
		//存在触发，且为非连续状态：新线段开始
		if(Grayscale_Out.Grayscale_Trigger_State[i] == 1 && Trigger_Continuous_State == 0)
		{
			//记录当前线段起点
			Segment_Start[Grayscale_Out.Number_Of_Discontinuous_Points] = i;
			//置连续状态
			Trigger_Continuous_State = 1;
		}
		//不存在触发，且为连续状态：线段结束
		else if(Grayscale_Out.Grayscale_Trigger_State[i] == 0 && Trigger_Continuous_State == 1)
		{
			//记录当前线段终点
			Segment_End[Grayscale_Out.Number_Of_Discontinuous_Points] = i - 1;
			//置不连续状态
			Trigger_Continuous_State = 0;
			//线段数量自增
			Grayscale_Out.Number_Of_Discontinuous_Points ++;
		}
	}
	//最后一个传感器仍触发，补录该线段终点
	if(Trigger_Continuous_State == 1)
	{
		//记录当前线段终点
		Segment_End[Grayscale_Out.Number_Of_Discontinuous_Points] = Grayscale_Num - 1;
		//线段数量自增
		Grayscale_Out.Number_Of_Discontinuous_Points ++;
		//置不连续状态
		Trigger_Continuous_State = 0;
	}
}


/** @brief	灰度传感器ADC映射计算
  * @note	该函数只计算离中心点最近的连续触发段的一维映射值
  **/
void Grayscale_ADC_Map_Compute(void)
{
	char Compute_Mode = 0;	//计算模式，0为计算灰度数据，1为不计算灰度数据，直接复位

	float Middle_Index = (Grayscale_Num - 1) / 2.0f;	//中部传感器的索引值
	
	//计算各个不连续触发线段的一维坐标映射值
	for(char i=0;i<Grayscale_Out.Number_Of_Discontinuous_Points;i++)
	{
		float Grayscale_Weight=0;	//灰度权重值
		float Grayscale_Int=0;		//灰度总值

		for(char j=Segment_Start[i];j<=Segment_End[i];j++)
		{
			//计算灰度权重值
			Grayscale_Weight += Grayscale.Grayscale_ADC_Compute_Percent[j]
								* ((float)j - Middle_Index)
								* Grayscale_Weight_Bias;
			//累计总值
			Grayscale_Int  += Grayscale.Grayscale_ADC_Compute_Percent[j];
		}
		//计算映射值（防止除零）
		if(Grayscale_Int != 0)
			Segment_Map[i] = Grayscale_Weight / Grayscale_Int;
		else
			Segment_Map[i] = 0;
		
		//复位灰度权重值与总值以便下次运算
		Grayscale_Weight = 0;
		Grayscale_Int = 0;
	}

	//干扰隔离
	//单段线触发数量大于等于Grayscale_Cover_Line_Num * 2个，为十字或直角路口
	for(char i=0;i<Grayscale_Out.Number_Of_Discontinuous_Points;i++)
	{
		if(Segment_End[i] - Segment_Start[i] >= Grayscale_Cover_Line_Num * 2)
			//计算模式为1，隔离路口大面积标线干扰
			Compute_Mode = 1;
	}
	//触发数量为0
	if(Grayscale_Out.Grayscale_Trigger_Num == 0)
		//计算模式为1，未寻到线，隔离干扰
		Compute_Mode = 1;

	//计算灰度
	if(Compute_Mode == 0 && Grayscale_Out.Number_Of_Discontinuous_Points > 0)
	{
		float Grayscale_Map_Middle_Err = 1000.0f;	//中值偏差
		char Grayscale_Map_Middle_Pointer = 0;		//最中值指针
		char Grayscale_Map_Min_Pointer = 0;			//最左值指针
		char Grayscale_Map_Max_Pointer = 0;			//最右值指针

		//搜寻最中值，最小值，最大值
		for(char i=0;i<Grayscale_Out.Number_Of_Discontinuous_Points;i++)
		{
			//搜寻最中值：映射绝对值最小者
			float Map_Value = Segment_Map[i];
			if(Map_Value < 0) Map_Value = -Map_Value;
			if(Map_Value < Grayscale_Map_Middle_Err)
			{
				Grayscale_Map_Middle_Err = Map_Value;
				Grayscale_Map_Middle_Pointer = i;
			}

			//搜寻最小值
			if(Segment_Map[Grayscale_Map_Min_Pointer] > Segment_Map[i])
				Grayscale_Map_Min_Pointer = i;
			//搜寻最大值
			if(Segment_Map[Grayscale_Map_Max_Pointer] < Segment_Map[i])
				Grayscale_Map_Max_Pointer = i;
		}

		//巡线跟随选择
		switch(Grayscale_Ctrl.Target_Following)
		{
			//中线巡线模式
			case Middle_Line_Follows: Grayscale_Out.Grayscale_Map = Segment_Map[Grayscale_Map_Middle_Pointer];break;
			//左线巡线模式
			case Left_Line_Follows: Grayscale_Out.Grayscale_Map = Segment_Map[Grayscale_Map_Min_Pointer];break;
			//右线巡线模式
			case Right_Line_Follows: Grayscale_Out.Grayscale_Map = Segment_Map[Grayscale_Map_Max_Pointer];break;
		}
	}
	//不计算灰度
	else
		Grayscale_Out.Grayscale_Map = 0;
}

#define Number_Of_Records  10	//历史记录次数

/** @brief	寻找最靠近阵列中心的连续触发段
  * @param	Number_Of_Segments	连续触发段数量
  * @param	Segment_Start		各连续触发段起始传感器索引
  * @param	Segment_End			各连续触发段结束传感器索引
  * @retval	最靠近阵列中心的线段序号
  **/
char Segment_Track_Find(char Number_Of_Segments, char *Segment_Start, char *Segment_End)
{
	//传感器阵列中心位置
	float Middle_Index = (Grayscale_Num - 1) / 2.0f;
	//最靠近中部的线段选择参数
	char Track_Segment = 0;						//被跟踪线段序号
	float Track_Segment_Deviation = 1000.0f;	//被跟踪线段中心与阵列中心的最小偏差
	float Segment_Deviation;					//线段中心与阵列中心的偏差

	//遍历各线段，寻找中心最靠近阵列中心的线段
	for(char i=0;i<Number_Of_Segments;i++)
	{
		//计算线段中心与阵列中心的偏差
		Segment_Deviation = (Segment_Start[i] + Segment_End[i]) / 2.0f - Middle_Index;
		if(Segment_Deviation < 0) Segment_Deviation = -Segment_Deviation;
		//更新最靠近中部的线段
		if(Segment_Deviation < Track_Segment_Deviation)
		{
			Track_Segment_Deviation = Segment_Deviation;
			Track_Segment = i;
		}
	}

	//返回被跟踪线段序号
	return Track_Segment;
}

/** @brief	路口识别
  * @note	当巡线不连续触发段起止索引发生变化时，将线段数量及起止索引存储至历史记录缓存区
  * @note	New_Records_Pointer始终指向最新数据，最新数据向上递增，环形存储，新数据覆盖最老数据
  * @note	根据当前与上一时段连续触发段统计判定路口类型，判断结果装载至Now_Intersection
  **/
void Intersection_Recognition(void)
{
	//历史连续触发段数量统计，并记录各线段起止索引
	static char New_Records_Pointer;								//最新时段指针
	static char Records_Number_Of_Discontinuous_Points[Number_Of_Records];	//各时段线段数量
	static char Records_Segment_Start[Number_Of_Records][Grayscale_Num];	//各线段起始传感器索引
	static char Records_Segment_End[Number_Of_Records][Grayscale_Num];		//各线段结束传感器索引

	char Records_Changed = 0;	//触发段变化标志

	//-------变化判定-------//
	//与最新存储的记录比对，判断触发段数量是否变化
	if(Records_Number_Of_Discontinuous_Points[New_Records_Pointer] != Grayscale_Out.Number_Of_Discontinuous_Points)
		Records_Changed = 1;
	//数量一致，逐段比对各线段起止索引
	else
	{
		for(char i=0;i<Grayscale_Out.Number_Of_Discontinuous_Points;i++)
		{
			if(Records_Segment_Start[New_Records_Pointer][i] != Segment_Start[i] ||
			   Records_Segment_End[New_Records_Pointer][i] != Segment_End[i])
			{
				Records_Changed = 1;
				break;
			}
		}
	}
	//存在变化时存储一次
	if(Records_Changed)
	{
		//最新时段指针向上递增，环形存储
		New_Records_Pointer = (New_Records_Pointer + 1) % Number_Of_Records;
		//存储线段数量
		Records_Number_Of_Discontinuous_Points[New_Records_Pointer] = Grayscale_Out.Number_Of_Discontinuous_Points;
		//存储各线段起止索引
		for(char i=0;i<Grayscale_Out.Number_Of_Discontinuous_Points;i++)
		{
			Records_Segment_Start[New_Records_Pointer][i] = Segment_Start[i];
			Records_Segment_End[New_Records_Pointer][i] = Segment_End[i];
		}
	}

	//-------路口判定-------//
	//上一时段记录指针（最新记录的前一位）
	char Last_Record_Pointer = (New_Records_Pointer + Number_Of_Records - 1) % Number_Of_Records;
	//当前与上一时段连续触发段数量
	char Now_Segment_Num = Grayscale_Out.Number_Of_Discontinuous_Points;
	char Last_Segment_Num = Records_Number_Of_Discontinuous_Points[Last_Record_Pointer];

	//默认无路口
	Grayscale_Out.Now_Intersection = No_Crossroad;

	//无触发段：未寻到线
	if(Now_Segment_Num == 0)
	{
		//无路口
		Grayscale_Out.Now_Intersection = No_Crossroad;
	}
	//单一连续触发段
	else if(Now_Segment_Num == 1)
	{
		char Start = Segment_Start[0];	//触发段起始传感器索引
		char End = Segment_End[0];		//触发段结束传感器索引

		//触发段横跨整个阵列：十字路口
		if(Start == 0 && End == Grayscale_Num - 1)
			Grayscale_Out.Now_Intersection = Crossroad;
		//触发段从阵列中部延伸至右端：右直角路口
		else if(End == Grayscale_Num - 1 && Start <= Grayscale_Num - (2 * Grayscale_Cover_Line_Num - 1))
			Grayscale_Out.Now_Intersection = Right_Right_angle_Intersection;
		//触发段从左端延伸至阵列中部：左直角路口
		else if(Start == 0 && End >= (2 * Grayscale_Cover_Line_Num - 1))
			Grayscale_Out.Now_Intersection = Left_Right_angle_Intersection;
		//上一时段存在分支，当前已合并为单线：分支汇入主线
		else if(Last_Segment_Num >= 2)
		{
			char Last_Main = Segment_Track_Find(Last_Segment_Num,
					Records_Segment_Start[Last_Record_Pointer], Records_Segment_End[Last_Record_Pointer]);
			float Last_Main_Center = (Records_Segment_Start[Last_Record_Pointer][Last_Main]
					+ Records_Segment_End[Last_Record_Pointer][Last_Main]) / 2.0f;
			char Left_Branch = 0;	//上一时段存在左侧分支
			char Right_Branch = 0;	//上一时段存在右侧分支

			//统计上一时段各分支相对主线的位置
			for(char i=0;i<Last_Segment_Num;i++)
			{
				float Branch_Center;
				if(i == Last_Main) continue;
				Branch_Center = (Records_Segment_Start[Last_Record_Pointer][i]
						+ Records_Segment_End[Last_Record_Pointer][i]) / 2.0f;
				if(Branch_Center > Last_Main_Center) Right_Branch = 1;
				else Left_Branch = 1;
			}

			//两侧分支同时汇入主线：两侧钝角路口
			if(Left_Branch && Right_Branch)
				Grayscale_Out.Now_Intersection = Two_Sided_Obtuse_Angle_Intersection;
			//仅右侧分支汇入主线：右钝角路口
			else if(Right_Branch)
				Grayscale_Out.Now_Intersection = Right_Obtuse_Angle_Intersection;
			//仅左侧分支汇入主线：左钝角路口
			else
				Grayscale_Out.Now_Intersection = Left_Obtuse_Angle_Intersection;
		}
	}
	//多个连续触发段：主线与分支线并存
	else
	{
		char Now_Main = Segment_Track_Find(Now_Segment_Num, Segment_Start, Segment_End);
		float Now_Main_Center = (Segment_Start[Now_Main] + Segment_End[Now_Main]) / 2.0f;
		char Last_Main = 0;
		float Last_Main_Center = 0;
		signed char Left_Separation = 0;	//左侧分支分离变化，1分离，-1汇入
		signed char Right_Separation = 0;	//右侧分支分离变化，1分离，-1汇入

		//上一时段存在多个触发段时，寻找其主线
		if(Last_Segment_Num >= 2)
		{
			Last_Main = Segment_Track_Find(Last_Segment_Num,
					Records_Segment_Start[Last_Record_Pointer], Records_Segment_End[Last_Record_Pointer]);
			Last_Main_Center = (Records_Segment_Start[Last_Record_Pointer][Last_Main]
					+ Records_Segment_End[Last_Record_Pointer][Last_Main]) / 2.0f;
		}

		//遍历当前各分支线段，统计其相对主线的分离变化
		for(char i=0;i<Now_Segment_Num;i++)
		{
			float Branch_Center;
			char Now_Gap, Last_Gap;
			if(i == Now_Main) continue;

			//上一时段为单线或无同侧分支时间隔为0，分支刚分离
			Last_Gap = 0;
			Branch_Center = (Segment_Start[i] + Segment_End[i]) / 2.0f;

			//右侧分支
			if(Branch_Center > Now_Main_Center)
			{
				//当前分支与主线间隔
				Now_Gap = Segment_Start[i] - Segment_End[Now_Main];
				//在上一时段中寻找同侧分支，计算上一时段分支与主线间隔
				if(Last_Segment_Num >= 2)
				{
					for(char k=0;k<Last_Segment_Num;k++)
					{
						float Last_Branch_Center;
						if(k == Last_Main) continue;
						Last_Branch_Center = (Records_Segment_Start[Last_Record_Pointer][k]
								+ Records_Segment_End[Last_Record_Pointer][k]) / 2.0f;
						if(Last_Branch_Center > Last_Main_Center)
						{
							Last_Gap = Records_Segment_Start[Last_Record_Pointer][k]
									- Records_Segment_End[Last_Record_Pointer][Last_Main];
							break;
						}
					}
				}

				//间隔增大：分支分离，间隔减小：分支汇入
				if(Now_Gap > Last_Gap) Right_Separation = 1;
				else if(Now_Gap < Last_Gap) Right_Separation = -1;
			}
			//左侧分支
			else
			{
				//当前分支与主线间隔
				Now_Gap = Segment_Start[Now_Main] - Segment_End[i];
				//在上一时段中寻找同侧分支，计算上一时段分支与主线间隔
				if(Last_Segment_Num >= 2)
				{
					for(char k=0;k<Last_Segment_Num;k++)
					{
						float Last_Branch_Center;
						if(k == Last_Main) continue;
						Last_Branch_Center = (Records_Segment_Start[Last_Record_Pointer][k]
								+ Records_Segment_End[Last_Record_Pointer][k]) / 2.0f;
						if(Last_Branch_Center < Last_Main_Center)
						{
							Last_Gap = Records_Segment_Start[Last_Record_Pointer][Last_Main]
									- Records_Segment_End[Last_Record_Pointer][k];
							break;
						}
					}
				}

				//间隔增大：分支分离，间隔减小：分支汇入
				if(Now_Gap > Last_Gap) Left_Separation = 1;
				else if(Now_Gap < Last_Gap) Left_Separation = -1;
			}
		}

		//两侧分支同时存在
		if(Left_Separation && Right_Separation)
		{
			//两侧分支均分离：两侧锐角路口
			if(Left_Separation == 1 && Right_Separation == 1)
				Grayscale_Out.Now_Intersection = Two_Sided_Acute_Angle_Intersection;
			//两侧分支均汇入：两侧钝角路口
			else if(Left_Separation == -1 && Right_Separation == -1)
				Grayscale_Out.Now_Intersection = Two_Sided_Obtuse_Angle_Intersection;
			//一侧分离一侧汇入，按整体变化趋势判定
			else if(Left_Separation + Right_Separation > 0)
				Grayscale_Out.Now_Intersection = Two_Sided_Acute_Angle_Intersection;
			else
				Grayscale_Out.Now_Intersection = Two_Sided_Obtuse_Angle_Intersection;
		}
		//仅右侧分支
		else if(Right_Separation)
		{
			//右侧分支分离：右锐角路口，右侧分支汇入：右钝角路口
			if(Right_Separation == 1)
				Grayscale_Out.Now_Intersection = Right_Acute_Angle_Intersection;
			else
				Grayscale_Out.Now_Intersection = Right_Obtuse_Angle_Intersection;
		}
		//仅左侧分支
		else if(Left_Separation)
		{
			//左侧分支分离：左锐角路口，左侧分支汇入：左钝角路口
			if(Left_Separation == 1)
				Grayscale_Out.Now_Intersection = Left_Acute_Angle_Intersection;
			else
				Grayscale_Out.Now_Intersection = Left_Obtuse_Angle_Intersection;
		}
	}

}

