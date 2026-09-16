///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 9rd, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为灰度采集计算及巡线解析头文件
///////////////////////////////////////
#ifndef __Grayscale_ADC_H__
#define __Grayscale_ADC_H__

/**↓↓用户可修改参数↓↓**/	
#define Calibrate_Sampling_Num		100			//校准时采样次数，采样间隔用户自行设定
#define Grayscale_Num				14			//灰度传感器数量设定，范围4-16
#define Grayscale_Cover_Line_Num	3			//线宽所能覆盖的传感器最大数量
#define Grayscale_Weight_Bias		1			//从中心点开始，每个传感器的偏移权重
#define Grayscale_Data_Stream_Type  0			//灰度数据流类型，0为模拟量，1为数字量
/**↑↑用户可修改参数↑↑**/

//灰度传感器ADC数据缓存区
extern short Grayscale_ADC_Buffer[16];

//校准状态结构体
typedef enum 
{
  Calibration_OK = 0,	//校准完成
  Calibration_BUSY,		//校准中
}Calibration_State;

typedef enum{
	No_Crossroad = 0,					//无路口

	Crossroad,							//十字路口
	Left_Right_angle_Intersection,		//左直角路口
	Right_Right_angle_Intersection,		//右直角路口

	Left_Acute_Angle_Intersection,		//左锐角路口
	Right_Acute_Angle_Intersection,		//右锐角路口
	Two_Sided_Acute_Angle_Intersection,	//两侧锐角路口，↓箭头形状

	Left_Obtuse_Angle_Intersection,		//左钝角路口
	Right_Obtuse_Angle_Intersection,	//右钝角路口
	Two_Sided_Obtuse_Angle_Intersection,//两侧钝角路口，↑箭头形状
}Intersection;

typedef enum{
	Middle_Line_Follows = 0,		//中心跟随模式，在多线情况下只跟随中心线
	Left_Line_Follows,				//左侧线跟随模式，在多线情况下只跟随最左侧线
	Right_Line_Follows,				//右侧线跟随模式，在多线情况下只跟随最右侧线
}Following_Mode;


//灰度传感器数据结构体
struct Grayscale_Parm{
	short Grayscale_ADC_Actual[16];	//灰度传感器ADC实际数据，从左至右依次对应每个传感器
	short Grayscale_ADC_Max[16];	//灰度传感器数据最大极限值
	short Grayscale_ADC_Min[16];	//灰度传感器数据最小极限值

	short Grayscale_ADC_Compute[16];//灰度传感器数据校准值
	float Grayscale_ADC_Compute_Percent[16];	//灰度传感器数据校准后百分比
};
extern struct Grayscale_Parm Grayscale;

//灰度传感器控制结构体
struct Grayscale_Control{
	float Grayscale_ADC_Trigger_Threshold;		//灰度传感器触发阈值
	char Compute_Map_Mode;						//映射计算模式，0：正向计算(线位置传感器值为高)，1：反向计算(线位置传感器值为低)
	Following_Mode Target_Following;			//巡线目标跟随选择
};
extern struct Grayscale_Control Grayscale_Ctrl;

//灰度传感器输出结构体
struct Grayscale_OutPut{
	char Grayscale_Trigger_State[16];			//灰度传感器触发状态
	char Grayscale_Trigger_Num;					//灰度传感器触发数量
	char Number_Of_Discontinuous_Points;		//不连续触发段数量统计
	
	float Grayscale_Map;						//灰度传感器ADC映射一维坐标值

	Intersection Now_Intersection;				//当前路口状态
};
extern struct Grayscale_OutPut Grayscale_Out;


typedef enum{
	Calibration_Max = 0,
	Calibration_Min
}Calibration_Mode;


void Grayscale_Init(void);			//灰度传感器ADC采集初始化
Calibration_State Grayscale_ADC_Calibration(Calibration_Mode Mode);	//灰度传感器ADC校准

void Grayscale_Data_Stream_Processing(void);	//灰度数据流处理

void Grayscale_ADC_Compute(void);		//灰度传感器ADC解算
void Scan_Line_Segment_Compute(void);	//巡线不连续触发段统计计算
void Grayscale_ADC_Map_Compute(void);	//灰度传感器ADC映射计算
char Segment_Track_Find(char Number_Of_Segments, char *Segment_Start, char *Segment_End);	//寻找最靠近阵列中心的连续触发段
void Intersection_Recognition(void);	//路口识别

#endif
