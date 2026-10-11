///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 21th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为IO输入输出源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "IO_Put.h"

#include "Control.h"

/** @brief	GPIO中断触发任务
  * @param	GPIO_Pin: 触发引脚
  **/
void GPIO_Trigger_Control(uint32_t GPIO_Pin)
{
	//PF7被触发
	if(GPIO_Pin == Sensor_Middle) 
	{
		//当前为允许运行状态
		if(RUN_Parm.RUN_Control == 1)
		{
			//上升沿触发
			if(HAL_GPIO_ReadPin(GPIOF,Sensor_Middle) == 1)
			{
				//开始运行
				RUN_Parm.RUN_State = 1;	
			}
		}
		//运行状态下
		if(RUN_Parm.RUN_State == 1)
		{
			INPut_Signal.Sensor_Middle_Int++;
		}
	}
	//PF6被触发
	if(GPIO_Pin == Sensor_Left)
		INPut_Signal.Sensor_Left_Int++;
	//PF5被触发
	if(GPIO_Pin == Sensor_Right)
		INPut_Signal.Sensor_Right_Int++;

	//PF4,PF3被触发
	if(GPIO_Pin == Microswitch_Left || GPIO_Pin == Microswitch_Right)
		INPut_Signal.Microswitch_State_Int++;
}


