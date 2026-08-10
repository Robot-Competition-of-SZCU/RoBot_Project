///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 15th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为控制输出源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "Control.h"

#include "OLED.h"

//控制状态结构体
struct Motor_Control Motor_Control_Parm;
//运行状态结构体
struct RUN RUN_Parm;

/** @brief	运行参数初始化
  **/
void RUN_Parm_Init(void)
{

	Motor_Control_Parm.M1 = Advance;
	Motor_Control_Parm.M2 = Advance;
	Motor_Control_Parm.M3 = Advance;
	Motor_Control_Parm.M4 = Advance;

    //Motor_Control_Parm.Base_Speed_Set = 1;
}

/** @brief	运行控制
  **/
void RUN_Control(void)
{
    if(RUN_Parm.RUN_State)
    {
        Motor_Control_Parm.Base_Speed +=0.1f;
        if(Motor_Control_Parm.Base_Speed >= Motor_Control_Parm.Base_Speed_Set)
            Motor_Control_Parm.Base_Speed = Motor_Control_Parm.Base_Speed_Set;
    }
    else
    {
        Motor_Control_Parm.Base_Speed = 0;
    }
}

