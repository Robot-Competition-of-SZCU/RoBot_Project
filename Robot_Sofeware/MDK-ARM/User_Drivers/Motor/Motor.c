///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为电机驱动源文件
///////////////////////////////////////
#include "Motor.h"
#include "stm32f4xx_hal.h"

#include "tim.h"

//电机状态控制宏定义
#define M1_Control(x)	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,(GPIO_PinState)(x))
#define M2_Control(x)	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,(GPIO_PinState)(x))
#define M3_Control(x)	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,(GPIO_PinState)(x))
#define M4_Control(x)	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,(GPIO_PinState)(x))

/** @brief	电机独立控制
  * @param	Motor_Select	电机编号
  * @param  RUN_State		运行状态
  * @param	Duty			输出占空比	范围：0~90
  **/
void Motor_Control_One(Motor_Number Motor_Select,Motor_State RUN_State,float Duty)
{
	//占空比限位，防止超过90%
	if(Duty>=95)	Duty=90;
	if(Duty<=0)		Duty=0;
	
	//计算CCR值
	short CCR = TIM1 -> ARR * (Duty/100);
	
	//装载控制状态与CCR
	switch(Motor_Select)
	{
		case M1:	
				M1_Control(RUN_State);
				TIM1->CCR1 = CCR;
			break;
		case M2:
				M2_Control(RUN_State);
				TIM1->CCR2 = CCR;
			break;
		case M3:
				M3_Control(RUN_State);
				TIM1->CCR3 = CCR;
			break;
		case M4:
				M4_Control(RUN_State);
				TIM1->CCR4 = CCR;
			break;
	}
}

/** @brief	电机初始化
  * @note	关闭所有电机
  **/
void Motor_Init(void)
{
	Motor_Stop();
}

/** @brief	开启所有电机
  **/
void Motor_Start(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

/** @brief	关闭所有电机
  **/
void Motor_Stop(void)
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
}




