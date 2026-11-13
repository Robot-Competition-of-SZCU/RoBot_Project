///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 8th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为LED底层驱动源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "LED.h"

/** @brief	运行指示灯控制，控制端口为PC13，高电平开启
  * @param	State	传递参数为 LED_ON ；LED_OFF
  **/
void RUN_LED_Control(LED_PinState State)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,(GPIO_PinState)State);
}

/** @brief	PD3指示灯控制，控制端口为PD3，高电平开启
  * @param	State	传递参数为 LED_ON ；LED_OFF
  **/
void PD3_LED_Control(LED_PinState State)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,(GPIO_PinState)State);
}

/** @brief	PD3指示灯控制，控制端口为PD4，高电平开启
  * @param	State	传递参数为 LED_ON ；LED_OFF
  **/
void PD4_LED_Control(LED_PinState State)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,(GPIO_PinState)State);
}


