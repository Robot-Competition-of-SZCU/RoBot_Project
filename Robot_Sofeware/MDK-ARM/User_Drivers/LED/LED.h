///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 8th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为LED底层驱动头文件
///////////////////////////////////////
#ifndef __LED_H__
#define __LED_H__

//LED控制状态枚举
typedef enum
{
  LED_OFF = 0,
  LED_ON
}LED_PinState;


void RUN_LED_Control(LED_PinState State);	//运行指示灯控制
void PD3_LED_Control(LED_PinState State);	//PD3指示灯控制
void PD4_LED_Control(LED_PinState State);	//PD4指示灯控制

#endif
