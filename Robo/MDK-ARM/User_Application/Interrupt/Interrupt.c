///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为中断任务源文件
///////////////////////////////////////
#include "Interrupt.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "main.h"
#include "usart.h"
#include "UART.h"

/** @brief	GPIO外部输入中断
  **/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t pin_number = GPIO_Pin;

    if (GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_5 || GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_3)
    {
        //CMSIS-RTOS v2消息队列接口可在中断中调用(超时参数必须为0)
        osMessageQueuePut(GPIO_Tigger_StateHandle, &pin_number, 0U, 0U);
    }
}

/** @brief	串口空闲中断回调函数
  * @note	USART6一帧数据接收完成后触发，由USART6中断服务函数调用
  **/
void USART6_IDLE_Interrupt_Callback(void)
{
    //检测串口空闲中断标志
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
    {
        //一帧数据接收完成，计算数据包长度并发送消息队列，重启DMA接收
        UART_Visual_Identity_IDLE_Handle();
    }
}



///** @brief	TIM6定时中断回调函数
//  * @note	中断间隔为1ms
//  **/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM6)
//	{
//		
//	}
//}

