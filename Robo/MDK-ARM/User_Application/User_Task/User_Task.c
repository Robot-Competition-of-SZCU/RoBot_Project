///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 10th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为任务设置源文件
///////////////////////////////////////
#include "User_Task.h"
#include "stm32f4xx_hal.h"
#include <string.h>

//HAL库文件
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//底层驱动文件
#include "LED.h"
#include "KEY.h"
#include "Motor.h"
#include "Servo.h"
#include "Encoder.h"
#include "UART.h"
#include "OLED.h"
#include "Grayscale_ADC.h"
#include "Flash.h"

//应用程序文件
#include "Control.h"
#include "PID.h"
#include "Set_And_Show.h"
#include "IMU.h"
#include "Interrupt.h"

//各任务信息存储区
TaskStatus_t xTaskDetails_defaultTask;
TaskStatus_t xTaskDetails_RUN_LED_Flash;
TaskStatus_t xTaskDetails_KEY_Scan;
TaskStatus_t xTaskDetails_State_LED_Flash;
TaskStatus_t xTaskDetails_Set_And_Show;
TaskStatus_t xTaskDetails_Slow_Compute;
TaskStatus_t xTaskDetails_High_Compute;
TaskStatus_t xTaskDetails_UART_Debug;

/** @brief	总初始化函数
  * @note	该函数为在任务调度器启动前的所有硬件与软件初始化函数
  **/
void User_Init(void)
{
	//单片机首次下载程序时不要运行该函数，否则会产生数据错误
	//再执行一遍flash写入操作后方可运行该函数
	Flash_Read();			//从Flash中读取数据
	
	//Motor_Init();			//电机初始化
	Servo_Init();			//舵机初始化
	Encoder_Init();			//编码器初始化
	UART_Receive_Init();	//串口接收初始化
	Grayscale_ADC_Init();	//灰度传感器ADC采集初始化
	OLED_Init();			//OLED初始化
	RUN_Parm_Init();		//运行参数初始化
	PID_Parameter_Init();	//PID参数初始化
	
	HAL_Delay(1000);
	UART_Audio_Loud_Control(30);
	HAL_Delay(20);
	UART_Audio_Mode_Control();
	// HAL_Delay(20);
	// UART_Audio_Control(0);

	Motor_Start();			//电机控制启动
	
	//HAL_TIM_Base_Start_IT(&htim6);		//开启TIM6定时中断，间隔为1ms
}


/** @brief	运行指示灯闪烁任务
  * @note	闪烁间隔为200ms
  **/
void RUN_LED_Flash_Task(void *argument)
{
	for(;;)
	{
		RUN_LED_Control(LED_ON);
		osDelay(200);
		RUN_LED_Control(LED_OFF);
		osDelay(200);
	}
}

/** @brief	按键扫描任务
  * @note	扫描周期为20ms
  **/
void KEY_Scan_Task(void *argument)
{
	//固定任务周期为20ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(20);
	
	for(;;)
	{	//20ms阻塞延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		KEY_Polling_Function();		//按键轮询
	}
}

/** @brief	OLED显示任务
  * @note	屏幕刷新周期为100ms,10FPS
  **/
void Set_And_Show_Task(void *argument)
{
	//固定任务周期为100ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(100);
	for(;;)
	{	//100ms阻塞延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		System_Menu_Control_And_Show();	//系统菜单控制与显示
	}
}

/** @brief	高速计算任务
  * @note	执行间隔1ms
  **/
void High_Compute_Task(void *argument)
{
	//固定任务周期为1ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	
	for(;;)
	{	//1ms阻塞延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		Grayscale_ADC_Compute();		//灰度传感器ADC解算
	}
}

/** @brief	低速计算任务
  * @note	执行间隔5ms
  **/
void Slow_Compute_Task(void *argument)
{
	//固定任务周期为5ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(5);
	for(;;)
	{	//5ms阻塞延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  
		static short Time_Base;	//时间基准 
		Time_Base++;

		RUN_Speed_Control();				//运行速度控制
		Mileage_Int_Compute();				//里程累计
		
		//5ms延时任务
		Encoder_Compute();					//计算编码器输入频率
		Encoder_Input_Frequiency_Filter();	//编码器输入频率滤波
		Encoder_Speed_Compute();			//编码器速度计算
		Motor_Speed_Control();				//电机速度控制，PID内环

		//IMU姿态解析
		IMU_GET_Data();
		
		//10ms延时任务
		if(Time_Base % 2 == 0)
		{
			Grayscale_ADC_Map_Compute();	//灰度传感器ADC映射计算
			//RUN_Control();				//运行控制
			//根据RUN_Parm.Control_State状态切换控制方式
			if(RUN_Parm.Control_State == Direct_Control)
			{	//直接控制模式
				Motor_Control_Parm.Motor1_Speed = Motor_Control_Parm.Base_Speed;
				Motor_Control_Parm.Motor2_Speed = Motor_Control_Parm.Base_Speed;
				Motor_Control_Parm.Motor3_Speed = Motor_Control_Parm.Base_Speed;
				Motor_Control_Parm.Motor4_Speed = Motor_Control_Parm.Base_Speed;
			}
			else if(RUN_Parm.Control_State == ScanLine_Control)
			{
				Scan_Line_Control();			//巡线控制
			}
			else if(RUN_Parm.Control_State == Angle_Control)
			{
				if(RUN_Parm.Angle_Control_Choice == Angle_Control_Advance)
					Angle_Patrol_Control();			//角度跟随控制
				else if(RUN_Parm.Angle_Control_Choice == Angle_Control_Retreat)
					Angle_Patrol_Retreat_Control();	
			}	
			else if(RUN_Parm.Control_State == Arc_Turn_Control)
			{
				Arc_Turn_Control_Compute();			//行进中弧线转弯控制
			}

			Servo_Control();				//舵机控制
		}
		//1000ms延时任务
		if(Time_Base == 200)
		{
			// //获取各任务的信息
			// vTaskGetInfo(xTaskGetHandle("defaultTask"),&xTaskDetails_RUN_LED_Flash,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("RUN_LED_Flash"),&xTaskDetails_RUN_LED_Flash,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("KEY_Scan"),&xTaskDetails_KEY_Scan,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("Set_And_Show"),&xTaskDetails_Set_And_Show,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("State_LED_Flash"),&xTaskDetails_State_LED_Flash,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("Slow_Compute"),&xTaskDetails_Slow_Compute,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("High_Compute"),&xTaskDetails_High_Compute,pdTRUE,eInvalid);
			// vTaskGetInfo(xTaskGetHandle("UART_Debug"),&xTaskDetails_UART_Debug,pdTRUE,eInvalid);
		}
		Time_Base %= 200;	//限位
	}
}

/** @brief	系统运行控制任务
  * @note	阻塞式运行控制程序
  **/
void RUN_Control_Task(void *argument)
{
	for(;;)
	{
		//系统运行控制任务
		RUN_System_Control();
		osDelay(1);
	}
}

/** @brief	GPIO中断触发任务
  * @note	阻塞式等待中断触发信号
  **/
void GPIO_IT_Trigger_Task(void *argument)
{
  uint32_t pin_number;

  for(;;)
  {
    //阻塞等待中断回调发送的消息
    if(osMessageQueueGet(GPIO_Tigger_StateHandle, &pin_number, NULL, osWaitForever) == osOK)
    {
		//GPIO中断控制
		GPIO_Trigger_Control(pin_number);
    }
  }
}

/** @brief	串口接收解析任务
  * @note	解析串口数据
  **/
void Visual_Identity_Task(void *argument)
{
  for(;;)
  {
    //阻塞等待串口数据包，收到后立即解析，不影响下一次接收
    if(osMessageQueueGet(Visual_Identity_QueueHandle, &Packet, NULL, osWaitForever) == osOK)
    {
      Visual_Identity_Data_Analysis(&Packet);	//视觉识别数据解析
    }
  }
  /* USER CODE END Visual_Identity_Task */
}

/** @brief	串口调试任务
  * @note	执行间隔10ms
  **/
void UART_Debug_Task(void *argument)
{
	//固定任务周期为10ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);

	for(;;)
	{	//10ms阻塞延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		// static char Send_Data[40];
		
		// memcpy(&Send_Data[0], &Grayscale_ADC_Buffer[0],2);
		// memcpy(&Send_Data[2], &Grayscale_ADC_Buffer[1],2);
		// memcpy(&Send_Data[4], &Grayscale_ADC_Buffer[2],2);
		// memcpy(&Send_Data[6], &Grayscale_ADC_Buffer[3],2);
		// memcpy(&Send_Data[8], &Grayscale_ADC_Buffer[4],2);
		// memcpy(&Send_Data[10], &Grayscale_ADC_Buffer[5],2);
		// memcpy(&Send_Data[12], &Grayscale_ADC_Buffer[6],2);
		// memcpy(&Send_Data[14], &Grayscale_ADC_Buffer[7],2);
		// memcpy(&Send_Data[16], &Grayscale_ADC_Buffer[8],2);
		// memcpy(&Send_Data[18], &Grayscale_ADC_Buffer[9],2);
		// memcpy(&Send_Data[20], &Grayscale_ADC_Buffer[10],2);
		// memcpy(&Send_Data[22], &Grayscale_ADC_Buffer[11],2);
		// memcpy(&Send_Data[24], &Grayscale_ADC_Buffer[12],2);
		// memcpy(&Send_Data[26], &Grayscale_ADC_Buffer[13],2);
		
		// memcpy(&Send_Data[0],&PID_Motor1.Target,sizeof(float));
		// memcpy(&Send_Data[4],&Encoder1.RMP_S,sizeof(float));
		// UART_Debug_Send_Date(Send_Data,8);
		
		// memcpy(&Motor_Control_Parm.Motor1_Speed,&Debug_Receive_Buffer[1],sizeof(float));
		// memcpy(&Line_Patrol_PID.Ki,&Debug_Receive_Buffer[9],sizeof(float));
		// memcpy(&Line_Patrol_PID.Kd,&Debug_Receive_Buffer[13],sizeof(float));
	}
}

/** @brief	任务栈溢出钩子函数
  * @note	由FreeRTOS内核在检测到任务栈溢出时调用，进入死循环并显示故障信息
  **/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	//溢出任务信息，防止编译器警告
	(void)xTask;
	(void)pcTaskName;

	//OLED显示硬件故障信息
	OLED_HardFault_Error_Message_Show();

	//关闭中断，进入死循环，便于现场排查
	taskDISABLE_INTERRUPTS();
	for(;;) {}
}
	

