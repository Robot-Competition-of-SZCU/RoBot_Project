///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 14th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为Flash操作源文件
///////////////////////////////////////
#include "Flash.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#include "freertos.h"
#include "task.h"

#include "Grayscale_ADC.h"
#include "Servo.h"

#include "Control.h"

//Flash存储数据结构体
struct Flash Flash_Save;

/** @brief	Flash写数据
  * @retval	写入状态码
  * @note	将Flash_Save结构体内数据写入单片机最后一个扇区
  **/
HAL_StatusTypeDef Flash_Write(void)
{
	//数据拷贝至Flash_Save，以更新
	//灰度传感器数据
	memcpy(&Flash_Save.Grayscale_ADC_Max,&Grayscale.Grayscale_ADC_Max,sizeof(Grayscale.Grayscale_ADC_Max));
	memcpy(&Flash_Save.Grayscale_ADC_Min,&Grayscale.Grayscale_ADC_Min,sizeof(Grayscale.Grayscale_ADC_Min));
	memcpy(&Flash_Save.Compute_Map_Mode,&Grayscale.Compute_Map_Mode,sizeof(Grayscale.Compute_Map_Mode));
	memcpy(&Flash_Save.Grayscale_ADC_Trigger_Threshold,&Grayscale.Grayscale_ADC_Trigger_Threshold,sizeof(Grayscale.Grayscale_ADC_Trigger_Threshold));
	//舵机数据
	memcpy(&Flash_Save.Servo_EN_State,&Servo_State,sizeof(Servo_State));
	//运行数据
	memcpy(&Flash_Save.Base_High_Speed_Set,&Motor_Control_Parm.Base_High_Speed_Set,sizeof(float));
	memcpy(&Flash_Save.Accelerated_Speed,&Motor_Control_Parm.Accelerated_Speed,sizeof(float));
	
	HAL_StatusTypeDef status = HAL_OK;					//状态码
	
	uint16_t Write_Length = sizeof(Flash_Save);			//获取需写入的数据长度
	int* P_Flash_Save = (int*)&Flash_Save;				//指向Flash_Save的指针
	uint32_t Target_Address = DATA_SECTOR_ADDR;			//写入目标地址
	
	//计算写入字数，不满1字则补全1字
	if(Write_Length % 4 == 0)
		Write_Length /= 4;
	else
		Write_Length = (Write_Length / 4) + 1;
	
	uint32_t SectorError = 0;		//报错扇区存储区
	
    taskENTER_CRITICAL();	//进入临界区，关闭中断响应及任务调度
	HAL_FLASH_Unlock();		//解锁flash
	
	//扇区擦除操作结构体
    FLASH_EraseInitTypeDef EraseInitStruct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,		//擦除单个扇区
        .Sector = FLASH_SECTOR_11,					//擦除扇区为扇区11，STM32F047ZGT6单片机Flash的最后一个扇区
        .NbSectors = 1,								//擦除扇区的数量
        .VoltageRange = FLASH_VOLTAGE_RANGE_3 		//电源电压范围，为2.7V~3.6V
    };
	//擦除目标扇区
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {	//擦除失败，返回错误
        status = HAL_ERROR;
        goto exit;
    }
	
	//循环写入数据
	for(uint16_t i=0;i<Write_Length;i++)
	{	//以字长度（4字节）写入数据
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Target_Address,P_Flash_Save[i]) != HAL_OK)
		{	//写入失败，返回错误
            status = HAL_ERROR;
            goto exit;
		}
		Target_Address+=4;
	}
	
	exit:
	
	HAL_FLASH_Lock();		//锁定flash
    taskEXIT_CRITICAL();	//退出临界区，开启中断响应及任务调度
	
	return status;			//返回状态
}

/** @brief	Flash读数据，单片机首次下载程序时，不要运行该函数
  * @note	读取单片机最后一个扇区至Flash_Save结构体
  **/
void Flash_Read(void)
{
	//从起始地址拷贝sizeof(Flash_Save)个字节到Flash_Save结构体中
    memcpy(&Flash_Save,(const void*)DATA_SECTOR_ADDR, sizeof(Flash_Save));
	
	//数据拷贝出Flash_Save，以使用
	//灰度传感器数据
	memcpy(&Grayscale.Grayscale_ADC_Max,&Flash_Save.Grayscale_ADC_Max,sizeof(Grayscale.Grayscale_ADC_Max));
	memcpy(&Grayscale.Grayscale_ADC_Min,&Flash_Save.Grayscale_ADC_Min,sizeof(Grayscale.Grayscale_ADC_Min));
	memcpy(&Grayscale.Compute_Map_Mode,&Flash_Save.Compute_Map_Mode,sizeof(Grayscale.Compute_Map_Mode));
	memcpy(&Grayscale.Grayscale_ADC_Trigger_Threshold,&Flash_Save.Grayscale_ADC_Trigger_Threshold,sizeof(Grayscale.Grayscale_ADC_Trigger_Threshold));
	//舵机数据
	memcpy(&Servo_State,&Flash_Save.Servo_EN_State,sizeof(Servo_State));
	//运行数据
	memcpy(&Motor_Control_Parm.Base_High_Speed_Set,&Flash_Save.Base_High_Speed_Set,sizeof(float));
	memcpy(&Motor_Control_Parm.Accelerated_Speed,&Flash_Save.Accelerated_Speed,sizeof(float));
}
