///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on July 8th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为按键控制读取底层驱动源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "KEY.h"

//按键长按触发时长判断
#define KEY_Long_Time_Tigger_Set	50		//长按判定时长为1000ms

KEY_Tigger_State KEY_Num[9];	//按键键码缓存区

/** @brief	按键轮询函数，轮询间隔为20ms
  **/
void KEY_Polling_Function(void)
{
	static short KEY_Tigger_Time[9];	//触发计时
	char i = 0;
	//读取KEY1，端口PG0
	if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取KEY2，端口PG1
	if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_1) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_1) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取KEY3，端口PE7
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取KEY4，端口PB12
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取摇杆_上，端口PF11
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_11) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_11) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取摇杆_右，端口PF12
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_12) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_12) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取摇杆_下，端口PF13
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取摇杆_中，端口PF14
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_14) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_14) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
	i++;

	//读取摇杆_左，端口PF15
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15) == 0)
		KEY_Tigger_Time[i]++;
	if (KEY_Tigger_Time[i] > 0)
	{	
		if(KEY_Tigger_Time[i] >= KEY_Long_Time_Tigger_Set)	//触发计时大于等于长按触发
			KEY_Num[i] = long_Tigger;						//键码持续置long_Tigger
		
		//按键松开
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15) == 1)
		{
			if(KEY_Tigger_Time[i] < KEY_Long_Time_Tigger_Set)//触发计时小于长按触发
				KEY_Num[i] = short_Tigger;					//键码置short_Tigger
			KEY_Tigger_Time[i] = 0;			//复位触发计时
		}
	}
}

/** @brief	获取按键状态
  * @param	KEY_X	选择需获取的按键
  * @retval	返回按键状态值
  * @note	获取按键键码后，原键码缓存区将被复位为无触发
  **/
KEY_Tigger_State Get_Key_State(KEY_Encode KEY_X)
{
	KEY_Tigger_State KEY_Num_Buffer;
	KEY_Num_Buffer = KEY_Num[KEY_X];
	KEY_Num[KEY_X] = NO_Tigger;
	return (KEY_Tigger_State)KEY_Num_Buffer;
}

/** @brief	复位所有键码状态
  **/
void Key_State_Clear(void)
{
	for(char i=0;i<9;i++)
		KEY_Num[i] = NO_Tigger;
}


