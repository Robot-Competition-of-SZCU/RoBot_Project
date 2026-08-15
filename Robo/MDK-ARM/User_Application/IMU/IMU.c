///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为控制输出源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "IMU.h"

uint8_t IMU_data_Buffer[IMU_Buffer_Length];	//IMU数据缓存区

//IMU数据结构体
struct IMU IMU_Data;

/** @brief	IMU数据解析函数，解析陀螺仪Z轴数据
  **/
void IMU_GET_Data(void)
{
    short i,j;
	uint8_t SUM;					//检验数据
	uint8_t IMU_Buffer_Sec[10];		//IMU数据二级缓存区
	//加速度数据缓存区
	short IMU_Accel_X,IMU_Accel_Y,IMU_Accel_Z;
	//角速度数据缓存区
	short IMU_Gyro_X,IMU_Gyro_Y,IMU_Gyro_Z;
	//角度数据缓存区
	short IMU_Angle_X,IMU_Angle_Y,IMU_Angle_Z;
	
    //检索IMU数据缓存区
	for(i=0;i<IMU_Buffer_Length;i++)
	{	//包头数据0X55
		if(IMU_data_Buffer[i] == 0X55)
		{
			i++;
			if(i >= IMU_Buffer_Length) i = 0;			//溢出则回绕至开头
			
			//加速度计数据帧头0X51
			if(IMU_data_Buffer[i] == 0X51)
			{
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_data_Buffer[i];			//装载数据至二级缓存区
				}
				//进行校验计算
				SUM = 0x55 + 0x51;
				for(j=0;j<8;j++)
					SUM += IMU_Buffer_Sec[j];
				//校验通过
				if(SUM == IMU_Buffer_Sec[8])
				{	//搬运X轴加速度数据
					IMU_Accel_X = (short)IMU_Buffer_Sec[0];
					IMU_Accel_X |= ((short)IMU_Buffer_Sec[1] << 8);
					//搬运Y轴加速度数据
					IMU_Accel_Y = (short)IMU_Buffer_Sec[2];
					IMU_Accel_Y |= ((short)IMU_Buffer_Sec[3] << 8);
					//搬运Z轴加速度数据
					IMU_Accel_Z = (short)IMU_Buffer_Sec[4];
					IMU_Accel_Z |= ((short)IMU_Buffer_Sec[5] << 8);
					//单位换算及装载
					IMU_Data.IMU_Accel_X = ((float)IMU_Accel_X / 32768.0f) * 16.0f;
					IMU_Data.IMU_Accel_Y = ((float)IMU_Accel_Y / 32768.0f) * 16.0f;
					IMU_Data.IMU_Accel_Z = ((float)IMU_Accel_Z / 32768.0f) * 16.0f;
				}
				//不通过则不进行任何操作
			}
			//陀螺仪数据帧头0X52
			if(IMU_data_Buffer[i] == 0X52)
			{		
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_data_Buffer[i];			//装载数据至二级缓存区
				}
				//进行校验计算
				SUM = 0x55 + 0x52;
				for(j=0;j<8;j++)
					SUM += IMU_Buffer_Sec[j];
				//校验通过
				if(SUM == IMU_Buffer_Sec[8])
				{	//搬运X轴角速度数据
					IMU_Gyro_X = (short)IMU_Buffer_Sec[0];
					IMU_Gyro_X |= ((short)IMU_Buffer_Sec[1] << 8);
					//搬运Y轴角速度数据
					IMU_Gyro_Y = (short)IMU_Buffer_Sec[2];
					IMU_Gyro_Y |= ((short)IMU_Buffer_Sec[3] << 8);
					//搬运Z轴角速度数据
					IMU_Gyro_Z = (short)IMU_Buffer_Sec[4];
					IMU_Gyro_Z |= ((short)IMU_Buffer_Sec[5] << 8);
					//单位换算及装载
					IMU_Data.IMU_Gyro_X = ((float)IMU_Gyro_X / 32768.0f) * 2000.0f;
					IMU_Data.IMU_Gyro_Y = ((float)IMU_Gyro_Y / 32768.0f) * 2000.0f;
					IMU_Data.IMU_Gyro_Z = ((float)IMU_Gyro_Z / 32768.0f) * 2000.0f;
				}
				//不通过则不进行任何操作
			}
			//角度数据帧头0X53
			if(IMU_data_Buffer[i] == 0X53)
			{
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_data_Buffer[i];			//装载数据至二级缓存区
				}
				//进行校验计算
				SUM = 0x55 + 0x53;
				for(j=0;j<8;j++)
					SUM += IMU_Buffer_Sec[j];
				//校验通过
				if(SUM == IMU_Buffer_Sec[8])
				{	//搬运X轴角速度数据
					IMU_Angle_X = (short)IMU_Buffer_Sec[0];
					IMU_Angle_X |= ((short)IMU_Buffer_Sec[1] << 8);
					//搬运Y轴角速度数据
					IMU_Angle_Y = (short)IMU_Buffer_Sec[2];
					IMU_Angle_Y |= ((short)IMU_Buffer_Sec[3] << 8);
					//搬运Z轴角速度数据
					IMU_Angle_Z = (short)IMU_Buffer_Sec[4];
					IMU_Angle_Z |= ((short)IMU_Buffer_Sec[5] << 8);
					//单位换算及装载
					IMU_Data.IMU_Angle_X = ((float)IMU_Angle_X / 32768.0f) * 180.0f;
					IMU_Data.IMU_Angle_Y = ((float)IMU_Angle_Y / 32768.0f) * 180.0f;
					IMU_Data.IMU_Angle_Z = ((float)IMU_Angle_Z / 32768.0f) * 180.0f;
				}
				//不通过则不进行任何操作
				break;
			}
		}
	}
}

/** @brief	角度累计函数
  * @param	DT 	调控周期
  **/
void IMU_Z_Angel(float DT)
{
	IMU_Data.IMU_Angle_Z_Int += (IMU_Data.IMU_Gyro_Z * DT);
}

