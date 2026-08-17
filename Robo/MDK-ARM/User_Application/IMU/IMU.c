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
#include <string.h>

uint8_t IMU_data_Buffer[IMU_Buffer_Length];	//IMU数据缓存区
static uint8_t IMU_Angle_Z_First = 1;			//Z轴角度首帧有效标志

//IMU数据结构体
struct IMU IMU_Data;

/** @brief	IMU数据解析函数，解析陀螺仪Z轴数据
  **/
void IMU_GET_Data(void)
{
	//快照拷贝：DMA为循环模式持续写入IMU_data_Buffer，解析期间数据可能被覆盖造成帧撕裂
	//连续拷贝两份并比对，不一致则重试(每次拷贝仅微秒级，几乎立即一致)，不丢失本周期数据
	uint8_t IMU_Buffer_Snap_A[IMU_Buffer_Length];	//快照A
	uint8_t IMU_Buffer_Snap_B[IMU_Buffer_Length];	//快照B
	uint8_t Try = 0;								//重试计数
	do
	{
		memcpy(IMU_Buffer_Snap_A,IMU_data_Buffer,IMU_Buffer_Length);
		memcpy(IMU_Buffer_Snap_B,IMU_data_Buffer,IMU_Buffer_Length);
		Try++;
	}
	while(memcmp(IMU_Buffer_Snap_A,IMU_Buffer_Snap_B,IMU_Buffer_Length) != 0 && Try < 10);
	//重试上限兜底：极端情况下即使仍不一致也采用最后一份快照继续解析，保证本周期不丢数据

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
		if(IMU_Buffer_Snap_A[i] == 0X55)
		{
			i++;
			if(i >= IMU_Buffer_Length) i = 0;			//溢出则回绕至开头
			
			//加速度计数据帧头0X51
			if(IMU_Buffer_Snap_A[i] == 0X51)
			{
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_Buffer_Snap_A[i];			//装载数据至二级缓存区
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
			if(IMU_Buffer_Snap_A[i] == 0X52)
			{		
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_Buffer_Snap_A[i];			//装载数据至二级缓存区
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
			if(IMU_Buffer_Snap_A[i] == 0X53)
			{
				for(j=0;j<9;j++)
				{
					i++;
					if(i>=IMU_Buffer_Length) i-=IMU_Buffer_Length;  //数据缓存区长度为IMU_Buffer_Length，越界则回绕
					IMU_Buffer_Sec[j] = IMU_Buffer_Snap_A[i];			//装载数据至二级缓存区
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
					//校验数据是否正确
					float IMU_Angle_Compute_X = ((float)IMU_Angle_X / 32768.0f) * 180.0f;
					float IMU_Angle_Compute_Y = ((float)IMU_Angle_Y / 32768.0f) * 180.0f;
					float IMU_Angle_Compute_Z = ((float)IMU_Angle_Z / 32768.0f) * 180.0f;

					//数据在正常范围内则装载
					if(IMU_Angle_Compute_X < 90 && IMU_Angle_Compute_X > -90)
						IMU_Data.IMU_Angle_X = IMU_Angle_Compute_X;
					if(IMU_Angle_Compute_Y < 90 && IMU_Angle_Compute_Y > -90)
						IMU_Data.IMU_Angle_Y = IMU_Angle_Compute_Y;

					//Z轴正常范围校验
					if(IMU_Angle_Compute_Z <= 180 && IMU_Angle_Compute_Z >= -180)
					{
						//首帧有效数据直接装载，用于初始化增量限幅基准
						if(IMU_Angle_Z_First)
						{
							IMU_Data.IMU_Angle_Z = IMU_Angle_Compute_Z;
							IMU_Angle_Z_First = 0;
						}
						else
						{
							//相邻帧增量限幅：与上次有效值之差回绕归一化至-180~+180，超限视为坏数据丢弃
							float IMU_Angle_Diff_Z = IMU_Angle_Compute_Z - IMU_Data.IMU_Angle_Z;
							while(IMU_Angle_Diff_Z > 180) IMU_Angle_Diff_Z -= 360;
							while(IMU_Angle_Diff_Z < -180) IMU_Angle_Diff_Z += 360;
							if(IMU_Angle_Diff_Z < IMU_Angle_Z_Delta_Max &&
							   IMU_Angle_Diff_Z > -IMU_Angle_Z_Delta_Max)
								IMU_Data.IMU_Angle_Z = IMU_Angle_Compute_Z;
						}
					}
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

