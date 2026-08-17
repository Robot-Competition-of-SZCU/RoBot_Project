///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Aug 11th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为控制输出头文件
///////////////////////////////////////
#ifndef __IMU_H__
#define __IMU_H__

#define IMU_Buffer_Length 64		//缓存区长度
#define IMU_Angle_Z_Delta_Max 20.0f	//Z轴角度相邻帧最大允许增量，单位度

extern uint8_t IMU_data_Buffer[IMU_Buffer_Length];	//IMU数据缓存区

struct IMU
{
	float IMU_Accel_X;			//IMU加速度计X轴
	float IMU_Accel_Y;			//IMU加速度计Y轴
	float IMU_Accel_Z;			//IMU加速度计Z轴
	
	float IMU_Gyro_X;			//IMU角速度X轴
	float IMU_Gyro_Y;			//IMU角速度Y轴
	float IMU_Gyro_Z;			//IMU角速度Z轴
	
	float IMU_Angle_X;			//IMU_角度X轴
	float IMU_Angle_Y;			//IMU_角度Y轴
	float IMU_Angle_Z;			//IMU_角度Z轴
	
	float IMU_Angle_Z_Int;		//IMU螺仪累计角度
};
extern struct IMU IMU_Data;



void IMU_GET_Data(void);    	//IMU数据解析函数
void IMU_Z_Angel(float DT);		//角度累计函数


#endif
