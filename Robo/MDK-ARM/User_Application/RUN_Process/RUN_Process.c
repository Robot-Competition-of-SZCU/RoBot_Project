///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 21th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为运行流程源文件
///////////////////////////////////////
#include "stm32f4xx_hal.h"
#include "RUN_Process.h"
#include "cmsis_os.h"

#include "Grayscale_ADC.h"
#include "Control.h"
#include "IMU.h"
#include "Servo.h"
#include "UART.h"

/** @brief	系统运行控制任务
  **/
void RUN_System_Control(void)
{
	//运行状态下
    if(RUN_Parm.RUN_State)
    {
		//举手
		Servo_State.Servo_Angle[Servo5] = 180;
		Servo_State.Servo_Angle[Servo6] = 180;
		UART_Audio_Control(0);			//播报准备完毕
		osDelay(500);
		//放手
		Servo_State.Servo_Angle[Servo5] = 0;
		Servo_State.Servo_Angle[Servo6] = 0;
		
		 	if(RUN_Parm.Round == 0)						//第一回合
		 		UART_Visual_Identity_Send_Order('2');	//开启视觉识别

		//2号台
		Platform1_to_Platform2_on_Bridge();		//平台1至平台2
		UART_Audio_Control(2);			//播报到达2号平台
		Turn_Around_For_Platform();		//平台转向
		osDelay(1000);

			if(RUN_Parm.Round == 0)						//第一回合
				UART_Visual_Identity_Send_Order('2');	//进行文字识别

		//2号景点
		Platform2_to_Scenic_Spot2();	//平台2至景点2
		Word_Test_Recognition();		//文字识别
		Hit_The_Scenic_Spot();			//撞景点
		osDelay(1000);

		//4号平台
		Scenic_Spot2_to_Platform4();	//景点2至平台4
		UART_Audio_Control(4);			//播报到达4号平台
		osDelay(1000);
		Turn_Around_For_Platform();		//平台转向

		//1号景点
		Platform4_to_Scenic_Spot1();	//平台4至景点1
		Word_Test_Recognition();		//文字识别
		Hit_The_Scenic_Spot();			//撞景点
		osDelay(1000);

			if(RUN_Parm.Round == 0)						//第一回合
				UART_Visual_Identity_Send_Order('9');	//退出文字识别

		//3号平台
		Scenic_Spot1_to_Platform3();	//景点1至平台3
		UART_Audio_Control(3);			//播报到达3号平台
		osDelay(1000);
			if(RUN_Parm.Round == 0)						//第一回合
		 		UART_Visual_Identity_Send_Order('1');	//进入颜色识别
		Turn_Around_For_Platform();		//平台转向
			if(RUN_Parm.Round == 0)						//第一回合
		  		UART_Visual_Identity_Send_Order('7');	//保存白平衡

		//首个回合
		if(RUN_Parm.Round == 0)
		{
			//进行通行检测
			Platform3_to_Test_A();
			Traffic_Sign_1_Test();		//通道1测试
			//检测1
			if(Colour_Test_Recognition() == 1)
			{	
						UART_Visual_Identity_Send_Order('9');	//退出颜色识别

				Traffic_Sign_1_Pass();				//通道1通过
				Visual_Identity.Common_Traffic_Sign = 1;	//标记1号通道可通过
				goto Rounter2;						//前往阶段2
			}

			Traffic_Sign_2_Test();		//通道2测试
			//检测2
			if(Colour_Test_Recognition() == 1)
			{	
						UART_Visual_Identity_Send_Order('9');	//退出颜色识别

				Traffic_Sign_2_Pass();				//通道2通过
				Visual_Identity.Common_Traffic_Sign = 2;	//标记2号通道可通过
				goto Rounter2;						//前往阶段2
			}

			Traffic_Sign_4_Test();		//通道4测试
			//检测3
			if(Colour_Test_Recognition() == 1)
			{	
						UART_Visual_Identity_Send_Order('9');	//退出颜色识别

				Traffic_Sign_4_Pass();				//通道4通过
				Visual_Identity.Common_Traffic_Sign = 4;	//标记4号通道可通过
				goto Rounter2;						//前往阶段2
			}
						UART_Visual_Identity_Send_Order('9');	//退出颜色识别
			Traffic_Sign_3_Pass();		//通道3直接通过测试
		}
		else
		{
			//直接通过
			// //存在通行记录
			switch(Visual_Identity.Common_Traffic_Sign)
			{
				case 1: Traffic_Sign_1_Pass_Direct();		//通行指示牌1直接通过
						goto Rounter2;						//前往阶段2
				case 2:	Traffic_Sign_2_Pass_Direct();		//通行指示牌2直接通过
						goto Rounter2;						//前往阶段2
				case 3: Traffic_Sign_2_Pass_Direct();		//通行指示牌3直接通过
						goto Rounter2;						//前往阶段2
				case 4: Traffic_Sign_2_Pass_Direct();		//通行指示牌4直接通过
						goto Rounter2;						//前往阶段2
			}
		}

		//阶段2
		Rounter2:
						UART_Visual_Identity_Send_Order('2');	//进行文字识别

		//进入后段路程
		switch(Visual_Identity.Common_Traffic_Sign)
		{
			//可通过路口为1
			case 1:	
			case 3:
					Test_C_to_Test_D();				//C点至D点

					Car_Arc_Turn_Control(Turn_Right,90,3);	//行进中右转90

					Test_D_to_Test_C();				//D点至C点
					break;
			case 2:
			case 4:
					Test_D_to_Test_C();				//D点至C点

					Car_Arc_Turn_Control(Turn_Right,90,3);	//行进中右转90

					Test_C_to_Test_D();				//C点至D点
					break;
		}

		//回家
		switch(Visual_Identity.Common_Traffic_Sign)
		{
			case 1:	Traffic_Sign_1_Pass_Direct();	//1号线路回家
					break;
			case 2:	Traffic_Sign_2_Pass_Direct();	//2号线路回家
					break;
			case 3: Traffic_Sign_3_Pass_Direct();	//3号线路回家
					break;
			case 4:	Traffic_Sign_4_Pass_Direct();	//4号线路回家
					break;
		}
		Turn_Around_For_Platform();		//平台转向

		UART_Audio_Control(1);			//播报到达1号平台
		//osDelay(2000);

		//停止
		RUN_Parm.RUN_State = 0;
		RUN_Parm.Round++;					//回合累计
	}
}

/** @brief	上平台
  **/
void Go_Up_Platform(void)
{	
	Wait_Side_Sensor_Trigger();		//等待光电传感器触发

	RUN_Speed_Set(2);			//速度设置

	Wait_X_Angle(Greater,15);	//等待X轴角度到达15度

	Wait_Grayscale_State(No_Trigger);	//等待灰度无触发
	Get_Now_Z_Angle(0);			//获取当前角度
	
	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_Microswitch_Trigger();	//等待两侧微动开关触发

	osDelay(50);				//等待撞击

	Car_Mode(Car_Stop);			//停止
	osDelay(100);				//等待停止

	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,1,0,4,End_Finish_Deceleration);

	Car_Mode(Car_Stop);				//停止
}

/** @brief	上出发平台
  **/
void Go_Up_Start_Platform(void)
{
	Wait_Side_Sensor_Trigger();		//等待光电传感器触发

	RUN_Speed_Set(2);			//速度设置

	Wait_X_Angle(Greater,15);	//等待X轴角度到达15度

	Wait_Grayscale_State(No_Trigger);	//等待灰度无触发
	Get_Now_Z_Angle(0);			//获取当前角度
	
	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,0,50,End_Finish_Deceleration);

	Car_Mode(Car_Stop);				//停止
}


/** @brief	上二阶高台
  **/
void Go_Up_2Step_High_Platform(void)
{	
	Wait_Side_Sensor_Trigger();			//等待光电传感器触发

	RUN_Speed_Set(2);					//速度设置

	Wait_X_Angle(Greater,15);			//等待X轴角度到达15度

	Wait_Grayscale_State(No_Trigger);	//等待灰度无触发
	Get_Now_Z_Angle(0);					//获取当前角度
	
	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,2,2,10,End_Finish_Deceleration);

	Wait_Grayscale_State(Trigger);		//等待灰度触发

	//巡线控制前进10
	Scan_Line_RUN_Control(2,2,10,End_Finish_Deceleration);

	Wait_X_Angle(Greater,15);			//等待X轴角度到达15度

	Wait_Grayscale_State(No_Trigger);	//等待灰度无触发
	Get_Now_Z_Angle(0);					//获取当前角度

	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_Microswitch_Trigger();	//等待两侧微动开关触发

	osDelay(50);				//等待撞击

	Car_Mode(Car_Stop);			//停止
	osDelay(100);				//等待停止

	//角度跟随后退8
	Angle_Following_RUN_Control(RUN_Retreat,1,0,4,End_Finish_Deceleration);

	Car_Mode(Car_Stop);				//停止
}

/** @brief	下平台
  **/
void Go_Down_Platform(void)
{
	Servo_State.Servo_Angle[Servo2] = 10;	//降下前瞻

	Get_Now_Z_Angle(0);						//获取当前角度

	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_Grayscale_State(Trigger);			//等待灰度触发

	//巡线控制,
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);

	Wait_X_Angle(Less,-15);					//等待X轴角度小于-15度

	Wait_X_Angle(Greater,-5);				//等待X轴角度大于-5度

	Servo_State.Servo_Angle[Servo2] = 35;	//抬起前瞻
}

/** @brief	下一阶高台
  **/
void Go_Down_1Step_High_Platform(void)
{
	Servo_State.Servo_Angle[Servo2] = 10;	//降下前瞻

	Get_Now_Z_Angle(0);						//获取当前角度

	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_Grayscale_State(Trigger);			//等待灰度触发

	//巡线控制,
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);

	Wait_X_Angle(Less,-10);					//等待X轴角度小于-10度

	//加速至2
	RUN_Speed_Set(2);

	Wait_X_Angle(Greater,-5);				//等待X轴角度大于-5度

	Servo_State.Servo_Angle[Servo2] = 35;	//抬起前瞻
}

/** @brief	下二阶高台
  **/
void Go_Down_2Step_High_Platform(void)
{
	Servo_State.Servo_Angle[Servo2] = 10;	//降下前瞻

	Get_Now_Z_Angle(0);						//获取当前角度

	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_Grayscale_State(Trigger);			//等待灰度触发

	//巡线控制,
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);

	Wait_X_Angle(Less,-15);					//等待X轴角度小于-15度

	//加速至2
	RUN_Speed_Set(2);

	Wait_Grayscale_State(No_Trigger);			//等待灰度无触发

	//角度跟随前进10
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_X_Angle(Greater,-5);				//等待X轴角度大于-5度

	Wait_Grayscale_State(Trigger);			//等待灰度触发

	//巡线控制,
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);

	Wait_X_Angle(Less,-15);					//等待X轴角度小于-15度

	//加速至2
	RUN_Speed_Set(2);

	Servo_State.Servo_Angle[Servo2] = 35;	//抬起前瞻
}

/** @brief	撞景点
  **/
void Hit_The_Scenic_Spot(void)
{
	Get_Now_Z_Angle(0);						//获取当前角度

	//巡线控制,
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);
	
	Wait_Microswitch_Trigger();				//等待两侧微动开关触发

	//角度跟随后退8
	Angle_Following_RUN_Control(RUN_Retreat,1.2,0,8,End_Finish_Deceleration);

	Car_Mode(Car_Stop);				//停止
}

/** @brief	平台转向
  **/
void Turn_Around_For_Platform(void)
{
	Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

	//举手
	Servo_State.Servo_Angle[Servo5] = 180;
	osDelay(200);									//等待200ms
	Servo_State.Servo_Angle[Servo6] = 180;
	osDelay(200);									//等待200ms

	Car_Turn_Control(Turn_Left,180,3);				//左转180度

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	//放手
	Servo_State.Servo_Angle[Servo5] = 0;
	osDelay(200);									//等待200ms
	Servo_State.Servo_Angle[Servo6] = 0;
	osDelay(200);									//等待200ms
}

/** @brief	平台转向，8号台特调
  **/
void Turn_Around_For_Platform_8(void)
{
		Servo_State.Servo_Angle[Servo2] = 90;			//舵机角度设定

	//举手
	Servo_State.Servo_Angle[Servo5] = 180;
	osDelay(200);									//等待200ms
	Servo_State.Servo_Angle[Servo6] = 180;
	osDelay(200);									//等待200ms

	Car_Turn_Control(Turn_Left,180,1.5);			//左转180度

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定

	//放手
	Servo_State.Servo_Angle[Servo5] = 0;
	osDelay(200);									//等待200ms
	Servo_State.Servo_Angle[Servo6] = 0;
	osDelay(200);	
}

/** @brief	翻越山
  **/
void Cross_the_Mountain(void)
{
	//Wait_Side_Sensor_Trigger();		//等待光电传感器触发
	Wait_Middle_Sensor_Trigger();		//等待中部光电传感器触发
	RUN_Speed_Set(2);			//速度设置
	
	Get_Now_Z_Angle(0);			//获取当前角度

	Servo_State.Servo_Angle[Servo2] = 90;	//抬起前瞻

	//角度跟随前进15
	Angle_Following_RUN_Control(RUN_Advance,1,1,0,End_Finish_Deceleration);

	Wait_X_Angle(Greater,15);	//等待姿态大于15度，上坡
	Wait_X_Angle(Less,5);		//等待姿态小于10度，上至坡顶

	Servo_State.Servo_Angle[Servo2] = 15;	//降下前瞻

	Wait_Grayscale_State(Trigger);			//等待灰度触发	

	//巡线控制,行进10
	Scan_Line_RUN_Control(1.5,1.5,0,End_Finish_Deceleration);

	Wait_X_Angle(Less,-15);		//等待姿态大于15度，下坡
	Wait_X_Angle(Greater,-5);	//等待姿态大于-5度，下至坡底

	Servo_State.Servo_Angle[Servo2] = 35;			//舵机角度设定
}

/** @brief	翻越波浪板
  * @param	Lenght	翻越长度
  **/
void Cross_the_Wave_Board(float Lenght)
{
	Wait_Side_Sensor_Trigger();		//等待光电传感器触发

	//巡线控制,行进Lenght
	Scan_Line_RUN_Control(1,1,Lenght + 40,End_Finish_Deceleration);
}


/** @brief	平台1至平台2,走桥
  **/
void Platform1_to_Platform2_on_Bridge(void)
{
	Go_Down_Platform();			//下平台

	//巡线控制,行进
	Scan_Line_RUN_Control(2.5,2.5,20,End_Finish_Deceleration);

	Wait_Side_Sensor_Trigger();		//等待两侧光电传感器触发

	Get_Now_Z_Angle(0);			//获取当前角度,左偏移

	//角度跟随前进15
	Angle_Following_RUN_Control(RUN_Advance,1.5,1.5,10,End_Finish_Deceleration);

	Wait_X_Angle(Less,5);	//等待姿态小于5度
	

	//角度跟随前进90
	Angle_Following_RUN_Control(RUN_Advance,2,2,90,End_Finish_Deceleration);

	Servo_State.Servo_Angle[Servo2] = 10;	//降下前瞻

	Wait_X_Angle(Less,-10);					//等待姿态小于-15度

	RUN_Speed_Set(2.5);

	Wait_Grayscale_State(Trigger);			//等待灰度传感器触发

	//巡线控制
	Scan_Line_RUN_Control(2,2,0,End_Finish_Deceleration);

	Wait_X_Angle(Greater,-5);				//等待姿态大于-5度
	Servo_State.Servo_Angle[Servo2] = 35;	//抬起前瞻

	//巡线控制,行进50
	Scan_Line_RUN_Control(4,3,50,End_Finish_Deceleration);

	Go_Up_Platform();						//上平台
}

/** @brief	平台2至景点2
  **/
void Platform2_to_Scenic_Spot2(void)
{
	Go_Down_Platform();				//下平台

	RUN_Speed_Set(3);				//速度设置

	Wait_Disperse_Intersection();	//等待到达分散路口

	//行进中右转
	Car_Arc_Turn_Control(Turn_Right,40,3);			
	
	//巡线控制
	Scan_Line_RUN_Control(2,2,0,End_Finish_Deceleration);

	Wait_Appoint_Line(1);			//等待只存在单线
	
	Cross_the_Mountain();			//翻越山

	//巡线控制
	Scan_Line_RUN_Control(3,3,0,End_Finish_Deceleration);

	Wait_Join_Intersection();		//等待到达交汇路口

	//行进中左转
	Car_Arc_Turn_Control(Turn_Left,25,3);			

	//巡线控制
	Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	//巡线控制
	Scan_Line_RUN_Control(5,3,150,End_Finish_Deceleration);

	Wait_Join_Intersection();		//等待到达交汇路口

	//行进中左转
	Car_Arc_Turn_Control(Turn_Left,25,3);	

	//巡线控制
	Scan_Line_RUN_Control(5,3,80,End_Finish_Deceleration);

	Wait_Middle_Sensor_Trigger();	//等待中部光电传感器触发

	Get_Now_Z_Angle(0);				//获取当前Z轴角度

	RUN_Speed_Set(0);				//停止
}

/** @brief	景点2至平台4
  **/
void Scenic_Spot2_to_Platform4(void)
{
	//角度跟随后退80
	Angle_Following_RUN_Control(RUN_Retreat,5,3,80,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//停止
	RUN_Speed_Set(0);
	osDelay(150);

	//巡线控制前进
	Scan_Line_RUN_Control(2,2,15,End_Finish_Deceleration);
	
	Car_Arc_Turn_Control(Turn_Right,25,3);			//行进中右转

	//巡线控制前进50
	Scan_Line_RUN_Control(5,3,40,End_Finish_Deceleration);

	Go_Up_Platform();								//上平台
}

/** @brief	平台4至景点1
  **/
void Platform4_to_Scenic_Spot1(void)
{
	Go_Down_Platform();								//下平台

	//巡线控制前进50
	Scan_Line_RUN_Control(3,3,50,End_Finish_Deceleration);

	//巡线控制前进500
	Scan_Line_RUN_Control(8,5,450,End_Finish_Deceleration);
	Scan_Line_RUN_Control(5,3,50,End_Finish_Deceleration);

	//等待到达左直角路口
	Wait_Appoint_Intersection(Left_Right_angle_Intersection);

	//弧线转弯
	Car_Arc_Turn_Control(Turn_Right,20,3);

	//巡线控制前进100
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	Wait_Middle_Sensor_Trigger();	//等待中部光电传感器触发

	Get_Now_Z_Angle(-2);				//获取当前Z轴角度

	RUN_Speed_Set(0);				//停止
}

/** @brief	景点1至平台3
  **/
void Scenic_Spot1_to_Platform3(void)
{
	//角度跟随后退100
	Angle_Following_RUN_Control(RUN_Retreat,5,3,100,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//停止
	RUN_Speed_Set(0);
	osDelay(150);

	//巡线控制前进
	Scan_Line_RUN_Control(2,2,15,End_Finish_Deceleration);

	//弧线转弯
	Car_Arc_Turn_Control(Turn_Left,20,3);

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	Go_Up_Platform();								//上平台
}

/** @brief	平台3至A点
  **/
void Platform3_to_Test_A(void)
{
	Go_Down_Platform();								//下平台

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,150,End_Finish_Deceleration);
	
	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);
}

/** @brief	通行指示牌1检测
  **/
void Traffic_Sign_1_Test(void)
{
	Car_Arc_Turn_Control(Turn_Right,90,3);	//行进中右转90度

	//巡线控制前进
	Scan_Line_RUN_Control(4,2,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
	Get_Now_Z_Angle(0);						//获取当前Z轴角度

	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	Car_Turn_Control(Turn_Right,90,3);		//右转90度
	osDelay(100);							//等待车身稳定
}

/** @brief	通行指示牌1通过
  **/
void Traffic_Sign_1_Pass(void)
{
	Car_Turn_Control(Turn_Left,90,3);			//左转90度

	//巡线控制前进
	Scan_Line_RUN_Control(4,3,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
}

/** @brief	通行指示牌2检测
  **/
void Traffic_Sign_2_Test(void)
{
	Car_Turn_Control(Turn_Left,90,3);			//左转90度
	osDelay(100);							//等待车身稳定

	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,4,2,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
	
	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	//巡线控制前进
	Scan_Line_RUN_Control(2,2,5,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Left,45,3);			//左转

	//巡线控制前进
	Scan_Line_RUN_Control(4,2,20,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
	Get_Now_Z_Angle(0);						//获取当前Z轴角度

	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	Car_Turn_Control(Turn_Right,90,3);		//右转90度
	osDelay(100);							//等待车身稳定
}

/** @brief	通行指示牌2通过
  **/
void Traffic_Sign_2_Pass(void)
{
	Car_Turn_Control(Turn_Left,90,3);					//左转

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,150,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中左转
}

/** @brief	通行指示牌4检测
  **/
void Traffic_Sign_4_Test(void)
{
	Car_Turn_Control(Turn_Left,90,3);					//左转90度
	osDelay(100);									//等待车身稳定

	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,4,2,30,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,5,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Left,50,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,200,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(4,2,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
	Get_Now_Z_Angle(0);						//获取当前Z轴角度

	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	Car_Turn_Control(Turn_Right,90,3);		//右转90度
	osDelay(100);							//等待车身稳定
}

/** @brief	通行指示牌4通过
  **/
void Traffic_Sign_4_Pass(void)
{
	Car_Turn_Control(Turn_Left,90,3);					//左转90度
	osDelay(100);									//等待车身稳定

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转
}

/** @brief	通行指示牌3通过
  **/
void Traffic_Sign_3_Pass(void)
{
	Car_Turn_Control(Turn_Left,90,3);				//左转90度
	osDelay(100);									//等待车身稳定

	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,4,2,30,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	//停止
	Car_Mode(Car_Stop);
	osDelay(150);

	//巡线控制前进
	//Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Right,30,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,250,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中右转
}

/** @brief	1点直接通过
  **/
void Traffic_Sign_1_Pass_Direct(void)
{
	//平台3至A点
	Platform3_to_Test_A();

	Car_Arc_Turn_Control(Turn_Right,90,3);	//行进中右转90度

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,170,End_Finish_Deceleration);
}

/** @brief	2点直接通过
  **/
void Traffic_Sign_2_Pass_Direct(void)
{
	//平台3至A点
	Platform3_to_Test_A();

	Car_Arc_Turn_Control(Turn_Right,40,3);	//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,270,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,35,3);			//行进中左转
}

/** @brief	3点直接通过
  **/
void Traffic_Sign_3_Pass_Direct(void)
{
	//平台3至A点
	Platform3_to_Test_A();

	//巡线控制前进
	Scan_Line_RUN_Control(6,3,200,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,10,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,130,3);	//右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,270,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,45,3);			//行进中左转
}

/** @brief	4点直接通过
  **/
void Traffic_Sign_4_Pass_Direct(void)
{
	//平台3至A点
	Platform3_to_Test_A();

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,200,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,170,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转
}

/** @brief	C点至景点4
  **/
void Test_C_to_Scenic_Spot4(void)
{
	//巡线控制前进
	Scan_Line_RUN_Control(3,3,15,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,90,3);				//右转

	Get_Now_Z_Angle(0);						//获取当前Z轴角度
}

/** @brief	景点4至景点5
  **/
void Scenic_Spot4_to_Scenic_Spot5(void)
{
	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,1.5,1.5,0,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	//停止
	Car_Mode(Car_Stop);
	osDelay(100);

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,5,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	//等待到达左直角路口
	Wait_Appoint_Intersection(Left_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,200,End_Finish_Deceleration);

	//等待到达左直角路口
	Wait_Appoint_Intersection(Left_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	//等待到达左直角路口
	Wait_Appoint_Intersection(Left_Right_angle_Intersection);

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Left,90,3);				//右转

	Get_Now_Z_Angle(0);						//获取当前Z轴角度
}

/** @brief	景点5至D点
  **/
void Scenic_Spot5_to_Test_D(void)
{
	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,1.5,1.5,0,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	//停止
	Car_Mode(Car_Stop);
	osDelay(100);

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,5,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
}

/** @brief	D点至平台5
  **/
void Test_D_to_Platform5(void)
{
	//巡线控制前进
	Scan_Line_RUN_Control(5,3,200,End_Finish_Deceleration);

	Go_Up_Platform();								///上平台
}

/** @brief	平台5至平台7
  **/
void Platform5_to_Platform7(void)
{
	Go_Down_Platform();								//下平台

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//等待到达分散路口
	Wait_Disperse_Intersection();

	Car_Arc_Turn_Control(Turn_Left,30,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,80,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,10,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Left,140,3);				//左转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,80,End_Finish_Deceleration);

	//翻越山
	Cross_the_Mountain();

	//巡线控制前进
	Scan_Line_RUN_Control(4,3,40,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,0,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,80,End_Finish_Deceleration);

	//翻越山
	Cross_the_Mountain();

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,0,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//翻越波浪板
	Cross_the_Wave_Board(120);

	//巡线控制前进
	Scan_Line_RUN_Control(2,2,0,End_Finish_Deceleration);

	//上平台
	Go_Up_Platform();
}

/** @brief	平台7至平台8
  **/
void Platform7_to_Platform8(void)
{
	//下一阶高台
	Go_Down_1Step_High_Platform();

	//翻越波浪板
	Cross_the_Wave_Board(120);

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,60,End_Finish_Deceleration);

	//翻越山
	Cross_the_Mountain();

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	//上二阶高台
	Go_Up_2Step_High_Platform();
}

/** @brief	平台8至景点3
  **/
void Platform8_to_Scenic_Spot3(void)
{
	//下二阶高台
	Go_Down_2Step_High_Platform();

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,0,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Left,140,3);		//左转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,120,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Left,120,3);		//左转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,80,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,100,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,90,3);				//右转

	Get_Now_Z_Angle(0);						//获取当前Z轴角度
}

/** @brief	景点3至C点
  **/
void Scenic_Spot3_to_Test_C(void)
{
	//角度跟随后退
	Angle_Following_RUN_Control(RUN_Retreat,1.5,1.5,0,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	//停止
	Car_Mode(Car_Stop);
	osDelay(100);

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,5,End_Finish_Deceleration);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,10,End_Finish_Deceleration);

	//等待到达右直角路口
	Wait_Appoint_Intersection(Right_Right_angle_Intersection);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,150,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);
}

/** @brief	Alpha点至出发平台
  **/
void Aplha_to_Platform1(void)
{
	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//翻越波浪板
	Cross_the_Wave_Board(60);

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Right,40,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	Go_Up_Start_Platform();
}

/** @brief	路线1回家
  **/
void GoHome_Way_1(void)
{
	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,160,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	Car_Arc_Turn_Control(Turn_Right,90,3);			//行进中右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,150,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中左转

	//Alpha点至平台1
	Aplha_to_Platform1();
}

/** @brief	路线3回家
  **/
void GoHome_Way_3(void)
{
	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,270,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,10,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Left,140,3);				//左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,10,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,140,3);				//右转

	//Alpha点至平台1
	Aplha_to_Platform1();
}

/** @brief	路线2回家
  **/
void GoHome_Way_2(void)
{
	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,30,End_Finish_Deceleration);

	//巡线控制前进
	Scan_Line_RUN_Control(8,3,270,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(3,0,10,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,140,3);				//右转

	//巡线控制前进
	Scan_Line_RUN_Control(5,3,130,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	Car_Arc_Turn_Control(Turn_Left,40,3);			//行进中左转

	//Alpha点至平台1
	Aplha_to_Platform1();
}

/** @brief	路线4回家
  **/
void GoHome_Way_4(void)
{
	//巡线控制前进
	Scan_Line_RUN_Control(8,3,200,End_Finish_Deceleration);

	//等待到达十字路口
	Wait_Appoint_Intersection(Crossroad);

	Car_Arc_Turn_Control(Turn_Left,90,3);			//行进中左转

	//巡线控制前进
	Scan_Line_RUN_Control(3,3,20,End_Finish_Deceleration);

	//等待到达交汇路口
	Wait_Join_Intersection();

	//巡线控制前进
	Scan_Line_RUN_Control(2,0,15,End_Finish_Deceleration);

	Car_Turn_Control(Turn_Right,140,3);				//右转

	//Alpha点至平台1
	Aplha_to_Platform1();
}

/** @brief	C点至D点
  * @note   起始地点需正对着4号景点
  * @note   结束地点为正对着1号平台
  **/
void Test_C_to_Test_D(void)
{
	Test_C_to_Scenic_Spot4();		//C点至景点4
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点
	osDelay(1000);
	Scenic_Spot4_to_Scenic_Spot5();	//景点4至景点5
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点
	osDelay(1000);
	Scenic_Spot5_to_Test_D();		//景点5至D点
}

/** @brief	D点至C点
  * @note   起始地点需正对着5号平台
  * @note   结束地点为正对着D点
  **/
void Test_D_to_Test_C(void)
{
	//5号平台
	Test_D_to_Platform5();			//D点至平台5
	UART_Audio_Control(5);			//播报到达5号平台
	osDelay(1000);
	Turn_Around_For_Platform();		//平台转向

	//7号平台
	Platform5_to_Platform7();		//平台5至平台7
	UART_Audio_Control(7);			//播报到达7号平台
	osDelay(1000);
	Turn_Around_For_Platform();		//平台转向

	//8号平台
	Platform7_to_Platform8();		//平台7至平台8
	UART_Audio_Control(8);			//播报到达8号平台
	osDelay(1000);
	Turn_Around_For_Platform_8();	//平台转向

	//3号景点
	Platform8_to_Scenic_Spot3();	//平台8至景点3
	Word_Test_Recognition();		//文字识别
	Hit_The_Scenic_Spot();			//撞景点
	osDelay(1000);

	Scenic_Spot3_to_Test_C();		//景点3至C点
}

