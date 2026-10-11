///////////////////////////////////////
//	***	*** 
//	 *  **
//	***	*
//  write by Ideal_Fox
//  Affiliated to Suzhou City University
//  Revision made on Sep 21th, 2026
///////////////////////////////////////
//	文件介绍：
//		该文件为运行流程头文件
///////////////////////////////////////
#ifndef __RUN_Process_H__
#define __RUN_Process_H__



void RUN_System_Control(void);			//系统运行控制任务

//通用功能
void Go_Up_Platform(void);						//上平台
void Go_Up_Start_Platform(void);				//上出发平台
void Go_Up_2Step_High_Platform(void);			//上二阶高台
void Go_Down_Platform(void);					//下平台
void Go_Down_1Step_High_Platform(void);			//下一阶高台
void Go_Down_2Step_High_Platform(void);			//下二阶高台
void Hit_The_Scenic_Spot(void);					//撞景点
void Turn_Around_For_Platform(void);			//平台转向
void Cross_the_Mountain(void);					//翻越山
void Cross_the_Wave_Board(float Lenght);		//翻越波浪板

//前段行程
void Platform1_to_Platform2_on_Bridge(void);	//平台1至平台2,走桥
void Platform2_to_Scenic_Spot2(void);			//平台2至景点2
void Scenic_Spot2_to_Platform4(void);			//景点2至平台4
void Platform4_to_Scenic_Spot1(void);			//平台4至景点1
void Scenic_Spot1_to_Platform3(void);			//景点1至平台3

//通行检测
void Platform3_to_Test_A(void);					//平台3至A点
void Traffic_Sign_1_Test(void);					//通行指示牌1检测
void Traffic_Sign_1_Pass(void);					//通行指示牌1通过
void Traffic_Sign_2_Test(void);					//通行指示牌2检测
void Traffic_Sign_2_Pass(void);					//通行指示牌2通过
void Traffic_Sign_4_Test(void);					//通行指示牌4检测
void Traffic_Sign_4_Pass(void);					//通行指示牌4通过
void Traffic_Sign_3_Pass(void);					//通行指示牌3通过

//直接通过
void Traffic_Sign_1_Pass_Direct(void);			//1点直接通过
void Traffic_Sign_2_Pass_Direct(void);			//2点直接通过
void Traffic_Sign_3_Pass_Direct(void);			//3点直接通过
void Traffic_Sign_4_Pass_Direct(void);			//4点直接通过

//后段路程
void Test_C_to_Scenic_Spot4(void);				//C点至景点4
void Scenic_Spot4_to_Scenic_Spot5(void);		//景点4至景点5
void Scenic_Spot5_to_Test_D(void);				//景点5至D点

void Test_D_to_Platform5(void);					//D点至平台5
void Platform5_to_Platform7(void);				//平台5至平台7
void Platform7_to_Platform8(void);				//平台7至平台8
void Platform8_to_Scenic_Spot3(void);			//平台8至景点3
void Scenic_Spot3_to_Test_C(void);				//景点3至C点

//回家
void Aplha_to_Platform1(void);					//Alpha点至平台1
void GoHome_Way_1(void);						//1号点回家
void GoHome_Way_2(void);						//1号点回家
void GoHome_Way_3(void);						//1号点回家
void GoHome_Way_4(void);						//1号点回家

//后段行程合并
void Test_C_to_Test_D(void);		//C点至D点
void Test_D_to_Test_C(void);		//D点至C点


#endif


