#include "Gimbal.h"
#include "RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Detect.h"
#include <stdio.h>
#include "UserFreertos.h"
#include "beep.h"
#include "Shooter.h"
#include <math.h>
#include "Judge.h"
#define EN_GIMBAL_TASK // 使能任务

void Gimbal_InitPID(void);
void Gimbal_RegisterEvents(void);
void Gimbal_UpdataAngle(void);
void Gimbal_MouseCtrl(void);
void Gimbal_VisionCtrl(void);
void Gimbal_RockerCtrl(void);
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
extern VisionReceive vision_receive;
extern Vision_Type vision;
extern float chassAngle;
Gimbal gimbal = {0};
float visionFindAver;
float errorP=0;


/********************初始化************************/
// 初始化云台函数vscode://lirentech.file-ref-tags?filePath=Gimbal.c&snippet=%2F%2F+%E5%88%9D%E5%A7%8B%E5%8C%96%E4%BA%91%E5%8F%B0%E5%87%BD%E6%95%B0
void Gimbal_Init()
{
	// 设定pitch角度限幅
	gimbal.pitch.pitchMax = 32; //			2150	11000
	gimbal.pitch.pitchMin = -14.5;
	//PITCHLIMIT -> INITANGLE
	
	gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平 
	gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

	gimbal.yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
	gimbal.yaw.targetAngle = gimbal.yaw.initAngle;

	gimbal.visionEnable = false; // 自瞄默认关闭
	gimbal.rockerCtrl = false;	 // 默认使用键盘控制
	gimbal.fire = 0;

	gimbal.visionPitchIncLevel = 0; // pitch增益初值
	gimbal.manualYawOffset = 0;		// yaw偏移值

	Filter_InitAverFilter(&gimbal.Mouse.yawFilter, 10); // 键盘数据均值滤波 平滑鼠标数据
	Filter_InitAverFilter(&gimbal.Mouse.pitchFilter, 2);
	Filter_InitAverFilter(&gimbal.visionFilter.find, 25); // 视觉数据均值滤波

	Gimbal_InitPID();		 // 初始化PID参数
	Gimbal_RegisterEvents(); // 注册事件
}

/************内部工具函数*****************/
// 初始化PID参数

//void Gimbal_InitPID()
//{
//	/*pitch由陀螺仪控制*/
//	PID_Init(&gimbal.pitch.imuPID.inner, 10, 0.02, 2.5, 450, 2048); // 100 0.018 12180,2.3,25
//	DEPID_Init(&gimbal.pitch.imuPID.deOuter, 15, 0, 0.1, 0, 1000, 0.4);
//	/*yaw由陀螺仪控制*/

////	PID_Init(&gimbal.yaw.imuPID.inner, 2.6, 0, 0.05, 100000, 700000);
////	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 10000, 1, 600, 300000, 720000, 0.3);
//	PID_Init(&gimbal.yaw.imuPID.inner, 6, 0.035, 2, 550, 2048);  
//	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 20, 0, 2, 0, 1000, 0.4);  //20 0 2.5 0.4 1000
//}

//云台PID初始化函数vscode://lirentech.file-ref-tags?filePath=Gimbal.c&snippet=%2F%2F%E4%BA%91%E5%8F%B0PID%E5%88%9D%E5%A7%8B%E5%8C%96%E5%87%BD%E6%95%B0
void Gimbal_InitPID()
{
	/*pitch由陀螺仪控制*/
	PID_Init(&gimbal.pitch.imuPID.inner, 3, 0.04, 0.4, 500, 2048); // 100 0.018 12180,2.3,25
	DEPID_Init(&gimbal.pitch.imuPID.deOuter, 20, 0.00, 0.015, 50, 1000, 0.9);
	
	/*yaw由陀螺仪控制*/
	PID_Init(&gimbal.yaw.imuPID.inner, 0.005, 0, 0.0005, 1000, 7);  
	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 80, 0.0001, 0.5, 200, 1000, 0.5);  //20 0 2.5 0.4 1000
	
	//一套软的
//	PID_Init(&gimbal.yaw.imuPID.inner, 4.5, 0.03, 1.5, 1000, 2048);  
//	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 14, 0.05, 0.1, 400, 1000, 0.4);  //20 0 2.5 0.4 1000	
	
//		PID_Init(&gimbal.yaw.imuPID.inner, 10, 0.02, 2, 1000, 2048); 
//		DEPID_Init(&gimbal.yaw.imuPID.deOuter, 27, 0.01, 0, 100, 500, 1);  //20 0 2.5 0.4 1000	
}

void PitchLimit()
{
    static uint16_t initPitch = -21495;//need change
    float temp;

    temp = gimbal.pitchMotor.M4005.angle + (gimbal.pitch.targetAngle - gimbal.pitch.angle) / 360.f * 65536.f;//目标角度转换为电机单位
	//目标原始值套圈
    if (temp > 65536)
        temp -= 65536;
    else if (temp < 0)
        temp += 65536;

	//目标与initPitch套圈		//todo:为什么要与initPitch套圈？难道不是与电机当前角度套圈吗？因为目标角度是基于initPitch的绝对角度，而电机当前角度是基于initPitch的相对角度，所以需要与initPitch套圈来计算误差。
    if (temp - initPitch > 65536 / 2.0f)
        temp = temp - 65536;
    else if (temp - initPitch < -65536 / 2.0f)
        temp = temp + 65536;

	//计算误差
	errorP=temp - initPitch;
    if ((temp - initPitch) >= gimbal.pitch.pitchMax / 360.f * 65536.f)
    {
        temp = initPitch + gimbal.pitch.pitchMax / 360.f * 65536.f;
		//目标角度与电机角度套圈
        if (temp - gimbal.pitchMotor.M4005.angle > 65536 / 2.0f)
            temp = temp - 65536;
        else if (temp - gimbal.pitchMotor.M4005.angle < -65536 / 2.0f)
            temp = temp + 65536;
        gimbal.pitch.targetAngle = gimbal.pitch.angle + (temp - gimbal.pitchMotor.M4005.angle) / 65536.f * 360.f;
    }
    else if ((temp - initPitch) <= gimbal.pitch.pitchMin / 360.f * 65536.f)
    {

        temp = initPitch + gimbal.pitch.pitchMin / 360.f * 65536.f;
        if (temp - gimbal.pitchMotor.M4005.angle > 65536 / 2.0f)
            temp = temp - 65536;
        else if (temp - gimbal.pitchMotor.M4005.angle < -65536 / 2.0f)
            temp = temp + 65536;
        gimbal.pitch.targetAngle = gimbal.pitch.angle + (temp - gimbal.pitchMotor.M4005.angle) / 65536.f * 360.f;
    }

}


// 注册事件
void Gimbal_RegisterEvents()
{
	//  RC_Register(Key_R,CombineKey_Ctrl,KeyEvent_OnDown,Gimbal_Return_KeyCallback);//ctrl R 一键调头
}

// 更新陀螺仪角度、累积yaw角度、处理套圈后totalAngle和targetAngle
void Gimbal_UpdataAngle()
{
	gimbal.yaw.gyro = User_Imu_Get_Z_Gyro();
	gimbal.yaw.angle = User_Imu_GetYaw();
	gimbal.pitch.gyro = User_Imu_Get_Y_Gyro();
	gimbal.pitch.angle = User_Imu_GetPitch();
	float dAngle = 0;

	//处理套圈累加问题
	if (gimbal.yaw.angle - gimbal.yaw.lastAngle < -270)
		dAngle = gimbal.yaw.angle + (360 - gimbal.yaw.lastAngle);
	else if (gimbal.yaw.angle - gimbal.yaw.lastAngle > 270)
		dAngle = -gimbal.yaw.lastAngle - (360 - gimbal.yaw.angle);
	else
		dAngle = gimbal.yaw.angle - gimbal.yaw.lastAngle;
	//累加
	gimbal.yaw.totalAngle += dAngle;

	//处理套圈后totalAngle和targetAngle
	while(gimbal.yaw.targetAngle - gimbal.yaw.totalAngle >= 360 || gimbal.yaw.totalAngle - gimbal.yaw.targetAngle >= 360)
	{
			if (gimbal.yaw.targetAngle - gimbal.yaw.totalAngle >= 360)
			{
				gimbal.yaw.targetAngle = gimbal.yaw.targetAngle - 360;
			}
			if (gimbal.yaw.totalAngle - gimbal.yaw.targetAngle >= 360)
			{
				gimbal.yaw.targetAngle = gimbal.yaw.targetAngle + 360;
			}
	}

	gimbal.yaw.lastAngle = gimbal.yaw.angle;
}

// 鼠标控制云台
void Gimbal_MouseCtrl()
{
	gimbal.Mouse.yawDPI = 0.005;
	gimbal.Mouse.pitchDPI = 0.005;
	gimbal.yaw.targetAngle -= Filter_AverCalc(&gimbal.Mouse.yawFilter, rcInfo.mouse.x * gimbal.Mouse.yawDPI);		// yaw修改
	gimbal.pitch.targetAngle += Filter_AverCalc(&gimbal.Mouse.pitchFilter, rcInfo.mouse.y * gimbal.Mouse.pitchDPI); // pitch修改
	
}

//视觉控制云台
//todo:没看懂
void Gimbal_VisionCtrl()
{
	
	int yaw_cycle;	//todo:何意味
	if ((gimbal.yaw.targetAngle / 360.f) > 0)
		yaw_cycle = (gimbal.yaw.targetAngle / 360.f) + 0.5f;	//todo:0.5啥意思
	else
		yaw_cycle = (gimbal.yaw.targetAngle / 360.f) - 0.5f;
	if ((yaw_cycle * 360.f + vision.yaw) - gimbal.yaw.targetAngle > 180.f)
		gimbal.yaw.targetAngle = yaw_cycle * 360.f + vision.yaw - 360.f;
	else if ((yaw_cycle * 360.f + vision.yaw) - gimbal.yaw.targetAngle < -180.f)
		gimbal.yaw.targetAngle = yaw_cycle * 360.f + vision.yaw + 360.f;
	else
		gimbal.yaw.targetAngle = yaw_cycle * 360.f + vision.yaw;
	gimbal.pitch.targetAngle = vision.pitch;
	
}

// 摇杆控制云台
void Gimbal_RockerCtrl()
{
	gimbal.yaw.targetAngle -= rcInfo.ch1 * 0.35 / 660.0f;	// yaw
	gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
	
}

/************外部接口函数*******************/
// 视觉PC掉线回调
void Gimbal_VisionLostCallback()
{
//	gimbal.ui.visionState.color=Color_White;
//	Graph_DrawCircle(&gimbal.ui.visionState,Operation_Change);
//	Vision.found=0;
}

// 视觉恢复连接回调
void Gimbal_VisionRecoverCallback()
{
//	gimbal.ui.visionState.color = Color_Yellow;
//	Graph_DrawCircle(&gimbal.ui.visionState, Operation_Change);
}

///*************************RC事件**************************
// 以下任务受键鼠event调度
//*********************************************************/

// 一键调头
void Gimbal_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	Slope_SetTarget(&chassis.move.ySlope, 0);
	Slope_SetTarget(&chassis.move.xSlope, 0); // 刹车
	gimbal.yaw.targetAngle += 180;
}

/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/
// 云台控制任务



void Task_Gimbal_Callback()
{
	// 电管不给电后云台电机把各项变量清零，防止电管重启后云台乱转vscode://lirentech.file-ref-tags?filePath=Gimbal.c&snippet=%2F%2F+%E7%94%B5%E7%AE%A1%E4%B8%8D%E7%BB%99%E7%94%B5%E5%90%8E%E4%BA%91%E5%8F%B0%E7%94%B5%E6%9C%BA%E6%8A%8A%E5%90%84%E9%A1%B9%E5%8F%98%E9%87%8F%E6%B8%85%E9%9B%B6%EF%BC%8C%E9%98%B2%E6%AD%A2%E7%94%B5%E7%AE%A1%E9%87%8D%E5%90%AF%E5%90%8E%E4%BA%91%E5%8F%B0%E4%B9%B1%E8%BD%AC
	if (GameRobotStat.power_management_gimbal_output == 0) // 7.17
	{
		//PID变量清零
		PID_Clear(&gimbal.yaw.imuPID.inner);
		PID_Clear(&gimbal.pitch.imuPID.inner);
		DEPID_Clear(&gimbal.yaw.imuPID.deOuter);
		DEPID_Clear(&gimbal.pitch.imuPID.deOuter);
		gimbal.yaw.targetAngle = gimbal.yaw.angle;
		gimbal.yaw.imuPID.output = 0;	//陀螺仪温控输出清零
	}

	// 遥控器拨轮切换摇杆控制还是鼠标控制，以及开启自瞄vscode://lirentech.file-ref-tags?filePath=Gimbal.c&snippet=%2F%2F+%E9%81%A5%E6%8E%A7%E5%99%A8%E6%8B%A8%E8%BD%AE%E5%88%87%E6%8D%A2%E6%91%87%E6%9D%86%E6%8E%A7%E5%88%B6%E8%BF%98%E6%98%AF%E9%BC%A0%E6%A0%87%E6%8E%A7%E5%88%B6%EF%BC%8C%E4%BB%A5%E5%8F%8A%E5%BC%80%E5%90%AF%E8%87%AA%E7%9E%84
	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
		gimbal.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		gimbal.rockerCtrl = false;
	if (rcInfo.wheel > 600) // 遥控器模式下拨轮拨到底端开启自瞄
		gimbal.visionEnable = true;
	else if ((rcInfo.wheel < 600 && gimbal.rockerCtrl) || rcInfo.mouse.r != 1) // 遥控器模式下拨轮不在底端关闭自瞄
		gimbal.visionEnable = false;

	// 对识别状态进行滤波，防止偶尔的误识别vscode://lirentech.file-ref-tags?filePath=Gimbal.c&snippet=%2F%2F+%E5%AF%B9%E8%AF%86%E5%88%AB%E7%8A%B6%E6%80%81%E8%BF%9B%E8%A1%8C%E6%BB%A4%E6%B3%A2%EF%BC%8C%E9%98%B2%E6%AD%A2%E5%81%B6%E5%B0%94%E7%9A%84%E8%AF%AF%E8%AF%86%E5%88%AB
	visionFindAver = Filter_AverCalc(&gimbal.visionFilter.find, vision.control);


	if (gimbal.visionEnable && visionFindAver >= 0.5f && vision.yaw != 0) // 自瞄开启并识别成功，且能跑
	{
		Gimbal_VisionCtrl(); // 由视觉控制云台
	}
	else // 自瞄未开启或未识别到，由摇杆或鼠标控制
	{
		if (gimbal.rockerCtrl)
			Gimbal_RockerCtrl();
		else
			Gimbal_MouseCtrl();
	}
//			if(rcInfo.left==2)
//					gimbal.yaw.targetAngle = gimbal.yaw.targetAngle + 0.1f;
//			if(rcInfo.left==3)
//				gimbal.yaw.targetAngle=0;
//			if(rcInfo.left==1)
//				gimbal.yaw.targetAngle=10;
//			if(rcInfo.left==3)
//				gimbal.pitch.targetAngle=0;
//			if(rcInfo.left==1)
//				gimbal.pitch.targetAngle=5;
	// 更新陀螺仪角度
	Gimbal_UpdataAngle();
	PitchLimit();
	if (GameRobotStat.power_management_gimbal_output == 0 ) // 7.17
	{
		gimbal.yaw.targetAngle=gimbal.yaw.totalAngle;
		gimbal.yaw.gyro=0;
	}
	// 计算yaw电机输出
			//临时积分分离
//	if(fabs(gimbal.yaw.imuPID.deOuter.error) < 2)
//	{gimbal.yaw.imuPID.inner.ki = 0.2;
//		gimbal.yaw.imuPID.deOuter.ki =0;
//	}
//	else 
//	{gimbal.yaw.imuPID.inner.ki = 0.02;
//	gimbal.yaw.imuPID.deOuter.ki = 0;
//	}
	DEPID_CascadeCalc(&gimbal.yaw.imuPID, gimbal.yaw.targetAngle, gimbal.yaw.totalAngle, gimbal.yaw.gyro);
	// 计算pitch电输出
	DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
	
}

#ifdef EN_GIMBAL_TASK
void OS_GimbalCallback(void const *argument)
{
	osDelay(500);
	for (;;)
	{
		Task_Gimbal_Callback();
		osDelay(1);
	}
}
#endif
