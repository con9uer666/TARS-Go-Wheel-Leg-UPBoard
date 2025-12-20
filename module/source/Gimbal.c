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
#include "SolveTrajectory.h"
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
extern struct SolveTrajectoryParams st;
extern struct tar_pos tar_position[4];
Gimbal gimbal = {0};
float visionFindAver;
float errorP=0;


/********************初始化************************/
// 初始化云台
void Gimbal_Init()
{
	gimbal.pitch.pitchMax = 32; // 设定pitch角度限幅			2150	11000
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

void Gimbal_InitPID()
{
	/*pitch由陀螺仪控制*/
	PID_Init(&gimbal.pitch.imuPID.inner, 7.5, 0.03, 0.33, 500, 2048); // 100 0.018 12180,2.3,25
	DEPID_Init(&gimbal.pitch.imuPID.deOuter, 35, 0.01, 0.015, 200, 1000, 0.9);
	
	/*yaw由陀螺仪控制*/
	PID_Init(&gimbal.yaw.imuPID.inner, 5.8, 0.02, 3.5, 1000, 2048);  
	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 24, 0.012, 0.15, 200, 1000, 0.45);  //20 0 2.5 0.4 1000
	
	//一套软的
//	PID_Init(&gimbal.yaw.imuPID.inner, 4.5, 0.03, 1.5, 1000, 2048);  
//	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 14, 0.05, 0.1, 400, 1000, 0.4);  //20 0 2.5 0.4 1000	
	
//		PID_Init(&gimbal.yaw.imuPID.inner, 10, 0.02, 2, 1000, 2048); 
//		DEPID_Init(&gimbal.yaw.imuPID.deOuter, 27, 0.01, 0, 100, 500, 1);  //20 0 2.5 0.4 1000	
}

void PitchLimit()
{
    static uint16_t initPitch = 4700;//need change
    float temp;
    temp = gimbal.pitchMotor.M4005.angle + (gimbal.pitch.targetAngle - gimbal.pitch.angle) / 360.f * 65536.f;
    if (temp > 65536)
        temp -= 65536;
    else if (temp < 0)
        temp += 65536;

    if (temp - initPitch > 65536 / 2.0f)
        temp = temp - 65536;
    else if (temp - initPitch < -65536 / 2.0f)
        temp = temp + 65536;
		errorP=temp - initPitch;
    if ((temp - initPitch) >= gimbal.pitch.pitchMax / 360.f * 65536.f)
    {
        temp = initPitch + gimbal.pitch.pitchMax / 360.f * 65536.f;
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
	 RC_Register(Key_R,CombineKey_Ctrl,KeyEvent_OnDown,Gimbal_Return_KeyCallback);//ctrl R 一键调头
}

// 更新陀螺仪角度并累积yaw角度
void Gimbal_UpdataAngle()
{
	gimbal.yaw.gyro = User_Imu_Get_Z_Gyro();
	gimbal.yaw.angle = User_Imu_GetYaw();
	gimbal.pitch.gyro = User_Imu_Get_Y_Gyro();
	gimbal.pitch.angle = User_Imu_GetPitch();
	float dAngle = 0;
	if (gimbal.yaw.angle - gimbal.yaw.lastAngle < -270)
		dAngle = gimbal.yaw.angle + (360 - gimbal.yaw.lastAngle);
	else if (gimbal.yaw.angle - gimbal.yaw.lastAngle > 270)
		dAngle = -gimbal.yaw.lastAngle - (360 - gimbal.yaw.angle);
	else
		dAngle = gimbal.yaw.angle - gimbal.yaw.lastAngle;
	gimbal.yaw.totalAngle += dAngle;

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

void Gimbal_VisionCtrl()
{
	int yaw_cycle;
	if ((gimbal.yaw.targetAngle / 360.f) > 0)
		yaw_cycle = (gimbal.yaw.targetAngle / 360.f) + 0.5f;
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
	if (GameRobotStat.power_management_gimbal_output == 0) // 7.17
	{
		PID_Clear(&gimbal.yaw.imuPID.inner);
		PID_Clear(&gimbal.pitch.imuPID.inner);
		DEPID_Clear(&gimbal.yaw.imuPID.deOuter);
		DEPID_Clear(&gimbal.pitch.imuPID.deOuter);
		gimbal.yaw.targetAngle = gimbal.yaw.angle;
		gimbal.yaw.imuPID.output = 0;
	}

	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
		gimbal.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		gimbal.rockerCtrl = false;
	//
	if (rcInfo.wheel > 600) // 遥控器模式下拨轮拨到底端开启自瞄
		gimbal.visionEnable = true;
	else if ((rcInfo.wheel < 600 && gimbal.rockerCtrl) || rcInfo.mouse.r != 1) // 遥控器模式下拨轮不在底端关闭自瞄
		gimbal.visionEnable = false;

	// 对识别状态进行滤波，防止偶尔的误识别
	visionFindAver = Filter_AverCalc(&gimbal.visionFilter.find, vision.found);


	if (gimbal.visionEnable && visionFindAver >= 0.5f && vision.yaw != 0) // 自瞄开启并识别成功
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
