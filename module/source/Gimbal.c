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
// 初始化云台函数
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
	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 0.7, 0.001, 0.02, 20, 20, 0.5);  //20 0 2.5 0.4 1000

//	一套软的
//	PID_Init(&gimbal.yaw.imuPID.inner, 4.5, 0.03, 1.5, 1000, 2048);  
//	DEPID_Init(&gimbal.yaw.imuPID.deOuter, 14, 0.05, 0.1, 400, 1000, 0.4);  //20 0 2.5 0.4 1000	

//		PID_Init(&gimbal.yaw.imuPID.inner, 10, 0.02, 2, 1000, 2048); 
//		DEPID_Init(&gimbal.yaw.imuPID.deOuter, 27, 0.01, 0, 100, 500, 1);  //20 0 2.5 0.4 1000	
}

// 头会侧翻（chassis roll≠0 时 IMU pitch 不等于真实头部俯仰），
// 所以 PID 用 IMU 闭环没问题，但限幅必须用电机自身编码器 —— 编码器和
// 头是机械绑定的，不受底盘姿态影响。
//
// 标定约定：
//   * DM 电机零点设在头部最低位置（低头 20°）
//   * 电机 “正转”（pos 增大）方向 = 头部低头方向
//   * 因此运行时电机 pos ∈ [pos_min, 0]，抬头让 pos 变负
//   * 头部摆幅 = 低头20° + 仰头30° = 50° ≈ 0.873 rad（直接驱动，1:1）
//
// 限幅策略：到了机械极限就把 IMU 目标角度拉回到当前 IMU 角度，让 PID
// 误差归零、不再继续往该方向推电机。两侧只各管自己的方向，反向请求
// 不受影响。
void PitchLimit()
{
    static const float pos_low_limit  =  0.0f;    // 低头到底（电机零点）
    static const float pos_high_limit = -0.873f;  // 仰头到顶（按实测调整）

    float p = gimbal.pitchMotor.DM4310.pos;

    // 撞下限（低头到底）：只禁止目标继续往低头（IMU pitch 减小）方向走
    if (p >= pos_low_limit && gimbal.pitch.targetAngle < gimbal.pitch.angle)
        gimbal.pitch.targetAngle = gimbal.pitch.angle;
    // 撞上限（仰头到顶）：只禁止目标继续往仰头（IMU pitch 增大）方向走
    if (p <= pos_high_limit && gimbal.pitch.targetAngle > gimbal.pitch.angle)
        gimbal.pitch.targetAngle = gimbal.pitch.angle;

    errorP = gimbal.pitch.targetAngle - gimbal.pitch.angle;
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

	// 套圈处理：把 targetAngle 折叠到 totalAngle 的 ±180° 内（最短路径）。
	// 原来用 ±360° 阈值，误差可达 359°——头被被动转近一圈后，发给下板的
	// PID 输出会让底盘跟着转一整圈，而不是反方向走最短角度。收紧到 ±180°
	// 后，PID 误差始终是最短角度，下板拿到的跟随量自然也是最短路径。
	while (gimbal.yaw.targetAngle - gimbal.yaw.totalAngle >  180.0f)
		gimbal.yaw.targetAngle -= 360.0f;
	while (gimbal.yaw.totalAngle - gimbal.yaw.targetAngle >= 180.0f)
		gimbal.yaw.targetAngle += 360.0f;

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
	// 电管不给电后云台电机把各项变量清零，防止电管重启后云台乱转
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

	// 遥控器拨轮切换摇杆控制还是鼠标控制，以及开启自瞄
	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
		gimbal.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		gimbal.rockerCtrl = false;
	if (rcInfo.wheel > 600) // 遥控器模式下拨轮拨到底端开启自瞄
		gimbal.visionEnable = true;
	else if ((rcInfo.wheel < 600 && gimbal.rockerCtrl) || rcInfo.mouse.r != 1) // 遥控器模式下拨轮不在底端关闭自瞄
		gimbal.visionEnable = false;

	// 对识别状态进行滤波，防止偶尔的误识别
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
	// 更新陀螺仪角度
	Gimbal_UpdataAngle();
	PitchLimit();
	if (GameRobotStat.power_management_gimbal_output == 0 ) // 7.17
	{
		gimbal.yaw.targetAngle=gimbal.yaw.totalAngle;
		gimbal.yaw.gyro=0;
	}
	//虽然算了角度速度环，但是为了使用达妙mit速度环，所以yaw电机的输出还是用的外环微分先行的输出
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
