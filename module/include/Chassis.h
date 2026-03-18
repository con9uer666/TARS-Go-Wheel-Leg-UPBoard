#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "moto.h"
#include "slope.h"
#include <stdbool.h>
#include "judge.h"
#include "graphics.h"

typedef enum
{
	ChassisMode_Follow,	  // 底盘跟随云台模式
	ChassisMode_Spin,	  // 小陀螺模式
	ChassisMode_Snipe_10, // 狙击模式10m，底盘与云台成45度夹角，移速大幅降低,鼠标DPS大幅降低
	ChassisMode_Snipe_20  // 20m
} ChassisMode;


typedef struct _Chassis
{
	// 底盘尺寸信息
	struct Info
	{
		float wheelbase;   // 轴距
		float wheeltrack;  // 轮距
		float wheelRadius; // 轮半径
		float offsetX;	   // 重心在xy轴上的偏移
		float offsetY;
	} info;
	// 4个电机
	DoubleMotor motors[4];
	float speedRto; // 底盘速度百分比 用于低速运动
	// 底盘移动信息
	struct Move
	{
		float vx; // 当前左右平移速度 mm/s
		float vy; // 当前前后移动速度 mm/s
		float vw; // 当前旋转速度 rad/s

		float maxVx, maxVy, maxVw; // 三个分量最大速度

		float Wheelangle[4];
		Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 斜坡
		PID buffer_pid;
		float out60_slope;
		float charge_slope;
		uint8_t cap_output;
		float maxPower;
		uint8_t last_power_management;
		uint8_t fastMode; // 快速模式  0-普通模式 1-快速模式

	} move;
	// 旋转相关信息
	struct
	{
		PID pid;				// 旋转PID，由relativeAngle计算底盘旋转速度
		float relativeAngle;	// 云台与底盘的偏离角 单位度
		int InitAngle;		// 云台与底盘对齐时的编码器值
		int InitpitchAngle; // 云台水平时编码器值
		int nowAngle;		// 此时云台的编码器值
		ChassisMode mode;		// 底盘模式
	} rotate;
	struct
	{
		int8_t key_w;
		int8_t key_a;
		int8_t key_s;
		int8_t key_d;
	} key;
	bool rockerCtrl;

} Chassis;

typedef struct Leg_Info
{
	float Current_L0;//腿当前长度 单位m
}Leg_Info_t;

typedef struct Foot_Chassis_Info
{
	float Yaw_Motor_Angle;//Yaw电机角度
	float Current_Speed;//底盘当前的速度 m/s
	Leg_Info_t L_Leg, R_Leg;//腿信息
}Foot_Chassis_Info_t;

typedef struct Foot_Chassis
{
	float Target_Vx, Target_Vy;//云台坐标系下的目标速度 单位m/s
	uint8_t Target_Leg_State;//目标腿长，0短腿 1长腿
	uint8_t Chassis_Mode;//0跟随 1小陀螺 2静止趴下
	Foot_Chassis_Info_t	Info;//底盘信息
}Foot_Chassis_t;

extern Chassis chassis;
extern Foot_Chassis_t Foot_Chassis;
extern char *Chassis_GetModeText(void);
extern uint8_t power_flag_offset;
void Chassis_Init(void);
void Chassis_UI_Init(void);
void Chassis_PowerCtrl(void);
void Chassis_TurnRecovercallback(void);
void Chassis_TurnLostcallback(void);

#endif
