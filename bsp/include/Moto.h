#ifndef _MOTO_H_
#define _MOTO_H_
#include "stdint.h"
#include "PID.h"
#include "smc.h"

// 各种电机编码值与角度的换算
#define MOTOR_M3508_DGR2CODE(dgr) ((int32_t)((dgr) * 436.9263f)) // 3591/187*8191/360
#define MOTOR_M3508_CODE2DGR(code) ((float)((code) / 436.9263f))

#define MOTOR_M2006_DGR2CODE(dgr) ((int32_t)((dgr) * 819.1f * 2.5f)) // 36*8191/360
#define MOTOR_M2006_CODE2DGR(code) ((float)((code) / 819.1f / 2.5f))

#define MOTOR_M6020_DGR2CODE(dgr) ((int32_t)((dgr) * 22.7528f)) // 8191/360
#define MOTOR_M6020_CODE2DGR(code) ((float)((code) / 22.7528f))

#define MOTOR_M4005_DGR2CODE(dgr) ((int32_t)((dgr) * 182.04444f)) // 65536/360
#define MOTOR_M4005_CODE2DGR(code) ((float)((code) / 182.04444f))

#define MOTOR_MAX_CODE_TRUN (65535.f)

typedef struct _MOTOR
{
	int16_t angle, speed, torque;
	int8_t temp;

	int16_t lastAngle; // 记录上一次得到的角度

	int16_t targetSpeed; // 目标速度
	int32_t targetAngle; // 目标角度(编码器值)

	int32_t totalAngle; // 累计转过的编码器值

	PID speedPID;		 // 速度pid(单级)
	CascadePID anglePID; // 角度pid，串级
	
	SMC FricSMC;

	uint8_t Is_Lost; // 0为在线
	uint16_t LostCnt;
} SingleMotor;

typedef struct
{
	int16_t angle, speed,voltage,current;
	int8_t torque,errorCode, temp,motorstage;

	int16_t lastAngle; // 记录上一次得到的角度

	int32_t targetSpeed; // 目标速度
	int32_t targetAngle; // 目标角度(编码器值)
	int16_t targetCurrent;
	uint16_t speed_limit;
	
} LKMotor;

typedef struct _M7010
{
	uint16_t DriveAngle;
	int16_t DriveSpeed;
	int16_t DriveTorque;
	int8_t DriveTemp;

	uint16_t TurnAngle;
	int16_t TurnSpeed;
	int16_t TurnTorque;
	int8_t TurnTemp;
	float TurnOffset; // 轮子对正前方的编码器值

	float now_angle;
	uint16_t lastAngle; // 记录上一次得到的角度

	int16_t targetDriveSpeed; // 目标m3508速度
	int32_t targetTurnSpeed;  // 目标4005速度
	uint16_t turn_speed_limit;
	PID TurnSpeed_LIMITpid;

	float multi_targetTurnAngle;
	float last_targetTurnAngle;
	float targetTurnAngle; // 目标角度(编码器值)

	int32_t totalAngle; // 累计转过的编码器值

	PID speedPID;		 // 速度pid(单级)
	CascadePID anglePID; // 角度pid，串级
	SMC FricSMC;

	uint8_t turnMotorDirection;

	uint8_t Is_Lost; // 0为在线
	uint16_t LostCnt;
} DoubleMotor;
typedef struct state
{
	int8_t temperature;
	uint8_t error_state;
} motor_state;
extern motor_state turn_motor_state[4];
// 开始计算电机累计角度
void Motor_StartCalcAngle(SingleMotor *motor);
// 计算电机累计转过的圈数
void Motor_CalcAngle(SingleMotor *motor);
// 更新电机数据(可能进行滤波)
void Motor_Update(SingleMotor *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp);
void LKMotor_Update(LKMotor *motor, uint8_t *rxdata);
void record_state(motor_state *state, int8_t temperature, uint8_t error_state);
void data_from_above(int16_t vx, int16_t vy, int16_t vw, uint8_t stop_flag, uint8_t power_limit);
// 7010电机带底盘3508数据更新
void Turn_Update(DoubleMotor *motor, uint16_t TurnAngle, int16_t TurnSpeed, int16_t TurnTorque, int8_t TurnTemp);
void Drive_Update(DoubleMotor *motor, int16_t DriveAngle, int16_t DriveSpeed, int16_t DriveTorque, int8_t DriveTemp);

#endif
