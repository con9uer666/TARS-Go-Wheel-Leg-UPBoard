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

#define MOTOR_DM2325_DGR2CODE(dgr) ((int32_t)((dgr) * 819.1f * 2.5f)) // 36*8191/360
#define MOTOR_DM2325_CODE2DGR(code) ((float)((code) / 819.1f / 2.5f))

/********************* DM4310 (达妙) MIT 模式参数 *********************
 *  以下宏 *必须* 与上位机调试助手中写入电机的参数一致，否则角度/速度/扭矩
 *  解码会出错。这是用户需要根据实际配置自行调整的部分。
 ***********************************************************************/
// MIT 模式三个量程（与电机内 P_MAX / V_MAX / T_MAX 寄存器一致）
#define DM4310_PMAX (12.5f)   // 位置最大值，单位 rad
#define DM4310_VMAX (30.0f)   // 速度最大值，单位 rad/s
#define DM4310_TMAX (10.0f)   // 扭矩最大值，单位 N·m

// DM 电机的 CAN ID（手册里的 CAN_ID）—— 发送 MIT 帧时使用
#define DM_PITCH_MOTOR_CAN_ID  (0x01)
// DM 电机反馈帧的 Master ID（手册里的 MST_ID）—— 接收解析时匹配
#define DM_PITCH_MASTER_ID     (0x011)

#define MOTOR_M6020_DGR2CODE(dgr) ((int32_t)((dgr) * 22.7528f)) // 8191/360
#define MOTOR_M6020_CODE2DGR(code) ((float)((code) / 22.7528f))

#define MOTOR_M4005_DGR2CODE(dgr) ((int32_t)((dgr) * 182.04444f)) // 65536/360
#define MOTOR_M4005_CODE2DGR(code) ((float)((code) / 182.04444f))

#define MOTOR_MAX_CODE_TRUN (65535.f)

#define MOTOR_DM_RAD2DGR(rad) ((float)((rad)/(2*PI)) * 360.0f)

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


	float Position; //单位为deg
	float Last_Position; //单位为deg
	float Total_Position; //单位为deg
	float Target_Position; //单位为deg
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

// 达妙 DM4310（MIT模式）电机数据
typedef struct _DMMotor
{
	uint8_t  id;          // 反馈帧低 4 位
	uint8_t  err;         // 反馈帧高 4 位：0关使能 1正常 8~F故障
	// 原始量化值（来自 CAN 反馈，调试用）
	uint16_t pos_raw;     // 16bit
	uint16_t vel_raw;     // 12bit
	uint16_t tor_raw;     // 12bit
	// 解码后的物理量
	float    pos;         // rad
	float    vel;         // rad/s
	float    torque;      // N·m
	int8_t   t_mos;       // 驱动板温度 ℃
	int8_t   t_rotor;     // 线圈温度  ℃

	uint8_t  Is_Lost;     // 0 在线
	uint16_t LostCnt;
} DMMotor;

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
// 达妙 DM4310 MIT 反馈解析
void DMMotor_Update(DMMotor *motor, uint8_t *rxdata);
void record_state(motor_state *state, int8_t temperature, uint8_t error_state);
void data_from_above(int16_t vx, int16_t vy, int16_t vw, uint8_t stop_flag, uint8_t power_limit);
// 7010电机带底盘3508数据更新
void Turn_Update(DoubleMotor *motor, uint16_t TurnAngle, int16_t TurnSpeed, int16_t TurnTorque, int8_t TurnTemp);
void Drive_Update(DoubleMotor *motor, int16_t DriveAngle, int16_t DriveSpeed, int16_t DriveTorque, int8_t DriveTemp);

#endif
