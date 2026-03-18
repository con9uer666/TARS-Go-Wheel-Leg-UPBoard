/****************PID运算****************/

#include "PID.h"

// 初始化pid参数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->deadzone = 0;
}

// 初始化微分先行pid参数
void DEPID_Init(DEPID *pid, float p, float i, float d, float maxI, float maxOut, float gama)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->gama = gama;
}
// 单级微分先行pid计算
void PIDRegulation(DEPID *vPID, float reference, float feedback, float differentiation)
{
	// 更新数据
	vPID->lasterror = vPID->error;
	vPID->error = reference - feedback;
	// 微分滤波
	differentiation = vPID->gama * differentiation + (1 - vPID->gama) * vPID->lastPv;
	// 计算微分
	vPID->output = differentiation * vPID->kd;
	// 计算比例
	vPID->output += vPID->error * vPID->kp;
	// 计算积分
	vPID->integral += vPID->error * vPID->ki;
	LIMIT(vPID->integral, -vPID->maxIntegral, vPID->maxIntegral); // 积分限幅
	vPID->output += vPID->integral;
	// 输出限幅
	LIMIT(vPID->output, -vPID->maxOutput, vPID->maxOutput);
	// 更新微分滤波
	vPID->lastPv = differentiation;
}

/**
 * @brief 单级pid计算（浮点型）
 * 
 * @param pid 
 * @param reference 
 * @param feedback 
 */
void PID_SingleCalc(PID *pid, float reference, float feedback)
{
	// 更新数据
	pid->lastError = pid->error;
	if (ABS(reference - feedback) < pid->deadzone) // 若误差在死区内则error直接置0
		pid->error = 0;
	else
		pid->error = reference - feedback;
	// 计算微分
	pid->output = (pid->error - pid->lastError) * pid->kd;
	// 计算比例
	pid->output += pid->error * pid->kp;
	// 计算积分
	pid->integral += pid->error * pid->ki;
	LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral); // 积分限幅
	pid->output += pid->integral;
	// 输出限幅
	LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);
}

/**
 * @brief 单级PID计算(整型)
 * 
 * @param pid PID结构体指针
 * @param reference 目标参考值
 * @param feedback 当前反馈值
 */
void PID_SingleCalc_INT(PID *pid, int32_t reference, int32_t feedback)
{
	// 更新数据
	pid->lastError = pid->error;
	if (ABS(reference - feedback) < pid->deadzone) // 若误差在死区内则error直接置0
		pid->error = 0;
	else
		pid->error = (int32_t)((int32_t)reference - (int32_t)feedback);
	// 计算微分
	pid->output = (pid->error - pid->lastError) * pid->kd;
	// 计算比例
	pid->output += pid->error * pid->kp;
	// 计算积分
	pid->integral += pid->error * pid->ki;
	LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral); // 积分限幅
	pid->output += pid->integral;
	// 输出限幅
	LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);
}

// 串级pid计算
void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
	PID_SingleCalc(&(pid->outer), angleRef, angleFdb);			// 计算外环(角度环)
	PID_SingleCalc(&(pid->inner), pid->outer.output, speedFdb); // 计算内环(速度环)
	pid->output = pid->inner.output;
}

/**
 * @brief 串级PID初始化
 * 
 * @param pid PID结构体指针
 * @param angleRef 角度参考值
 * @param angleFdb 角度反馈值
 * @param speedFdb 内环速度反馈值
 * 
 * 反馈值是pid->output
 */
float PID_CascadeCalc_INT(CascadePID *pid, int32_t angleRef, int32_t angleFdb, int32_t speedFdb)
{
	PID_SingleCalc_INT(&(pid->outer), angleRef, angleFdb);			// 计算外环(角度环)
	PID_SingleCalc_INT(&(pid->inner), pid->outer.output, speedFdb); 	// 计算内环(速度环)
	pid->output = pid->inner.output;

	return pid->output;
}

// 串级微分先行pid计算		适用于云台 TODO拨弹等其他电机
void DEPID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
	PIDRegulation(&(pid->deOuter), angleRef, angleFdb, -speedFdb); // 计算外环微分先行(角度环)
	PID_SingleCalc(&(pid->inner), pid->deOuter.output, speedFdb);  // 计算内环(速度环)
	pid->output = pid->inner.output;
}

// 清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error = 0;
	pid->lastError = 0;
	pid->integral = 0;
	pid->output = 0;
}

void DEPID_Clear(DEPID *pid)
{
	pid->error = 0;
	pid->lasterror = 0;
	pid->integral = 0;
	pid->output = 0;
	pid->lastPv = 0;
}

// 重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid, float maxOut)
{
	pid->maxOutput = maxOut;
}

// 设置PID死区
void PID_SetDeadzone(PID *pid, float deadzone)
{
	pid->deadzone = deadzone;
}
