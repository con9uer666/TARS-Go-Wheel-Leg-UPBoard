// Moto.c 文件注释
// 本文件主要实现电机相关的功能，包括电机角度计算、数据更新、状态记录等。

#include "Moto.h"
#include "chassis.h"
#include "super_cap.h"
#include "Gimbal.h"
#include "shooter.h"
#include "USER_CAN.h"
#include "UserFreertos.h"
#include "vision.h"
#include "n20.h"
#include "smc.h"
#include "Detect.h"
#include "bsp_delay.h"
#include "RS485.h"
#include "Com.h"
#include "Rc.h"
#include "Judge.h"
#include "beep.h"
#include <math.h>

#define EN_MOTOR_TASK // 使能任务

// 全局变量定义
char last_power_flag = 0; // 上一次功率标志
char power_flag = 0;      // 0：电管不给电，1：电管给电
char delay_flag = 0;      // 延迟标志，1：正在延迟
int time_count = 0;       // 时间计数器

motor_state turn_motor_state[4]; // 电机状态数组

// 电机套圈解决方案：电机累计角度vscode://lirentech.file-ref-tags?filePath=Moto.c&snippet=%2F%2F+%E7%94%B5%E6%9C%BA%E5%A5%97%E5%9C%88%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%88%EF%BC%9A%E7%94%B5%E6%9C%BA%E7%B4%AF%E8%AE%A1%E8%A7%92%E5%BA%A6
	// 开始计算电机累计角度
	void Motor_StartCalcAngle(SingleMotor *motor)
	{
		motor->totalAngle = 0;
		motor->lastAngle = motor->angle;
		motor->targetAngle = 0;
	}

	// 计算电机累计转过的圈数
	void Motor_CalcAngle(SingleMotor *motor)
	{
		float dAngle = 0;
		if (motor->Position - motor->Last_Position < -180.0f)
			dAngle = motor->Position + (360.0f - motor->Last_Position);
		else if (motor->Position - motor->Last_Position > 180.0f)
			dAngle = -motor->Last_Position - (360.0f - motor->Position);
		else
			dAngle = motor->Position - motor->Last_Position;
		// 将角度增量加入计数器
		motor->Total_Position += dAngle;
		// 记录角度
		motor->Last_Position = motor->Position;
	}

// 更新电机数据(可能进行滤波)
void Motor_Update(SingleMotor *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp)
{
	motor->angle = angle;
	motor->speed = speed;
	motor->torque = torque;
	motor->temp = temp;
}

// 达妙 DM4310 反馈解析（MIT 模式应答）
// 帧格式：D[0]=高4位ERR|低4位ID, D[1..2]=POS(16b),
//        D[3..4]=VEL(12b)<<4 | TOR(12b 高4),
//        D[5]=TOR(12b 低8), D[6]=T_MOS, D[7]=T_Rotor
static float dm_uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / (float)((1 << bits) - 1) + x_min;
}

void DMMotor_Update(DMMotor *motor, uint8_t *rxdata)
{
    motor->id  = rxdata[0] & 0x0F;
    motor->err = (rxdata[0] >> 4) & 0x0F;

    motor->pos_raw = ((uint16_t)rxdata[1] << 8) | rxdata[2];
    motor->vel_raw = ((uint16_t)rxdata[3] << 4) | ((rxdata[4] >> 4) & 0x0F);
    motor->tor_raw = ((uint16_t)(rxdata[4] & 0x0F) << 8) | rxdata[5];

    motor->pos    = dm_uint_to_float(motor->pos_raw, -DM4310_PMAX, DM4310_PMAX, 16);
    motor->vel    = dm_uint_to_float(motor->vel_raw, -DM4310_VMAX, DM4310_VMAX, 12);
    motor->torque = dm_uint_to_float(motor->tor_raw, -DM4310_TMAX, DM4310_TMAX, 12);

    motor->t_mos   = (int8_t)rxdata[6];
    motor->t_rotor = (int8_t)rxdata[7];
}

// 更新 LK 电机数据
void LKMotor_Update(LKMotor *motor, uint8_t *rxdata)
{
    if (rxdata[0] == 0x9c || rxdata[0] == 0xa4 || rxdata[0] == 0xa2 || rxdata[0] == 0xa8 || rxdata[0] == 0xa1 || rxdata[0] == 0xa6)
    {
        motor->angle = rxdata[7] << 8 | rxdata[6];
        motor->speed = rxdata[5] << 8 | rxdata[4];
        motor->torque = rxdata[3] << 8 | rxdata[2];
        motor->temp = rxdata[1];
    }
    if (rxdata[0] == 0x9a || rxdata[0] == 0x9b)
    {
        motor->temp = rxdata[1];
        motor->voltage = rxdata[3] << 8 | rxdata[2];
        motor->current = rxdata[5] << 8 | rxdata[4];
        motor->motorstage = rxdata[6];
        motor->errorCode = rxdata[7];
    }
}
//不知什么电机的数据更新
void Drive_Update(DoubleMotor *motor, int16_t DriveAngle, int16_t DriveSpeed, int16_t DriveTorque, int8_t DriveTemp)
{
	motor->DriveAngle = DriveAngle;
	motor->DriveSpeed = DriveSpeed;
	motor->DriveTorque = DriveTorque;
	motor->DriveTemp = DriveTemp;
}

// 7010电机数据更新
void Turn_Update(DoubleMotor *motor, uint16_t TurnAngle, int16_t TurnSpeed, int16_t TurnTemp, int8_t TurnTorque)
{
	motor->TurnAngle = TurnAngle;
	motor->TurnSpeed = TurnSpeed;
	motor->TurnTemp = TurnTemp;
	motor->TurnTorque = TurnTorque;
}
//更新底盘数据
void data_from_above(int16_t vx, int16_t vy, int16_t vw, uint8_t stop_flag, uint8_t power_limit)
{
	chassis.move.vx = vx;
	chassis.move.vy = vy;
	chassis.move.vw = vw / 10.0f;
	stop_flag_t = stop_flag;
	// extern ext_game_robot_status_t GameRobotStat;
	// GameRobotStat.chassis_power_limit = power_limit;
	// power_flag_offset=power_flag_t;
}

// 记录电机状态
void record_state(motor_state *state, int8_t temperature, uint8_t error_state)
{
	state->temperature = temperature;
	state->error_state = error_state & 8;
}

// 初始化延迟逻辑vscode://lirentech.file-ref-tags?filePath=Moto.c&snippet=%2F%2F+%E5%88%9D%E5%A7%8B%E5%8C%96%E5%BB%B6%E8%BF%9F%E9%80%BB%E8%BE%91
void Init_delay()
{
	if (GameRobotStat.power_management_gimbal_output == 0)//电管不给底盘供电
	{
		power_flag = 0;
	}
	else if (GameRobotStat.power_management_gimbal_output == 1)
	{
		power_flag = 1;
	}

	if (power_flag == 1 && last_power_flag == 0)//给电的第一刻
	{
		delay_flag = 1;
	}

	if (delay_flag == 1)//正在延迟
	{
		time_count++;

		if (time_count == 1500)
		{
			time_count = 0;
			delay_flag = 0;	
		}
	}

	last_power_flag = power_flag;
}

extern uint8_t chaasisReady;
/************************freertos任务****************************/


// 所有CAN电机的控制任务
void Task_CANMotors_Callback()
{
	
	Motor_CalcAngle(&shooter.triggerMotor);
	//pitch电机双环
	PID_CascadeCalc_INT(&shooter.triggerMotor.anglePID, shooter.triggerMotor.Target_Position, shooter.triggerMotor.Total_Position, shooter.triggerMotor.speed);

    //摩擦轮pid
	PID_SingleCalc(&shooter.fricMotor[0].speedPID, shooter.fricMotor[0].targetSpeed, shooter.fricMotor[0].speed);
	PID_SingleCalc(&shooter.fricMotor[1].speedPID, shooter.fricMotor[1].targetSpeed, shooter.fricMotor[1].speed);

	//	Init_delay();
	// Chassis_PowerCtrl();

	// DM4310 故障/过热保护：err 非 1（正常使能）或线圈温度过高时清错并重新使能
	if (gimbal.pitchMotor.DM4310.err != 1 || gimbal.pitchMotor.DM4310.t_rotor > 90)
	{
		PID_Clear(&gimbal.pitch.imuPID.inner);
		DEPID_Clear(&gimbal.pitch.imuPID.deOuter);
		gimbal.pitch.imuPID.output = 0;
		DM_MotorClearError(&hfdcan2, DM_PITCH_MOTOR_CAN_ID);
		DM_MotorEnable(&hfdcan2, DM_PITCH_MOTOR_CAN_ID);
	}
	else
	{
		// 在 STM32 端跑双环 PID（位置外环 + 速度内环），DM 用纯力矩前馈：
		// p_des=0, v_des=0, kp=0, kd=0, t_ff = PID 输出
		DM_MIT_Ctrl(&hfdcan2, DM_PITCH_MOTOR_CAN_ID,
		            0.0f, 0.0f, 0.0f, 0.0f, gimbal.pitch.imuPID.output);
	}

	USER_CAN_SetMotorCurrent(&hfdcan3, 0x200, shooter.fricMotor[1].speedPID.output, shooter.fricMotor[0].speedPID.output, 0, 0);

}

#ifdef EN_MOTOR_TASK
// FreeRTOS 任务入口
void OS_MotorCallback(void const *argument)
{
	osDelay(300);
	// DM 上电使能序列：清错 → 使能。失败时由 Task_CANMotors_Callback 内的保护逻辑兜底重试
	DM_MotorClearError(&hfdcan2, DM_PITCH_MOTOR_CAN_ID);
	osDelay(2);
	DM_MotorEnable(&hfdcan2, DM_PITCH_MOTOR_CAN_ID);
	osDelay(2);
	for (;;)
	{
		Task_CANMotors_Callback();
		osDelay(1);
	}
}
#endif
