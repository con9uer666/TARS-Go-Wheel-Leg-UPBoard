#include "Judge.h"
#include "string.h"
#include "crc.h"
#include "Detect.h"
#include "Graphics.h"
#include "userfreertos.h"
#include "usart.h"
#include "vision.h"
#include "gimbal.h"
#include "chassis.h"
#include "shooter.h"
#include "rc.h"
#include "myQueue.h"
#define EN_JUDGE_TASK

extern int16_t remainHeat;
extern float initialSpeed;
extern uint16_t coolingValue;
bool Judge_Data_TF = FALSE; // 裁判数据是否可用,辅助函数调用

void JUDGE_GraphTest_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

/*****************系统数据定义**********************/
ext_game_status_t GameState;						   // 0x0001
ext_game_result_t GameResult;						   // 0x0002
ext_game_robot_HP_t GameRobotHP;					   // 0x0003
ext_event_data_t EventData;							   // 0x0101
ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
ext_referee_warning_t RefereeWarning;				   // 0x0104
ext_dart_info_t DartRemainingTime;					   // 0x0105
ext_game_robot_status_t GameRobotStat;				   // 0x0201
ext_power_heat_data_t PowerHeatData;				   // 0x0202
ext_game_robot_pos_t GameRobotPos;					   // 0x0203
ext_buff_musk_t BuffMusk;							   // 0x0204
aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
ext_robot_hurt_t RobotHurt;							   // 0x0206
ext_shoot_data_t ShootData;							   // 0x0207
ext_bullet_remaining_t BulletRemaining;				   // 0x0208
ext_rfid_status_t RfidStatus;						   // 0x0209
ext_dart_client_cmd_t DartClientCmd;				   // 0x020A

remote_control_t RemoteControl; // 0x0304

xFrameHeader FrameHeader; // 发送帧头信息
/****************************************************/

// 串口接收缓冲区
uint8_t usart6RxBuf[JUDGE_MAX_RX_LENGTH];

uint16_t shootNum = 0; // 统计发弹量

// 获取己方颜色
RobotColor JUDGE_GetSelfColor()
{
	if (JUDGE_GetSelfID() > 10) // 蓝方
	{
		return RobotColor_Blue;
	}
	else // 红方
	{
		return RobotColor_Red;
	}
}

// 获取自身ID
uint8_t JUDGE_GetSelfID()
{
	return GameRobotStat.robot_id;
}

// 获取客户端ID
uint16_t JUDGE_GetClientID()
{
	return 0x100 + GameRobotStat.robot_id;
}

// 获取机器人坐标
void JUDGE_GetPosition(float *x, float *y)
{
	*x = GameRobotPos.x;
	*y = GameRobotPos.y;
}

// 获取底盘功率限制
uint8_t JUDGE_GetChassisPowerLimit()
{
	return GameRobotStat.chassis_power_limit;
}

// 判断发射电源是否输出
bool JUDGE_GetShooterOutputState()
{
	return GameRobotStat.power_management_shooter_output;
}

bool JUDGE_GetGimbalOutputState()
{
	return GameRobotStat.power_management_gimbal_output;
}

// 获取枪口热量限制
uint16_t JUDGE_GetHeatLimit()
{
	return GameRobotStat.shooter_barrel_heat_limit;
}

// 获取射速限制
uint16_t JUDGE_GetShootSpeedLimit()
{
	return 25;
}

// 获取底盘缓冲能量
uint16_t JUDGE_GetPowerBuffer()
{
	return PowerHeatData.chassis_power_buffer;
}
// 获取剩余枪口热量
int16_t JUDGE_GetRemainHeat()
{
	return remainHeat;
}

// 剩余17发弹数
uint16_t JUDGE_GetRemain_42_Num()
{
	return BulletRemaining.projectile_allowance_42mm;
}

// 读取当前血量
uint16_t JUDGE_GetHP()
{
	return GameRobotStat.current_HP;
}

// 获取冷却速度
uint16_t JUDGE_GetCoolingValue()
{
	GameRobotStat.shooter_barrel_cooling_value = coolingValue;
	return GameRobotStat.shooter_barrel_cooling_value;
}
bool JUDGE_IsValid(void)
{
	return Judge_Data_TF;
}


/**********************freertos任务*********************************/

#ifdef EN_JUDGE_TASK
void OS_JudgeCallback(void const *argument)
{
	osDelay(1000); // 任务偏移时间


	osDelay(500);
	for (;;)
	{
//		Task_Judge_Callback();
		osDelay(200);
	}
}
#endif
