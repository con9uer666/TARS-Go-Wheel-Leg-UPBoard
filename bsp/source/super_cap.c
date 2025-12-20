#include "super_cap.h"
#include "UserFreertos.h"
#include "graphics.h"
#include "chassis.h"
#include "judge.h"
#include <stdio.h>
#include "Detect.h"

SuperCap cap;
extern ext_game_robot_status_t GameRobotStat;
void Cap_AnalysisData()
{
	if (GameRobotStat.power_management_chassis_output == 0)
	{
		cap.power_ctrl_mode = TURNOFF;
		cap.target_output_power = 0;
		cap.target_charge_power = 0;
		chassis.move.outputSlope.value = 0;
		chassis.move.chargeSlope.value = 0;
		cap.last_power_management = 0;
	}
	if (GameRobotStat.power_management_chassis_output == 1 && cap.last_power_management == 0)
	{
		uint8_t lost = 0;
		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor1);
		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor2);
		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor3);
		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor4);
		if (lost != 0)
		{
			cap.power_ctrl_mode = TURNOFF;
			cap.target_output_power = 0;
			cap.target_charge_power = 0;
			chassis.move.outputSlope.value = 0;
			chassis.move.chargeSlope.value = 0;
			Cap_CanSendData();
		}
		if (lost == 0)
			cap.last_power_management = 1;
	}
	cap.cap_vot = cap.receive_data.cap_voltage / 100.f;
	cap.energy = (cap.cap_vot * cap.cap_vot - 5.0f * 5.0f) * 0.5f * 11.0f;
	cap.per_energy = cap.energy / 1744.875f * 100.0f; // 相对5v的百分比能量值 1744.875=0.5*11*（18.5*18.5-5*5）
}
void OS_SuperCapCallback(void const *argument)
{
	osDelay(1500);
	uint32_t cnt = 0;
	for (;;)
	{
		Cap_AnalysisData();
		Cap_CanSendData();
		if (cnt % 200 == 0)
		{
//			char textBuf[30] = {0};
//			sprintf(textBuf, "%03d %%", (int)cap.per_energy);
			//			Graph_SetText(&chassis.ui.super_cap,"CAP",Color_Orange,4,0,1000,780,textBuf,5,30);
			//			Graph_DrawText(&chassis.ui.super_cap,Operation_Change);
		}
//		cnt++;
		osDelay(1);
	}
}
