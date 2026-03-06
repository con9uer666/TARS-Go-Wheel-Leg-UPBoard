#include "stdint.h"
#include "Board2Board.h"
#include "chassis.h"
#include "gimbal.h"
#include "shooter.h"
#include "usart.h"
#include "dma.h"
#include "detect.h"
#include "super_cap.h"
#include "vision.h"
#include "RC.h"

#define EN_B2B_TASK // 使能任务
uint8_t usart2TxBuf[64];
uint8_t usart2RxBuf[128];
uint8_t STOPFLAG = 0;
uint8_t FEEDBACK = 0;
extern int8_t YawLost;
extern remote_control_t RemoteControl;
extern ext_referee_warning_t RefereeWarning;
extern DetectDevice detectList[DETECT_DEVICE_NUM];
extern SuperCap cap;
extern int8_t diagonal_enable ;
extern uint8_t UIupdateState;
extern uint8_t Rune_direction;
extern RC_TypeDef rcInfo;
extern bool Judge_Data_TF; // 裁判数据是否可用,辅助函数调用

extern DMA_HandleTypeDef hdma_usart2_rx;

extern float sendCurrent;

void RS485_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

int16_t temp555;

extern float Foot_Target_Relative_Angle;
extern float Foot_Target_Speed;

uint8_t UP_Leg;

void Rs485_Trans()
{
	usart2TxBuf[0] = 0xAA;
	usart2TxBuf[1] = ((int16_t)(Foot_Target_Relative_Angle * 1000));
	usart2TxBuf[2] = (((int16_t)(Foot_Target_Relative_Angle * 1000))) >> 8;

	usart2TxBuf[3] = ((int16_t)(Foot_Target_Speed * 1000));
	usart2TxBuf[4] = ((int16_t)(Foot_Target_Speed * 1000)) >> 8;

	temp555 = gimbal.yaw.imuPID.output * 1000;
	usart2TxBuf[25] = (((int16_t)temp555)) & 0xFF;
	usart2TxBuf[26] = (((int16_t)temp555) >> 8) & 0xFF;
//	usart2TxBuf[25] = 0;
//	usart2TxBuf[26] = 0;
	usart2TxBuf[27] = (int16_t)(shooter.triggerMotor.anglePID.output) & 0xFF;
	usart2TxBuf[28] = (int16_t)(shooter.triggerMotor.anglePID.output) >> 8 & 0xFF;
	usart2TxBuf[29] = STOPFLAG << 7 | chassis.rotate.mode << 5 | (visionFindAver > 0.8) << 4 | Vision_Mode << 2 | chassis.move.fastMode <<1 | UIupdateState;
	usart2TxBuf[30] = (int16_t)(ABS(shooter.fricMotor[0].speed) * 0.5f + ABS(shooter.fricMotor[1].speed) * 0.5f) & 0xff;
	usart2TxBuf[31] = (int16_t)(ABS(shooter.fricMotor[0].speed) * 0.5f + ABS(shooter.fricMotor[1].speed) * 0.5f) >> 8 & 0xff;
	usart2TxBuf[32] = (int16_t)chassis.rotate.relativeAngle & 0xff;
	usart2TxBuf[33] = (((int16_t)chassis.rotate.relativeAngle) >> 8) & 0xff;

	usart2TxBuf[34] = (int16_t)vision.distance & 0xff;
	//TODO!!	
//	usart2TxBuf[36] = (int32_t)(gimbal.yaw.imuPID.deOuter.output * 100) & 0xFF;
//	usart2TxBuf[37] = (int32_t)(gimbal.yaw.imuPID.deOuter.output * 100) >> 8 & 0xFF;
//	usart2TxBuf[38] = (int32_t)(gimbal.yaw.imuPID.deOuter.output * 100) >> 16 & 0xFF;
//	usart2TxBuf[39] = (int32_t)(gimbal.yaw.imuPID.deOuter.output * 100) >> 24 & 0xFF;
	usart2TxBuf[36] = shooter.number & 0xff ;
	usart2TxBuf[37] = shooter.number >> 8 & 0xff;
	usart2TxBuf[38] = shooter.number >> 16 & 0xff;
	usart2TxBuf[39] = shooter.workState == TRIGGER_REVERSE;
	
	usart2TxBuf[40] = (uint8_t)cap.per_energy;
	usart2TxBuf[41] = diagonal_enable << 7 | vision.exposure_time << 2 | Rune_direction << 1 | shooter.block.state ;	
	// usart2TxBuf[42] = rcInfo.left;

	if(rcInfo.left == 1)
		UP_Leg = 1;
	else
		UP_Leg = 0;
	// else if(rcInfo.left == 0)
	// 	UP_Leg = 2;
	usart2TxBuf[42] = UP_Leg;
	usart2TxBuf[63] = 0xFE;
	HAL_UART_Transmit_DMA(&huart2, usart2TxBuf, sizeof(usart2TxBuf));
}

int16_t remainHeat;
float initialSpeed;
uint16_t coolingValue;
float temp1,temp2;

void RS485_Rec()
{
	if (usart2RxBuf[0] == 0xAB && usart2RxBuf[63] == 0xFD)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			chassis.motors[i].TurnAngle = 0;
			for (uint8_t j = 0; j < 4; j++)
			{
				chassis.motors[i].TurnAngle |= (int16_t)usart2RxBuf[i * 4 + j + 1] << (j * 8);
			}
		}
		for (uint8_t i = 0; i < 4; i++)
		{
			chassis.motors[i].DriveSpeed = (int16_t)usart2RxBuf[17 + i * 2] | (int16_t)usart2RxBuf[17 + i * 2 + 1] << 8;
		}
		gimbal.yawMotor_M4005.angle = (int16_t)usart2RxBuf[25] | (int16_t)usart2RxBuf[26] << 8;
		gimbal.yawMotor_M4005.speed = (int16_t)usart2RxBuf[27] | (int16_t)usart2RxBuf[28] << 8;

		temp1 = (int16_t)(usart2RxBuf[29] | usart2RxBuf[30] << 8);

		shooter.triggerMotor.Position = 180.0f + ((temp1 / 1000.0f)/PI) * 180.0f;//0-360°

		temp2 = (int16_t)(usart2RxBuf[31] | usart2RxBuf[32] << 8);
		shooter.triggerMotor.speed = ((temp2/100.0f)/(2*PI)) * 60.0f * 25;

		FEEDBACK = usart2RxBuf[33] >> 7 & 0x01;
		YawLost = usart2RxBuf[33] >> 6 & 0x01;

		uint8_t statusByte = usart2RxBuf[33];
		uint8_t feedback = (statusByte >> 7) & 0x01;
		int8_t yawlost = (statusByte >> 6) & 0x01;
		uint8_t gimbalOutput = (statusByte >> 5) & 0x01;
		uint8_t chassisOutput = (statusByte >> 4) & 0x01;
		uint8_t shooterOutput = (statusByte >> 3) & 0x01;
		uint8_t level = (statusByte) & 0x07;
		// 电机离线检测
		detectList[DeviceID_ChassisMotor1].lastLostState = detectList[DeviceID_ChassisMotor1].isLost;
		detectList[DeviceID_ChassisMotor2].lastLostState = detectList[DeviceID_ChassisMotor2].isLost;
		detectList[DeviceID_ChassisMotor3].lastLostState = detectList[DeviceID_ChassisMotor3].isLost;
		detectList[DeviceID_ChassisMotor4].lastLostState = detectList[DeviceID_ChassisMotor4].isLost;
		detectList[DeviceID_Turn_Motor1].lastLostState = detectList[DeviceID_Turn_Motor1].isLost;
		detectList[DeviceID_Turn_Motor2].lastLostState = detectList[DeviceID_Turn_Motor2].isLost;
		detectList[DeviceID_Turn_Motor3].lastLostState = detectList[DeviceID_Turn_Motor3].isLost;
		detectList[DeviceID_Turn_Motor4].lastLostState = detectList[DeviceID_Turn_Motor4].isLost;

		detectList[DeviceID_ChassisMotor1].isLost = (usart2RxBuf[34] >> 7) & 0x01;
		detectList[DeviceID_ChassisMotor2].isLost = (usart2RxBuf[34] >> 6) & 0x01;
		detectList[DeviceID_ChassisMotor3].isLost = (usart2RxBuf[34] >> 5) & 0x01;
		detectList[DeviceID_ChassisMotor4].isLost = (usart2RxBuf[34] >> 4) & 0x01;
		detectList[DeviceID_Turn_Motor1].isLost = (usart2RxBuf[34] >> 3) & 0x01;
		detectList[DeviceID_Turn_Motor2].isLost = (usart2RxBuf[34] >> 2) & 0x01;
		detectList[DeviceID_Turn_Motor3].isLost = (usart2RxBuf[34] >> 1) & 0x01;
		detectList[DeviceID_Turn_Motor4].isLost = (usart2RxBuf[34] >> 0) & 0x01;
		// 更新相应状态变量
		FEEDBACK = feedback;
		YawLost = yawlost;
		detectList[DeviceID_YawMotor].isLost = yawlost;
		GameRobotStat.power_management_gimbal_output = gimbalOutput;
		GameRobotStat.power_management_chassis_output = chassisOutput;
		GameRobotStat.power_management_shooter_output = shooterOutput;
		GameRobotStat.robot_level = level;

		// 解析热量数据
		remainHeat = usart2RxBuf[35] | (usart2RxBuf[36] << 8);

		// 解析射击初速度

		uint16_t speedRaw = usart2RxBuf[37] | (usart2RxBuf[38] << 8);
		ShootData.initial_speed = speedRaw / 1000.0f;

		// 解析冷却值
		coolingValue = usart2RxBuf[39] | (usart2RxBuf[40] << 8);
		GameRobotStat.shooter_barrel_cooling_value = coolingValue;
		PowerHeatData.chassis_power_buffer = usart2RxBuf[41] | (usart2RxBuf[42] << 8);
		GameRobotStat.chassis_power_limit = usart2RxBuf[43] | (usart2RxBuf[44] << 8);
		Judge_Data_TF = usart2RxBuf[45];

		cap.receive_data.cap_voltage = usart2RxBuf[47] | (usart2RxBuf[48] << 8);


		cap.receive_data.bus_power = usart2RxBuf[49] | (usart2RxBuf[50] << 8);

		cap.receive_data.L_current = usart2RxBuf[51] | (usart2RxBuf[52] << 8);
		cap.receive_data.L_current = cap.receive_data.L_current / 100;

		cap.receive_data.power_ctrl_mode = (usart2RxBuf[53] >> 2) & 0x03;
		cap.receive_data.automode_stage = usart2RxBuf[53] & 0x03;
		cap.receive_data.max_power = cap.receive_data.cap_voltage * 9;
		
		
	}
}
uint16_t frequent = 0;
#ifdef EN_B2B_TASK // 使能任务
void OS_Board2BoardCallback(void const *argument)
{
	while (1)
	{
		frequent++;
		Rs485_Trans();
		osDelay(1);
	}
}
#endif
