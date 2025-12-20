#include "Detect.h"
#include "USER_CAN.h"
#include "Beep.h"
#include "Vision.h"
#include "Gimbal.h"
#include "judge.h"
#include "Com.h"
#include "Chassis.h"
#include "Shooter.h"
#include "RC.h"
void Motor_StartCalcAngle_M4005(DoubleMotor *motor);
/****内部函数声明****/
// 默认掉线处理函数
void Detect_DefaultLostHandler(uint8_t deviceID);
// 默认上线处理函数
void Detect_DefaultRecoverHandler(uint8_t deviceID);
// 初始化单个设备
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void));

// 设备离线信息列表
DetectDevice detectList[DETECT_DEVICE_NUM];

// 更新指定id设备的状态
void Detect_Update(uint8_t deviceID)
{
	detectList[deviceID].lastRecieveTime = HAL_GetTick();
}

// 默认掉线处理函数，若lostFunc==NULL则调用
void Detect_DefaultLostHandler(uint8_t deviceID)
{
	if (deviceID == DeviceID_Turn_Motor1)
	{
		chassis.motors[0].multi_targetTurnAngle = 0;
	}
	else if (deviceID == DeviceID_Turn_Motor2)
	{
		chassis.motors[1].multi_targetTurnAngle = 0;
	}
	else if (deviceID == DeviceID_Turn_Motor3)
	{
		chassis.motors[2].multi_targetTurnAngle = 0;
	}
	else if (deviceID == DeviceID_Turn_Motor4)
	{
		chassis.motors[3].multi_targetTurnAngle = 0;
	}
}

// 默认掉线处理函数，若recoverFunc==NULL则调用
void Detect_DefaultRecoverHandler(uint8_t deviceID)
{
	if (deviceID == DeviceID_Turn_Motor1)
	{
		Motor_StartCalcAngle_M4005(&chassis.motors[0]);
	}
	else if (deviceID == DeviceID_Turn_Motor2)
	{
		Motor_StartCalcAngle_M4005(&chassis.motors[1]);
	}
	else if (deviceID == DeviceID_Turn_Motor3)
	{
		Motor_StartCalcAngle_M4005(&chassis.motors[2]);
	}
	else if (deviceID == DeviceID_Turn_Motor4)
	{
		Motor_StartCalcAngle_M4005(&chassis.motors[3]);
	}
}

// 初始化一个设备的掉线检测信息(设备id，数据接收最大间隔，掉线处理回调函数)
void Detect_InitDevice(uint8_t deviceID, uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void))
{
	detectList[deviceID].maxInterval = maxInterval;
	detectList[deviceID].isLost = 0;
	detectList[deviceID].lostFunc = lostFunc;
	detectList[deviceID].recoverFunc = recoverFunc;
}

// 初始化所有设备的掉线检测信息
void Detect_InitAll()
{
	Detect_InitDevice(DeviceID_ChassisMotor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor3, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_ChassisMotor4, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_Turn_Motor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_Turn_Motor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_Turn_Motor3, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_Turn_Motor4, 50, NULL, NULL);
	
	Detect_InitDevice(DeviceID_YawMotor, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_PitchMotor, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_FricMotor1, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_FricMotor2, 50, NULL, NULL);
	Detect_InitDevice(DeviceID_TrigMotor, 50, NULL, NULL);

	Detect_InitDevice(DeviceID_RC, 100, RC_LostCallback, NULL);

	Detect_InitDevice(DeviceID_Judge, 500, NULL, NULL);

	Detect_InitDevice(DeviceID_IMU, 500, NULL, NULL);

	Detect_InitDevice(DeviceID_PC, 500, Gimbal_VisionLostCallback, Gimbal_VisionRecoverCallback);
}

// 掉线检测任务回调
void Task_Detect_Callback()
{
	uint32_t presentTime = HAL_GetTick();
	for (uint8_t id = 0; id < DETECT_DEVICE_NUM; id++)
	{
		// 判定是否掉线
		if (presentTime - detectList[id].lastRecieveTime > detectList[id].maxInterval && detectList[id].isLost == 0 && id >= DeviceID_FricMotor1 || (detectList[id].lastLostState == 0 && detectList[id].lastLostState == 1 && id < DeviceID_FricMotor1))
		{
			// 更新标识
			detectList[id].isLost = 1;
			// 判定执行默认还是自定义的处理函数
			if (detectList[id].lostFunc == NULL)
				Detect_DefaultLostHandler(id);
			else
				detectList[id].lostFunc();
		}
		else if (presentTime - detectList[id].lastRecieveTime <= detectList[id].maxInterval && detectList[id].isLost == 1 && id >= DeviceID_FricMotor1 || (detectList[id].lastLostState == 1 && detectList[id].lastLostState == 0 && id < DeviceID_FricMotor1))
		{
			// 判定执行默认还是自定义的处理函数
			if (detectList[id].recoverFunc == NULL)
				Detect_DefaultRecoverHandler(id);
			else
				detectList[id].recoverFunc();
			// 更新标识
			detectList[id].isLost = 0;
		}
	}
}

// 获取设备离线情况
// 返回值 1-离线 0-在线
uint8_t Detect_IsDeviceLost(uint8_t deviceID)
{
	return detectList[deviceID].isLost;
}

void OS_DetectCallback(void const *argument)
{
	osDelay(3000);
	for (;;)
	{
		Task_Detect_Callback();
		osDelay(10);
	}
}
