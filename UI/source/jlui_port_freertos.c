
#include "jlui.h"
#include "usart.h"
#include <freertos.h>
#include <semphr.h>
#include "userfreertos.h"
#include "judge.h"
#include "super_cap.h"
#include "chassis.h"
#include "gimbal.h"
#include "chassis.h"
#include "vision.h"
#include "shooter.h"
#define EN_UI_TASK

// 注1：对FreeRTOS移植时，需要在使用JLUI库前传递一个已经创建好的互斥锁给JLUI库。
// 互斥锁的创建方式如下：
// void *mutex = xSemaphoreCreateMutex();
// JLUI_SetMutexObject(mutex);
//
// 注2：在FreeRTOS中使用JLUI库时，调用10Hz Tick请使用FreeRTOS定时器；不要用中断服务函数，
// 这里给出的互斥锁移植是不能在中断中使用的！！！
// 创建定时器的方式如下：
// TimerHandle_t timer = xTimerCreate("JLUI_Tick", pdMS_TO_TICKS(100), pdTRUE, NULL, JLUI_Tick);
// xTimerStart(timer, 0);
// JLUI_Tick函数中调用JLUI_Tick10Hz即可：
// void JLUI_Tick(TimerHandle_t xTimer) { JLUI_10HzTick(); }

// 【用户需要实现此函数】
// 对互斥锁上锁。如果成功获取了锁，返回1；否则返回0。
int JLUI_MutexLock(void *mutex)
{
	// return xSemaphoreTake((SemaphoreHandle_t)mutex, 10) == pdTRUE;
	// 容易在各种电控的车上出现问题，先禁用互斥锁。后续可能也不需要使用锁
	return 1;
}

// 【用户需要实现此函数】
// 对互斥锁解锁。
void JLUI_MutexUnlock(void *mutex)
{
	//	xSemaphoreGive((SemaphoreHandle_t)mutex);
}

// 【用户需要实现此函数】
// 从串口向裁判系统发送数据函数。注意！从函数返回后，data缓冲区即应视为失效。如要做异步发送，请自行拷贝data。
uint8_t x2;
void JLUI_SendData(const uint8_t *data, size_t len)
{
	// TODO: !!!
	//		huart6.gState = HAL_UART_STATE_READY;//由于hal库上锁机制有问题所以暴力解锁
	//		HAL_UART_Transmit(&huart6, data, len, 1000);
	//	HAL_UART_Transmit_IT(&huart6, data, len);
}

#ifdef EN_UI_TASK
uint8_t UI_time = 100;
float anglestart = 0;
float angleend = 0;
	Uiid VisionFound;

// void OS_UICallBack(void const *argument)
// {
// 	//	static int
// 	//        color666 = UiTeam,
// 	//        colorFslash = UiBlack,
// 	//        colorBslash = UiCyan;
// 	//    static int tickCount = 0; // (tick / 50) % 2 == 0时设置为State1，反之为State2
// 	//    static int currentState = 1;
// 	//    const char* stateString[] = {"State1", "State2"};

// 	uint32_t cnt = 0;
// 	uint32_t Current_HP;
// 	uint32_t Last_HP = JUDGE_GetHP();
// 	while (!JUDGE_GetSelfID())
// 		osDelay(1000);
// 	JLUI_SetSenderReceiverId(JUDGE_GetSelfID(), JUDGE_GetClientID());

// 	// 测试用例
// 	//		JLUI_CreateString(4,UiMagenta,0,600,130,20,"slow");
// 	//		JLUI_CreateString(4,UiGreen,0,600,180,20,"fast");
// 	//		Uiid test666 = JLUI_CreateInt(5, color666, 0, 600, 400, 30, 666);
// 	//		Uiid Fslash = JLUI_CreateLine(5, colorFslash, 1, 0, 0, 1920, 1080);
// 	//		Uiid Bslash = JLUI_CreateLine(20, colorBslash, 1, 0, 1080, 1920, 0);
// 	//		Uiid uiTick = JLUI_CreateInt(5, UiTeam, 2, 600, 230, 30, 0);
// 	//		Uiid changingString = JLUI_CreateString(5, UiWhite, 2, 700, 150, 20, stateString[0]);

// 	// 超电能量条
// 	Uiid Super_Rec = JLUI_CreateRect(5, UiMagenta, 0, 800, 140, 1200, 80);
// 	Uiid Super_Line = JLUI_CreateLine(40, UiYellow, 0, 800, 110, 1200, 110);
// 	osDelay(3);

// 	////受击未开小陀螺提示
// 	//    Uiid HP_Dec;
// 	//    HP_Dec = JLUI_CreateString(4,UiMagenta,0,750,700,40,"Please Spin");
// 	//		osDelay(3);

// 	// 视觉识别提示
// 	VisionFound = JLUI_CreateCircle(5, UiTeam, 0, 960, 538, 200);
// 	osDelay(3);
// 	// 车头方位提示
// 	Uiid headstock;
// 	headstock = JLUI_CreateArc(5, UiMagenta, 0, 960, 538, 280, 280, 0, 0);
// 	osDelay(3);
// 	// 四种视觉模式
// 	Uiid VisionMode;
// 	JLUI_CreateString(4, UiBlack, 0, 100, 600, 20, "VisionMode:");
// 	VisionMode = JLUI_CreateInt(5, UiOrange, 0, 320, 600, 20, vision_transmit.task_mode);
// 	osDelay(3);
// 	// 摩擦轮转速
// 	Uiid FricSpeed;
// 	JLUI_CreateString(4, UiOrange, 0, 100, 550, 20, "FricSpeed:");
// 	FricSpeed = JLUI_CreateInt(5, UiOrange, 0, 300, 550, 20, shooter.fricMotor[0].speed);
// 	osDelay(3);

// 	// 超电开启状态
// 	Uiid SuperCap;
// 	SuperCap = JLUI_CreateCircle(5, UiGreen, 0, 960, 538, 220);
// 	osDelay(3);
// 	// 超电爆发状态
// 	Uiid CAPBurst;
// 	CAPBurst = JLUI_CreateCircle(5, UiMagenta, 0, 960, 538, 240);
// 	osDelay(3);
// 	////底盘状态
// 	//		Uiid Chassis;
// 	//		JLUI_CreateString(4,UiOrange,0,750,780,20,Chassis_GetModeText());
// 	//    osDelay(3);
// 	// 弹舱盖
// 	Uiid ShooterCover;
// 	ShooterCover = JLUI_CreateLine(12, UiOrange, 0, 1400, 630, 1460, 630);
// 	osDelay(3);
// 	// 车宽线

// 	for (;;)
// 	{
// 		osDelay(UI_time);
// 		if (UI_time > 35)
// 			UI_time -= 2;
// 		JLUI_10HzTick();
// 		//      cnt++;
// 		anglestart = chassis.rotate.relativeAngle - 15;
// 		angleend = chassis.rotate.relativeAngle + 15;
// 		if (anglestart < 0)
// 			anglestart += 360;
// 		if (angleend > 360)
// 			angleend -= 360;
// 		if (angleend < 0)
// 			angleend += 360;
// 		JLUI_SetStartAngle(headstock, anglestart);
// 		JLUI_SetEndAngle(headstock, angleend);
// 		// 刷新超电能量条
// 		JLUI_MoveP2To(Super_Line, 800 + cap.per_energy * 4, 110);
// 		// 小陀螺状态
// 		//				if(chassis.rotate.mode == ChassisMode_Spin)
// 		//						JLUI_SetVisible(HP_Dec, 0);
// 		//				if(chassis.rotate.mode == ChassisMode_Follow)
// 		//						JLUI_SetVisible(HP_Dec, 1);
// 		//				if(cnt%10 ==0)
// 		//				{
// 		//						Current_HP = JUDGE_GetHP();
// 		//						if(Last_HP-Current_HP>=10)
// 		//						{
// 		//								Last_HP = Current_HP;
// 		//								if(chassis.rotate.mode == ChassisMode_Follow)
// 		//										JLUI_SetVisible(HP_Dec, 1);
// 		//						}
// 		//						else if(Last_HP<=Current_HP)
// 		//						{
// 		//								Last_HP = Current_HP;
// 		//								JLUI_SetVisible(HP_Dec, 0);
// 		//						}
// 		//				}
// 		// 刷新视觉识别状态
// 		if (visionFindAver > 0.8)
// 			JLUI_SetVisible(VisionFound, 1);
// 		else
// 			JLUI_SetVisible(VisionFound, 0);
// 		// 更新视觉模式
// 		JLUI_SetInt(VisionMode, vision_transmit.task_mode);
// 		// 更新摩擦轮转速
// 		JLUI_SetInt(FricSpeed, shooter.fricMotor[0].speed);
// 		// 更新超电开启状态
// 		if (chassis.move.cap_output != 2)
// 		{
// 			JLUI_SetVisible(CAPBurst, 0);
// 		}
// 		if (chassis.move.cap_output == 1)
// 		{
// 			JLUI_SetVisible(SuperCap, 1);
// 		}
// 		if (chassis.move.cap_output != 1)
// 			JLUI_SetVisible(SuperCap, 0);

// 		// 更新弹舱盖状态
// 		if (shooter.box == 0)
// 			JLUI_SetVisible(ShooterCover, 1);
// 		if (shooter.box == 1)
// 			JLUI_SetVisible(ShooterCover, 0);
// 	}
// }
#endif
