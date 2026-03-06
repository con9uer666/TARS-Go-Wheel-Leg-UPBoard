#include "Shooter.h"
#include "RC.h"
#include <stdio.h>
#include "Userfreertos.h"
#include "judge.h"
#include "beep.h"
#include "range.h"
#include "n20.h"
#include "gimbal.h"
#include "USER_CAN.h"
#include "vision.h"
#include "Slope.h"
#define EN_SHOOTER_TASK // 使能任务

Shooter shooter;
uint8_t shootMaxSpeed = 25;
// uint32_t shoot_interval = 0,Last_tick = 0;
uint32_t t = 0;					// 线性火控延时
extern int16_t remainHeat;
uint8_t debugCnt = 0;

void Shooter_InitPID(void);
void Shooter_RegisterEvents(void);
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_CoverOpen_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_CoverClose_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_ChangeLineIncHigh_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_IncFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_DecFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_InquireBulletState(void);
// 射击系统初始化

void Shooter_Init()
{
	shooter.rockerCtrl = false;			   // 初始状态键鼠控制
	shooter.fricSpd = 6000;				   //
	Slope_Init(&shooter.fricSlope, 140, 0); // 摩擦轮斜坡
	Shooter_InitPID();					   // m初始化电机pid
	Shooter_RegisterEvents();			   // 注册事件
	shooter.fricMotor[0].targetSpeed = -0;
	shooter.fricMotor[1].targetSpeed = +0;
	shooter.workState = IDLE;

}

// 初始化shooter UI形状
/*在发送时须保证裁判系统串口已开启，并成功解析*/
void Shooter_UI_Init()
{
	/**************************射击辅助线 约 5.71坐标点/mm**************************************/
	// 10米左右击打前哨战线簇
	//      Graph_SetLine(&shooter.ui.auxiliaryLine,"AL0",Color_Orange,2,1,865,540,1045,540);  //中心线
	//      Graph_DrawLine(&shooter.ui.auxiliaryLine,Operation_Add);
	//        osDelay(3);
	//      Graph_SetLine(&shooter.ui.auxiliaryLine,"AL5",Color_Orange,1,1,960,200,960,583);  //竖线
	//      Graph_DrawLine(&shooter.ui.auxiliaryLine,Operation_Add);
	//        osDelay(3);
	////      Graph_SetText(&shooter.ui.fricClose,"FCS",Color_Orange,4,2,100,680,"Closed!!!",9,20);
	////      Graph_DrawText(&shooter.ui.fricClose,Operation_Add);
	//      Graph_SetText(&shooter.ui.bombBay,"Bay",Color_Orange,4,2,1400,680,"Reload",6,20);
	//      Graph_DrawText(&shooter.ui.bombBay,Operation_Add);
	osDelay(3);
}
/************内部工具函数*****************/
// 初始化PID参数
void Shooter_InitPID()
{
	Motor_StartCalcAngle(&shooter.triggerMotor);							 // 初始化电机角度累计
	PID_Init(&shooter.triggerMotor.anglePID.inner, 15, 0, 0, 6000,30000); // 6 0.15 8//10000
	PID_Init(&shooter.triggerMotor.anglePID.outer, 80, 0,1500, 0, 10000); // 1.5 0 2.3   3.27 maxoutput 9000->7000
	
	PID_Init(&shooter.triggerMotor.speedPID, 10, 0, 0, 7800,12000); // 6 0.15 8//10000

	// 5 0.1 8			1.3 0.01 2.5
	//	DEPID_Init(&shooter.triggerMotor.anglePID.deOuter,1,0,0.04,0,5000,0.3);
	//	PID_Init(&shooter.triggerMotor.anglePID.inner,15,0,4,7800,10000);
	//	PID_Init(&shooter.triggerMotor.anglePID.outer,1.2,0,0.4,0,5000);

	PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮
	PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);
	SMCInit(&shooter.fricMotor[0].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // 左
	SMCInit(&shooter.fricMotor[1].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // 右
}

// 注册事件
void Shooter_RegisterEvents()
{
	// 左键按下抬起开关拨弹
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnClick, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnUp, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnLongPress, Shooter_SwitchState_KeyCallback);
	// F开启摩擦轮
	RC_Register(Key_F, CombineKey_None, KeyEvent_OnDown, Shooter_StartFric_KeyCallback);

	// SHIFT+Q提高100摩擦轮转速
	RC_Register(Key_Q, CombineKey_Shift, KeyEvent_OnDown, Shooter_IncFricSpeed_KeyCallback);
	// SHIFT+E减小100摩擦轮转速
	RC_Register(Key_E, CombineKey_Shift, KeyEvent_OnDown, Shooter_DecFricSpeed_KeyCallback);
}
//Shooter fricopen openflag
static void Shooter_state(_Bool openflag)
{
	if(openflag == 1){
		Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
		Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
		shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
		shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	}
	else{
		Slope_SetTarget(&shooter.fricSlope, 0);								  // 摩擦轮斜坡
		Slope_NextVal(&shooter.fricSlope);									  // 斜坡下一个值
		shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
		shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	}
}

// 遥感控制
void Shooter_RockerCtrl()
{
	if (rcInfo.right == 1) // 右拨杆向上则打开摩擦轮
	{
		shooter.fricOpenFlag = 1; 
	}
	else // 否则关闭摩擦轮
	{
		shooter.fricOpenFlag = 0; 
	}
	// 左拨杆向上(上升沿)则开启拨弹
	static uint8_t triggerFlag = 0;
	if (rcInfo.left == 1 && triggerFlag == 0)
	{
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
		{
			shooter.workState = TRIGGER;
			triggerFlag = 1;
		}
	}
	else if (rcInfo.left != 1)
	{
		shooter.workState = IDLE;
		triggerFlag = 0;
	}
}

/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/
// 触发/停止拨弹工作
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (event)
	{
	case KeyEvent_OnClick:			   // 单发拨弹
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
		{
			shooter.workState = TRIGGER_CLICK;
		}
		break;

	case KeyEvent_OnLongPress:		   // 连发拨弹
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
		{
			shooter.workState = TRIGGER_CONTINUE;
		}
		break;

	case KeyEvent_OnUp:				   // 鼠标左键抬起
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 ，不允许拨弹
		{
			shooter.workState = IDLE;
		}
		break;
	default:
		break;
	}
}

// 手动打开/关闭摩擦轮
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricOpenFlag = !shooter.fricOpenFlag;
}



void Shooter_IncFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd += 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
	Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
	shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	shooter.fricMotor[1].targetSpeed = +Slope_GetVal(&shooter.fricSlope);
}

void Shooter_DecFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd -= 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
	Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
	shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	shooter.fricMotor[1].targetSpeed = +Slope_GetVal(&shooter.fricSlope);
}
uint32_t getCurrentMicros(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	uint32_t m = HAL_GetTick();
	__IO uint32_t v = SysTick->VAL;
	// If an overflow happened since we disabled irqs, it cannot have been
	// processed yet, so increment m and reload VAL to ensure we get the
	// post-overflow value.
	if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
	{
		++m;
		v = SysTick->VAL;
	}

	// Restore irq status
	__set_PRIMASK(primask);

	const uint32_t tms = SysTick->LOAD + 1;
	return (m * 1000 + ((tms - v) * 1000) / tms);
}

// 热量控制
bool Heat_Limit()
{
	float d = 0.6; // 阈值
	(void)d;	   // 阈值
	float k = 1;   // 防止浪费未使用热量,增加冷却缩减系数
	if (JUDGE_GetRemainHeat() < 20)
	{
		k = 5;
		return true;
	}
	else if (JUDGE_GetRemainHeat() <= 35)
	{

		//		osDelay(100);
		//		shooter.triggerMotor.anglePID.outer.maxOutput=(J-UDGE_GetCoolingValue()/10)*60*36/8;
		k = 1;
		t = k * 1000 * 10 / JUDGE_GetCoolingValue() * 1.01;

		return true;
	}
	else if (JUDGE_GetRemainHeat() <= 100)
	{
		//		osDelay(100);
		//		shooter.triggerMotor.anglePID.outer.maxOutput=(((JUDGE_GetRemainHeat()/(JUDGE_GetHeatLimit()*(1-d)))*(18-JUDGE_GetCoolingValue()/10))+(JUDGE_GetCoolingValue()/10))*60*36/8;

		//		t = k*1000*10*(-JUDGE_GetRemainHeat()+JUDGE_GetHeatLimit()*(1-d))/(JUDGE_GetHeatLimit()*(1-d)-10)/JUDGE_GetCoolingValue();
		k = 0.5;
		t = k * 1000 * 10 * (-JUDGE_GetRemainHeat() + 100) / (100 - 30) / JUDGE_GetCoolingValue();

		return true;
	}
	else
	{
		//		shooter.triggerMotor.anglePID.outer.maxOutput=5000;
		t = 0;
		k = 0.5;
		return true;
	}
}
/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/
// 射击任务回调
float heat;
extern float initialSpeed;

void Task_Shooter_Callback()
{
	static uint8_t over_speed;
	static uint8_t less_speed;
	heat = JUDGE_GetRemainHeat();
	if (ShootData.initial_speed > 10)
		shooter.bullet_speed = ShootData.initial_speed;
	if (shooter.bullet_speed != shooter.last_bullet_speed)
	{
		if (shooter.bullet_speed > shootMaxSpeed - 1.9)
		{
			over_speed++;
			if (over_speed >= 1)
			{
				over_speed = 0;
				shooter.fricSpd -= 10;
				Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
				Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
				shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
				shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
			}
		}
		if (shooter.bullet_speed < shootMaxSpeed - 2.1)
		{
			less_speed++;
			if (less_speed >= 1)
			{
				less_speed = 0;
				shooter.fricSpd += 10;
				if(shooter.bullet_speed<22.0)
						shooter.fricSpd += 50;
				Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd);				  // 摩擦轮斜坡
				Slope_NextVal(&shooter.fricSlope);									  // 斜坡下一个值
				shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
				shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
			}
		}
	}
	shooter.last_bullet_speed = shooter.bullet_speed;
	if (rcInfo.wheel > 600) // 遥控器拨轮切换摇杆控制还是鼠标控制
		shooter.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		shooter.rockerCtrl = false;

	if (shooter.rockerCtrl)
		Shooter_RockerCtrl();
	
	Shooter_state(shooter.fricOpenFlag);
	// 堵转处理
	// 电机角度与目标角度相差超过10度则进行堵转判定
	if (ABS(shooter.triggerMotor.Total_Position - shooter.triggerMotor.Target_Position) > 10 * 2.5 && shooter.workState != TRIGGER_REVERSE)
	{
		// shooter.block.judgeCnt++;		  // 堵转判定计数器++
		if (shooter.block.judgeCnt > 100) // 计数器达到一定值，则判定为堵转，触发反转
		{
			shooter.block.judgeCnt = 0;
			shooter.block.reverseCnt++;
			shooter.workState = TRIGGER_REVERSE;
			if(shooter.block.reverseCnt > 5)
			{
					shooter.block.state = 1;
			}	
			else
				shooter.block.state = 0;
		}
	}
	else // 与目标值相差小于10度，拨弹状态正常，将堵转判定计数器归零
	{
		shooter.block.judgeCnt = 0;
	}
	
	Heat_Limit();

	// 拨弹
	switch (shooter.workState)
	{
	case TRIGGER:
	{
		if (JUDGE_IsValid() == false || Heat_Limit() && remainHeat > 20) // 未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.Target_Position -= (360 * 1 / 9.0) * 2.5; // 每次转动1/9圈
				shooter.workState = IDLE;
				shooter.number += 1;
				osDelay(t);

				//						shoot_interval = getCurrentMicros() - Last_tick;//测试发弹延迟
				//						Last_tick = getCurrentMicros();
			}
		}

		else
			shooter.workState = IDLE;
	}
	break;
	case TRIGGER_CLICK:
	{
		if (JUDGE_IsValid() == false || Heat_Limit() && remainHeat > 20) // 未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // 每次转动1/9圈
				shooter.workState = IDLE;
				shooter.number += 1;
				osDelay(t);
			}
			else
				shooter.workState = IDLE;
		}
	}
	break;
	case TRIGGER_CONTINUE:
		if (JUDGE_IsValid() == false || Heat_Limit() && remainHeat > 20) // 未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // 每次转动1/9圈
				shooter.number += 1;
				osDelay(t);
			}
		}
		else
			shooter.workState = IDLE;
		break;
	case TRIGGER_REVERSE:
		// 嘀嘀嘀
		Beep_PlayNotes((Note[]){{T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}}, 3);
		shooter.triggerMotor.targetAngle += (360 * 1 / 9.0) * 2.5; // 电机反向拨动0.5/9圈
		osDelay(500);
		shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // 正转0.5/9圈
		shooter.workState = IDLE;

		break;
	default:
		break;
	}
}

#ifdef EN_SHOOTER_TASK
void OS_ShooterCallback(void const *argument)
{
	osDelay(100);
	Shooter_Init();
	PID_Clear(&shooter.triggerMotor.anglePID.inner);
	PID_Clear(&shooter.triggerMotor.anglePID.outer);
	osDelay(100);
	for (;;)
	{
		if (JUDGE_GetShooterOutputState() || 1)
		{
			Task_Shooter_Callback();
		}
		osDelay(10);
	}
}
#endif
