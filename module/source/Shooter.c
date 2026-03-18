#include "Shooter.h"
#include "RC.h"
#include <stdio.h>
#include "Userfreertos.h"
#include "judge.h"
#include "beep.h"
// #include "range.h"
#include "n20.h"
#include "gimbal.h"
#include "USER_CAN.h"
#include "vision.h"
#include "Slope.h"
#define EN_SHOOTER_TASK // ʹ������

Shooter shooter;
uint8_t shootMaxSpeed = 25;
// uint32_t shoot_interval = 0,Last_tick = 0;
uint32_t t = 0;					// ���Ի����ʱ
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
// ���ϵͳ��ʼ��

void Shooter_Init()
{
	shooter.rockerCtrl = false;			   // ��ʼ״̬�������
	shooter.fricSpd = 6500;				   //
	Slope_Init(&shooter.fricSlope, 140, 0); // Ħ����б��
	Shooter_InitPID();					   // m��ʼ�����pid
	Shooter_RegisterEvents();			   // ע���¼�
	shooter.fricMotor[0].targetSpeed = -0;
	shooter.fricMotor[1].targetSpeed = +0;
	shooter.workState = IDLE;

}

// ��ʼ��shooter UI��״
/*�ڷ���ʱ�뱣֤����ϵͳ�����ѿ��������ɹ�����*/
void Shooter_UI_Init()
{
	/**************************��������� Լ 5.71�����/mm**************************************/
	// 10�����һ���ǰ��ս�ߴ�
	//      Graph_SetLine(&shooter.ui.auxiliaryLine,"AL0",Color_Orange,2,1,865,540,1045,540);  //������
	//      Graph_DrawLine(&shooter.ui.auxiliaryLine,Operation_Add);
	//        osDelay(3);
	//      Graph_SetLine(&shooter.ui.auxiliaryLine,"AL5",Color_Orange,1,1,960,200,960,583);  //����
	//      Graph_DrawLine(&shooter.ui.auxiliaryLine,Operation_Add);
	//        osDelay(3);
	////      Graph_SetText(&shooter.ui.fricClose,"FCS",Color_Orange,4,2,100,680,"Closed!!!",9,20);
	////      Graph_DrawText(&shooter.ui.fricClose,Operation_Add);
	//      Graph_SetText(&shooter.ui.bombBay,"Bay",Color_Orange,4,2,1400,680,"Reload",6,20);
	//      Graph_DrawText(&shooter.ui.bombBay,Operation_Add);
	osDelay(3);
}
/************�ڲ����ߺ���*****************/
// ��ʼ��PID����
void Shooter_InitPID()
{
	Motor_StartCalcAngle(&shooter.triggerMotor);							 // ��ʼ������Ƕ��ۼ�
	PID_Init(&shooter.triggerMotor.anglePID.inner, 15, 0, 0, 6000,30000); // 6 0.15 8//10000
	PID_Init(&shooter.triggerMotor.anglePID.outer, 80, 0,1500, 0, 10000); // 1.5 0 2.3   3.27 maxoutput 9000->7000
	
	PID_Init(&shooter.triggerMotor.speedPID, 10, 0, 0, 7800,12000); // 6 0.15 8//10000

	// 5 0.1 8			1.3 0.01 2.5
	//	DEPID_Init(&shooter.triggerMotor.anglePID.deOuter,1,0,0.04,0,5000,0.3);
	//	PID_Init(&shooter.triggerMotor.anglePID.inner,15,0,4,7800,10000);
	//	PID_Init(&shooter.triggerMotor.anglePID.outer,1.2,0,0.4,0,5000);

	PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // Ħ����
	PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);
	SMCInit(&shooter.fricMotor[0].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // ��
	SMCInit(&shooter.fricMotor[1].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // ��
}

// ע���¼�
void Shooter_RegisterEvents()
{
	// �������̧�𿪹ز���
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnClick, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnUp, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnLongPress, Shooter_SwitchState_KeyCallback);
	// F����Ħ����
	RC_Register(Key_F, CombineKey_None, KeyEvent_OnDown, Shooter_StartFric_KeyCallback);

	// SHIFT+Q���100Ħ����ת��
	RC_Register(Key_Q, CombineKey_Shift, KeyEvent_OnDown, Shooter_IncFricSpeed_KeyCallback);
	// SHIFT+E��С100Ħ����ת��
	RC_Register(Key_E, CombineKey_Shift, KeyEvent_OnDown, Shooter_DecFricSpeed_KeyCallback);
}
//Shooter fricopen openflag
static void Shooter_state(_Bool openflag)
{
	if(openflag == 1){
		Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // Ħ����б��
		Slope_NextVal(&shooter.fricSlope);					  // б����һ��ֵ
		shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // Ħ�����ٶ�
		shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	}
	else{
		Slope_SetTarget(&shooter.fricSlope, 0);								  // Ħ����б��
		Slope_NextVal(&shooter.fricSlope);									  // б����һ��ֵ
		shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // Ħ�����ٶ�
		shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	}
}

// ң�п���
void Shooter_RockerCtrl()
{
	if (rcInfo.right == 1) // �Ҳ����������Ħ����
	{
		shooter.fricOpenFlag = 1; 
	}
	else // ����ر�Ħ����
	{
		shooter.fricOpenFlag = 0; 
	}
	// �󲦸�����(������)��������
	static uint8_t triggerFlag = 0;
	if (rcInfo.left == 1 && triggerFlag == 0)
	{
		if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ��������
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

/*************************RC�¼�**************************
���������ܼ���event����
*********************************************************/
// ����/ֹͣ��������
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (event)
	{
	case KeyEvent_OnClick:			   // ��������
		if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ��������
		{
			shooter.workState = TRIGGER_CLICK;
		}
		break;

	case KeyEvent_OnLongPress:		   // ��������
		if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ��������
		{
			shooter.workState = TRIGGER_CONTINUE;
		}
		break;

	case KeyEvent_OnUp:				   // ������̧��
		if (shooter.fricOpenFlag == 1) // Ħ���ֿ��� ������������
		{
			shooter.workState = IDLE;
		}
		break;
	default:
		break;
	}
}

// �ֶ���/�ر�Ħ����
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricOpenFlag = !shooter.fricOpenFlag;
}



void Shooter_IncFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd += 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // Ħ����б��
	Slope_NextVal(&shooter.fricSlope);					  // б����һ��ֵ
	shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
	shooter.fricMotor[1].targetSpeed = +Slope_GetVal(&shooter.fricSlope);
}

void Shooter_DecFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd -= 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // Ħ����б��
	Slope_NextVal(&shooter.fricSlope);					  // б����һ��ֵ
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

// ��������
bool Heat_Limit()
{
	float d = 0.6; // ��ֵ
	(void)d;	   // ��ֵ
	float k = 1;   // ��ֹ�˷�δʹ������,������ȴ����ϵ��
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
/************************freertos����**********************
����������freertos����ϵͳ����
**********************************************************/
// �������ص�
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
				Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // Ħ����б��
				Slope_NextVal(&shooter.fricSlope);					  // б����һ��ֵ
				shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // Ħ�����ٶ�
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
				Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd);				  // Ħ����б��
				Slope_NextVal(&shooter.fricSlope);									  // б����һ��ֵ
				shooter.fricMotor[0].targetSpeed = Slope_GetVal(&shooter.fricSlope); // Ħ�����ٶ�
				shooter.fricMotor[1].targetSpeed = -Slope_GetVal(&shooter.fricSlope);
			}
		}
	}
	shooter.last_bullet_speed = shooter.bullet_speed;
	if (rcInfo.wheel > 600) // ң���������л�ҡ�˿��ƻ���������
		shooter.rockerCtrl = true;
	else if (rcInfo.wheel < -600)
		shooter.rockerCtrl = false;

	if (shooter.rockerCtrl)
		Shooter_RockerCtrl();
	
	Shooter_state(shooter.fricOpenFlag);
	// ��ת����
	// ����Ƕ���Ŀ��Ƕ�����10������ж�ת�ж�
	if (ABS(shooter.triggerMotor.Total_Position - shooter.triggerMotor.Target_Position) > 10 * 2.5 && shooter.workState != TRIGGER_REVERSE)
	{
		// shooter.block.judgeCnt++;		  // ��ת�ж�������++
		if (shooter.block.judgeCnt > 100) // �������ﵽһ��ֵ�����ж�Ϊ��ת��������ת
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
	else // ��Ŀ��ֵ���С��10�ȣ�����״̬����������ת�ж�����������
	{
		shooter.block.judgeCnt = 0;
	}
	
	Heat_Limit();

	// ����
	switch (shooter.workState)
	{
	case TRIGGER:
	{
		if (JUDGE_IsValid() == false || Heat_Limit() && remainHeat > 20) // δ��װ����ϵͳ �� ����ϵͳʣ����������100 ��������
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.Target_Position -= (360 * 1 / 9.0) * 2.5; // ÿ��ת��1/9Ȧ
				shooter.workState = IDLE;
				shooter.number += 1;
				osDelay(t);
				vision_transmit.bullet_id = vision_receive.bullet_id;
				//						shoot_interval = getCurrentMicros() - Last_tick;//���Է����ӳ�
				//						Last_tick = getCurrentMicros();
			}
		}

		else
			shooter.workState = IDLE;
	}
	break;
	case TRIGGER_CLICK:
	{
		if (JUDGE_IsValid() == false || (Heat_Limit() && remainHeat > 20))// δ��װ����ϵͳ �� ����ϵͳʣ����������100 ��������
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // ÿ��ת��1/9Ȧ
				shooter.workState = IDLE;
				shooter.number += 1;
				osDelay(t);
				vision_transmit.bullet_id = vision_receive.bullet_id;
			}
			else
				shooter.workState = IDLE;
		}
	}
	break;
	case TRIGGER_CONTINUE:
		if (JUDGE_IsValid() == false || (Heat_Limit() && remainHeat > 20)) // δ��װ����ϵͳ �� ����ϵͳʣ����������100 ��������
		{
			if (shooter.triggerMotor.Target_Position - shooter.triggerMotor.Total_Position > -8 * 2.5)
			{
				shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // ÿ��ת��1/9Ȧ
				shooter.number += 1;
				osDelay(t);
				vision_transmit.bullet_id = vision_receive.bullet_id;
			}
		}
		else
			shooter.workState = IDLE;
		break;
	case TRIGGER_REVERSE:
		// ������
		Beep_PlayNotes((Note[]){{T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}, {T_M1, D_Sixteenth}}, 3);
		shooter.triggerMotor.targetAngle += (360 * 1 / 9.0) * 2.5; // ������򲦶�0.5/9Ȧ
		osDelay(500);
		shooter.triggerMotor.targetAngle -= (360 * 1 / 9.0) * 2.5; // ��ת0.5/9Ȧ
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
