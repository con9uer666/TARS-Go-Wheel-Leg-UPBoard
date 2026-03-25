#include "Chassis.h"
#include "Trigger_about.h"


#define EN_CHASSIS_TASK
// #define POWER_GENERATE
#define EN_DIAGONAL 
#define INIT_YAW_ANGLE 32669

int8_t YawLost = 0;
uint8_t UIupdateState=0;
uint16_t SET_WHEELSPEED_MAX = 8000;
static uint8_t last_left_state = 0;
uint8_t upstair_flag = 0;//0：常态；1：上台阶的瞬间
uint8_t leg_button_flag = 0;//0:按下的瞬间 1：按下后持续




void Task_Chassis_Callback(void);

Chassis chassis = {0};

int8_t diagonal_enable ;
extern uint8_t flagRC;

/********************??????************************/
void Chassis_Init()
{
	// 初始化对角线模式
	diagonal_enable = 1;
	chassis.rockerCtrl = false;
	// ?��???????????��??????????????
	chassis.info.wheelbase = 316.57f;
	chassis.info.wheeltrack = 322.70f;
	chassis.info.wheelRadius = 60;
	chassis.info.offsetX = 0; 
	chassis.info.offsetY = 0; 
	// ??????????????
	chassis.move.cap_output = 0;
	// ??��???????????
	chassis.rotate.InitAngle = INIT_YAW_ANGLE - diagonal_enable * 8192 - 16384;	
	if(chassis.rotate.InitAngle>=65536)
		chassis.rotate.InitAngle-=65536;
	if(chassis.rotate.InitAngle<0)
		chassis.rotate.InitAngle+=65536;
	
	chassis.rotate.InitpitchAngle = 1290;
	// ?????��yaw?��???��?????��0-360??
	chassis.motors[0].TurnOffset = 210.0750f - 180.0f  ;//102
	chassis.motors[1].TurnOffset = 142;//UNUSED
	chassis.motors[2].TurnOffset = 170;//UNUSED
	chassis.motors[3].TurnOffset = 15.1391f+300.0f-60.0f;


	// ?��????????????
	Slope_Init(&chassis.move.xSlope, 0.015, 0);
	Slope_Init(&chassis.move.ySlope, 0.015, 0);
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);
	// 4005��????	???
	chassis.motors[0].turn_speed_limit = 12000; // 0-15000;
	chassis.motors[1].turn_speed_limit = 12000; // ???��??��?????0?????????��
	chassis.motors[2].turn_speed_limit = 12000;
	chassis.motors[3].turn_speed_limit = 12000;

	Chassis_InitPID();
//	PowerControl_AutoUpdateParamInit();

	Chassis_RegisterEvents();
}

/************?????��??????*****************/
void Motor_StartCalcAngle_M4005(DoubleMotor *motor)
{
	motor->totalAngle = motor->TurnAngle;
	if (motor->totalAngle > 65536 / 2.0f)
		motor->totalAngle -= 65536;
	motor->lastAngle = motor->TurnAngle;
	motor->targetTurnAngle = 0;
}

void Motor_CalcAngle_M4005(DoubleMotor *motor)
{
	int32_t dAngle = 0;
	if (motor->TurnAngle - motor->lastAngle < -30000)
		dAngle = motor->TurnAngle + (65536 - motor->lastAngle);
	else if (motor->TurnAngle - motor->lastAngle > 30000)
		dAngle = -motor->lastAngle - (65536 - motor->TurnAngle);
	else
		dAngle = motor->TurnAngle - motor->lastAngle;
	motor->totalAngle += dAngle;
	motor->lastAngle = motor->TurnAngle;
}

float Angle_SignedMod(float angle)
{
	if (angle > 180)
	{
		return angle - 360;
	}
	else if (angle < -180)
	{
		return angle + 360;
	}
	else
	{
		return angle;
	}
}

float Angle_UnsignedMod(float angle)
{
	if (angle > 360)
	{
		return angle - 360;
	}
	else if (angle < 0)
	{
		return angle + 360;
	}
	else
	{
		return angle;
	}
}

void Chassis_InitPID()
{
	for (uint8_t i = 0; i < 4; i++)
	{
		PID_Init(&chassis.motors[i].speedPID, 10, 0, 6, 8000, 16000);
		PID_Init(&chassis.motors[i].TurnSpeed_LIMITpid, 178, 0.02, 0, 3000, 9000);
	}

	PID_SetMaxOutput(&chassis.motors[2].anglePID.inner, 2000);
	PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 1, 11);
	PID_Init(&chassis.move.buffer_pid, 3, 0, 0, 0, 80);
	PID_SetDeadzone(&chassis.motors[0].speedPID, 0);
	PID_SetDeadzone(&chassis.motors[1].speedPID, 0);
	PID_SetDeadzone(&chassis.motors[2].speedPID, 0);
	PID_SetDeadzone(&chassis.motors[3].speedPID, 0);
	PID_SetDeadzone(&chassis.rotate.pid, 0.1);
}

// 注册事件
// TODO:wasd斜坡、小陀螺Q、抬腿E、上台阶R、
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); // WASD
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	// WASD
	//QR事件注册vscode://lirentech.file-ref-tags?filePath=Chassis.c&snippet=%2F%2FQR%E4%BA%8B%E4%BB%B6%E6%B3%A8%E5%86%8C
	RC_Register(Key_Q, CombineKey_None, KeyEvent_OnDown, KeyCallback_Q_OnDown);//按Q
	RC_Register(Key_R, CombineKey_None, KeyEvent_OnDown, KeyCallback_R_OnDown);//按R
	RC_Register(Key_E, CombineKey_None, KeyEvent_OnDown, KeyCallback_E_OnDown);	//按E
	RC_Register(Key_Q, CombineKey_None, KeyEvent_OnUp, KeyCallback_Q_OnUp);//按Q
	RC_Register(Key_R, CombineKey_None, KeyEvent_OnUp, KeyCallback_R_OnUp);//按R
	RC_Register(Key_E, CombineKey_None, KeyEvent_OnUp, KeyCallback_E_OnUp);//按E
// 	RC_Register(Key_V, CombineKey_None, KeyEvent_OnDown, Chassis_capOutputChange_KeyCallback);//!已注释
// 	// RC_Register(Key_E | Key_R, CombineKey_None, KeyEvent_OnDown, Chassis_Return_KeyCallback);//!已注释
	
// 	RC_Register(Key_B, CombineKey_None, KeyEvent_OnDown, Chassis_ChangeDiagonal);//!已注释
// 	// SHIFT+B
// 	RC_Register(Key_B, CombineKey_Shift, KeyEvent_OnDown, Chassis_Change_DebugTurn0);//!已注释
// //  RC_Register(Key_V,CombineKey_Shift,KeyEvent_OnDown,Chassis_capBurstChange_KeyCallback);
// //	RC_Register(Key_Shift,CombineKey_None,KeyEvent_OnDown,Cap_On_KeyCallback);
// 	RC_Register(Key_C,CombineKey_None,KeyEvent_OnDown,UI_UPdate);//!已注释
// 	RC_Register(Key_C,CombineKey_None,KeyEvent_OnUp,UI_UPdate);//!已注释

}


void Spin_SpeedUpdate()
{
	chassis.move.maxVw = chassis.move.maxPower * 0.225f + 3.25f; // 0.065 2.8
	if (chassis.move.maxVw <= 0.5f)
	{
		chassis.move.maxVw = 0.5f;
	}

	if (chassis.rotate.mode == ChassisMode_Spin)
	{
		chassis.move.maxVx = 0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f;
		if (chassis.move.maxVx <= 0)
			chassis.move.maxVx = 0;
		chassis.move.maxVy = chassis.move.maxVx;
	}
	else
	{
		chassis.move.maxVx = 5500;
		chassis.move.maxVy = 5500;
	}
}

float keyboard_speed_max = 2.7f;

// 底盘目标解算
void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
	Slope_NextVal(&chassis.move.spinSlope);
	Slope_SetTarget(&chassis.move.ySlope, keyboard_speed_max * (chassis.key.key_w + chassis.key.key_s)); // 前后
	Slope_SetTarget(&chassis.move.xSlope, keyboard_speed_max * (chassis.key.key_d + chassis.key.key_a)); // 左右
	Spin_SpeedUpdate();
	if (JUDGE_GetChassisPowerLimit() > 70)
	{
		for (uint8_t i = 0; i < 4; i++)
			PID_SetMaxOutput(&chassis.motors[i].anglePID.inner, 800);
	}
	else
	{
		for (uint8_t i = 0; i < 4; i++)
			PID_SetMaxOutput(&chassis.motors[i].anglePID.inner, 1500);
	}
	uint16_t SET_WHEELSPEED_MAX = 7500;
	fp32 rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;
	chassis.move.maxVx = SET_WHEELSPEED_MAX / 60.0f / (268.f/17.f) * 2 * PI * chassis.info.wheelRadius;
	chassis.move.maxVy = chassis.move.maxVx;
	chassis.move.maxVw = 6000 / rotateRatio / 60.0f / (268.f/17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f/1.414f;
}
// 遥控器回调
// void Chassis_RockerCtrl()
// {
	
// 	if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2 && rcInfo.right != 1)
// 		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown); // g?????????????
// 	else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
// 		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown);
// 	// ?��?��????��??��?��????
// 	Slope_SetTarget(&chassis.move.xSlope, (float)rcInfo.ch3 * chassis.move.maxVx / 660); // ?????????
// 	Slope_SetTarget(&chassis.move.ySlope, (float)rcInfo.ch4 * chassis.move.maxVy / 660);
// }	
// ?????��??????UI??��?
char *Chassis_GetModeText()
{
	switch (chassis.rotate.mode)
	{
	case ChassisMode_Spin:
		return "Spin  ";
	case ChassisMode_Follow:
		return "Follow";
		;
	default:
		return NULL;
	}
}
// 
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		chassis.key.key_w = 1.0f;
		break;
	case Key_S:
		chassis.key.key_s = -1.0f;
		break;
	case Key_D:
		chassis.key.key_d = 1.0f;
		break;
	case Key_A:
		chassis.key.key_a = -1.0f;
		break;
	default:
		break;
	}
}
void UI_UPdate(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// if(event == KeyEvent_OnDown)
	// 		UIupdateState=1;
	// else
	// 		UIupdateState=0;
}

//
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
		case Key_W:
			chassis.key.key_w = 0;
			break;
		case Key_S:
			chassis.key.key_s = 0;
			break;
		case Key_D:
			chassis.key.key_d = 0;
			break;
		case Key_A:
			chassis.key.key_a = 0;
			break;
		default:
			break;
	}
}

// ????????
void Chassis_capOutputChange_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// if (chassis.move.fastMode == 0 && cap.receive_data.max_power > 80)
	// {
	// 	chassis.move.fastMode = 1;
	// 	chassis.move.cap_output = 1;
	// 	return;
	// }
	// if (chassis.move.fastMode == 1)
	// 	chassis.move.fastMode = 0;
}
// ????????
//  void Cap_On_KeyCallback(KeyType key,KeyCombineType combine,KeyEventType event)
//{
//	if(chassis.move.cap_output==0)
//	{
//		chassis.move.cap_output=1;
//		return;
//	}
//  }
//  void Cap_Off_KeyCallback(KeyType key,KeyCombineType combine,KeyEventType event)
//{
//	if(chassis.move.cap_output==1)
//	{
//		chassis.move.cap_output=0;
//		return;
//	}
//  }
void Chassis_ChangeDiagonal(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// if(key == Key_B)
	// {
	// 	if(diagonal_enable!=0)
	// 	{
	// 		diagonal_enable=0;
	// 		chassis.rotate.mode = ChassisMode_Follow;
	// 	}
	// 	else
	// 		diagonal_enable=1;
	// }
	// chassis.rotate.InitAngle = INIT_YAW_ANGLE - diagonal_enable * 8192-16384;	
	// if(chassis.rotate.InitAngle>=65536)
	// 	chassis.rotate.InitAngle-=65536;
	// if(chassis.rotate.InitAngle<0)
	// 	chassis.rotate.InitAngle+=65536;
}

void Chassis_Change_DebugTurn0(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// if(chassis.motors[0].TurnOffset == 230.6964f)
	// 	chassis.motors[0].TurnOffset = 230.6964f + 38;
	// else chassis.motors[0].TurnOffset=230.6964f;
}


KeyProtectSingleTrigger_Def Key_R_Protect = {0};
//QR回调vscode://lirentech.file-ref-tags?filePath=Chassis.c&snippet=%2F%2FQR%E5%9B%9E%E8%B0%83
		void KeyCallback_Q_OnDown(KeyType key, KeyCombineType combine, KeyEventType event)
		{
			Foot_Chassis.Chassis_Mode = 1;
		}

		void KeyCallback_R_OnDown(KeyType key, KeyCombineType combine, KeyEventType event)
		{					
			upstair_flag = KeyProtectSingleTrigger_Update(&Key_R_Protect, 1, 10);//10次 确保发出
			Foot_Chassis.Target_Leg_State = 0;
		}

		void KeyCallback_Q_OnUp(KeyType key, KeyCombineType combine, KeyEventType event)
		{
			Foot_Chassis.Chassis_Mode = 0;
		}

		void KeyCallback_R_OnUp(KeyType key, KeyCombineType combine, KeyEventType event)
		{
			upstair_flag = KeyProtectSingleTrigger_Update(&Key_R_Protect, 0, 10);
		}

		void KeyCallback_E_OnDown(KeyType key, KeyCombineType combine, KeyEventType event)
		{
			if(leg_button_flag == 0)
			{
				if(Foot_Chassis.Target_Leg_State == 0)
					Foot_Chassis.Target_Leg_State = 1;
				else
					Foot_Chassis.Target_Leg_State = 0;
			}
			leg_button_flag = 1;
		}

		void KeyCallback_E_OnUp(KeyType key, KeyCombineType combine, KeyEventType event)
		{
			leg_button_flag = 0;
		}



void Chassis_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// switch (key)
	// {
	// case Key_E:
	// 	gimbal.yaw.targetAngle += 180;
	// 	break;
	// case Key_R:
	// 	gimbal.yaw.targetAngle -= 180;
	// default:
	// 	break;
	// }
}

uint8_t spin_switch = 0;

// //老代码，底盘回调
// void Task_Chassis_Callback()
// {
// 	chassis.rotate.nowAngle = gimbal.yawMotor_M4005.angle;
// 	chassis.rotate.relativeAngle = ((chassis.rotate.nowAngle - chassis.rotate.InitAngle));
//     if (rcInfo.wheel > 600)
//         chassis.rockerCtrl = true;
//     else if (rcInfo.wheel < -600)
//         chassis.rockerCtrl = false;

//     if (chassis.rockerCtrl)
//         Chassis_RockerCtrl();

//     last_left_state = rcInfo.left;

//     Chassis_UpdateSlope();

//     float gimbalAngleSin = arm_sin_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);
//     float gimbalAngleCos = arm_cos_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);

//     chassis.move.vx = (Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);

//     chassis.move.vy = (-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);

//     if (chassis.rotate.mode == ChassisMode_Follow)
//     {
//         Slope_SetTarget(&chassis.move.spinSlope, 0);
//         if (chassis.rotate.relativeAngle > 180)
//             chassis.rotate.relativeAngle -= 360;
//         if (chassis.rotate.relativeAngle < -180)
//             chassis.rotate.relativeAngle += 360;
//         if (YawLost == 0)
//         {
//             PID_SingleCalc(&chassis.rotate.pid, 0, chassis.rotate.relativeAngle);
//         }
//         else
//         {
//             chassis.rotate.relativeAngle = 0;
//             chassis.rotate.pid.output = 0;
//         }
//         chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;

//         LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
//     }
//     else if (chassis.rotate.mode == ChassisMode_Spin) 
//     {
//         chassis.move.vw = -chassis.move.spinSlope.value;

//         float ratio;
//         if (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
//             ratio = 0.6;
//         else
//             ratio = 1.0f;
//         if (chassis.rockerCtrl)
//         {
//             if (rcInfo.left == 2 && rcInfo.wheel > 600 && flagRC == 1)
//             {
//                 if (spin_switch == 0)
//                     Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
//                 else if (spin_switch == 1)
//                     Slope_SetTarget(&chassis.move.spinSlope, -chassis.move.maxVw * ratio);
//                 else
//                     Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);

//                 spin_switch++;
//                 spin_switch = spin_switch % 2;
//             }
//         }
//     }
// }

float Foot_Target_Speed;
float Foot_Relative_Angle;
float Foot_Target_Relative_Angle;

Foot_Chassis_t Foot_Chassis = {0};
int times;

float sloped_Target_Vx = 0;
float sloped_raw_Target_Vy = 0;

// extern uint8_t upstair_flag_usable;
KeyProtectSingleTrigger_Def upstair_flag_protect = {0};

//控腿函数
void Foot_CallBack(void)
{
	//解锁控制
	if (rcInfo.wheel > 600)
        chassis.rockerCtrl = true;
    else if (rcInfo.wheel < -600)
        chassis.rockerCtrl = false;

	//控制腿长切换计时器
	if(times > 0)
	{
		times --;
	}

	if (chassis.rockerCtrl)//解锁后
	{
		//目标速度
		Foot_Chassis.Target_Vx = (float)rcInfo.ch3 * 2.7 / 660;
		Foot_Chassis.Target_Vy = (float)rcInfo.ch4 * 2.7 / 660;

		if(rcInfo.left == 2 && Foot_Chassis.Target_Leg_State == 0)//下
			Foot_Chassis.Chassis_Mode = 1;
		else if (rcInfo.left == 3 || (rcInfo.left == 2 && Foot_Chassis.Target_Leg_State == 1))//中或（下且长腿）
			Foot_Chassis.Chassis_Mode = 0;
		
		//控制腿长
		if(rcInfo.left == 1 && rcInfo.right != 1 && times == 0)//左上，右不急停
		{
			times = 300;//每300ms允许切换一次腿长
			if(Foot_Chassis.Target_Leg_State == 0)
				Foot_Chassis.Target_Leg_State = 1;
			else if (Foot_Chassis.Target_Leg_State == 1)
				Foot_Chassis.Target_Leg_State = 0;
		}

		if(rcInfo.left == 2 && Foot_Chassis.Target_Leg_State == 1)
		{
			upstair_flag = KeyProtectSingleTrigger_Update(&upstair_flag_protect, 1, 10);//10次 确保发出
			Foot_Chassis.Target_Leg_State = 0;
		}
		else 
		{
			upstair_flag = KeyProtectSingleTrigger_Update(&upstair_flag_protect, 0, 10);
		}
		// if(rcInfo.left == 3)
		// {
		// 	upstair_flag_usable = 1;
		// }
	}
	else
	{
		Chassis_UpdateSlope();

		// Foot_Chassis.Target_Vx = chassis.move.xSlope.value;
		Foot_Chassis.Target_Vy = chassis.move.ySlope.value;
	}
}	

//下板底盘任务
#ifdef EN_CHASSIS_TASK
void OS_ChassisCallback(void const *argument)
{
	Chassis_RegisterEvents();

	osDelay(500);
	for (;;)
	{
		// Task_Chassis_Callback();
		Foot_CallBack();
		osDelay(2);
	}
}

#endif
