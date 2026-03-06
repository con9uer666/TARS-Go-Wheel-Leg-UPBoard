#include "Chassis.h"
#include "RC.h"
#include "arm_math.h"
#include "rc.h"
#include "UserFreertos.h"
#include "gimbal.h"
#include "super_cap.h"
#include "detect.h"
#include "Moto.h"
#include "USER_CAN.h"
#include "Beep.h"
#include "struct_typedef.h"
#include "math.h"
#include "detect.h"
#include "vision.h"
#include "RLS.h"

#define EN_CHASSIS_TASK
// #define POWER_GENERATE
#define EN_DIAGONAL 
#define INIT_YAW_ANGLE 32669

int8_t YawLost = 0;
uint8_t UIupdateState=0;
uint16_t SET_WHEELSPEED_MAX = 8000;
static uint8_t last_left_state = 0;

//680

void Chassis_InitPID(void);
void Chassis_RegisterEvents(void);
char *Chassis_GetModeText(void);
void Chassis_UpdateSlope(void);
void Chassis_RockerCtrl(void);
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Turn_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_TurnSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_capOutputChange_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_capBurstChange_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_ChangeDiagonal(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Change_DebugTurn0(KeyType key, KeyCombineType combine, KeyEventType event);
void UI_UPdate(KeyType key, KeyCombineType combine, KeyEventType event);
void Cap_On_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Cap_Off_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Motor_StartCalcAngle_M4005(DoubleMotor *motor);
void Motor_CalcAngle_M4005(DoubleMotor *motor);

void Task_Chassis_Callback(void);

Chassis chassis = {0};

int8_t diagonal_enable ;
extern uint8_t flagRC;

/********************??????************************/
void Chassis_Init()
{
	//??????????
	diagonal_enable = 1;
	chassis.rockerCtrl = false;
	// ?¡Á???????????¡§??????????????
	chassis.info.wheelbase = 316.57f;
	chassis.info.wheeltrack = 322.70f;
	chassis.info.wheelRadius = 60;
	chassis.info.offsetX = 0; 
	chassis.info.offsetY = 0; 
	// ??????????????
	chassis.move.cap_output = 0;
	// ??¡Á???????????
	chassis.rotate.InitAngle = INIT_YAW_ANGLE - diagonal_enable * 8192 - 16384;	
	if(chassis.rotate.InitAngle>=65536)
		chassis.rotate.InitAngle-=65536;
	if(chassis.rotate.InitAngle<0)
		chassis.rotate.InitAngle+=65536;
	
	chassis.rotate.InitpitchAngle = 1290;
	// ?????¡Àyaw?¨¢???¨²?????¡§0-360??
	chassis.motors[0].TurnOffset = 210.0750f - 180.0f  ;//102
	chassis.motors[1].TurnOffset = 142;//UNUSED
	chassis.motors[2].TurnOffset = 170;//UNUSED
	chassis.motors[3].TurnOffset = 15.1391f+300.0f-60.0f;


	// ?¡À????????????
	Slope_Init(&chassis.move.xSlope, 80, 0);
	Slope_Init(&chassis.move.ySlope, 80, 0);
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);
	// 4005¡Á????	???
	chassis.motors[0].turn_speed_limit = 12000; // 0-15000;
	chassis.motors[1].turn_speed_limit = 12000; // ???¡ã??¡¤?????0?????????¨´
	chassis.motors[2].turn_speed_limit = 12000;
	chassis.motors[3].turn_speed_limit = 12000;

	Chassis_InitPID();
//	PowerControl_AutoUpdateParamInit();

	Chassis_RegisterEvents();
}

/************?????¡è??????*****************/
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

// ¡Á??¨¢?¨´??????
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); // WASD??????????z?
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	// WASD????????????z?
	RC_Register(Key_Q | Key_E | Key_G, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchMode_KeyCallback);	// QER???????g?
	RC_Register(Key_V, CombineKey_None, KeyEvent_OnDown, Chassis_capOutputChange_KeyCallback);
	RC_Register(Key_E | Key_R, CombineKey_Ctrl, KeyEvent_OnDown, Chassis_Return_KeyCallback);
	//B ?????????¨¹??
	RC_Register(Key_B, CombineKey_None, KeyEvent_OnDown, Chassis_ChangeDiagonal);
	// SHIFT+B
	RC_Register(Key_B, CombineKey_Shift, KeyEvent_OnDown, Chassis_Change_DebugTurn0);
//  RC_Register(Key_V,CombineKey_Shift,KeyEvent_OnDown,Chassis_capBurstChange_KeyCallback);
//	RC_Register(Key_Shift,CombineKey_None,KeyEvent_OnDown,Cap_On_KeyCallback);
	RC_Register(Key_C,CombineKey_None,KeyEvent_OnDown,UI_UPdate);
	RC_Register(Key_C,CombineKey_None,KeyEvent_OnUp,UI_UPdate);

}

// ?¨¹????????¡Á??????¡ã????????????????
void Spin_SpeedUpdate() // TODO??????????????????
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

// ?¨¹??????????
void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
	Slope_NextVal(&chassis.move.spinSlope);
	Slope_SetTarget(&chassis.move.ySlope, chassis.move.maxVy * (chassis.key.key_w + chassis.key.key_s)); // ?????????
	Slope_SetTarget(&chassis.move.xSlope, chassis.move.maxVx * (chassis.key.key_d + chassis.key.key_a)); // ?????????
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
// ?????????¡Á??
void Chassis_RockerCtrl()
{
	// ¡Á¨®??????????¡¤????????????????????????¨²??????
	if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2 && rcInfo.right != 1)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown); // g?????????????
	else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown);
	// ?¨¨?¡§????¡¤??¨°?¡ã????
	Slope_SetTarget(&chassis.move.xSlope, (float)rcInfo.ch3 * chassis.move.maxVx / 660); // ?????????
	Slope_SetTarget(&chassis.move.ySlope, (float)rcInfo.ch4 * chassis.move.maxVy / 660);
}	
// ?????¡Á??????UI??¡Á?
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
/************????????*******************/
/*************************RC???**************************
?????????????event????
*********************************************************/
// WASD????
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		chassis.key.key_w = 1;
		break;
	case Key_S:
		chassis.key.key_s = -1;
		break;
	case Key_D:
		chassis.key.key_d = 1;
		break;
	case Key_A:
		chassis.key.key_a = -1;
		break;
	default:
		break;
	}
}
void UI_UPdate(KeyType key, KeyCombineType combine, KeyEventType event){
	if(event == KeyEvent_OnDown)
			UIupdateState=1;
	else
			UIupdateState=0;
}

// ????????¡ã??¨¹???¡Â
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
	if (chassis.move.fastMode == 0 && cap.receive_data.max_power > 80)
	{
		chassis.move.fastMode = 1;
		chassis.move.cap_output = 1;
		return;
	}
	if (chassis.move.fastMode == 1)
		chassis.move.fastMode = 0;
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
void Chassis_ChangeDiagonal(KeyType key, KeyCombineType combine, KeyEventType event){
	if(key == Key_B)
	{
		if(diagonal_enable!=0)
		{
			diagonal_enable=0;
			chassis.rotate.mode = ChassisMode_Follow;
		}
		else
			diagonal_enable=1;
	}
	chassis.rotate.InitAngle = INIT_YAW_ANGLE - diagonal_enable * 8192-16384;	
	if(chassis.rotate.InitAngle>=65536)
		chassis.rotate.InitAngle-=65536;
	if(chassis.rotate.InitAngle<0)
		chassis.rotate.InitAngle+=65536;}

void Chassis_Change_DebugTurn0(KeyType key, KeyCombineType combine, KeyEventType event){
	if(chassis.motors[0].TurnOffset == 230.6964f)
		chassis.motors[0].TurnOffset = 230.6964f + 38;
	else chassis.motors[0].TurnOffset=230.6964f;
}

void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (chassis.rotate.mode != ChassisMode_Follow || diagonal_enable==0) //  Q/E/R???g? ????????????g?
	{
		chassis.rotate.mode = ChassisMode_Follow;
	}
	else if(diagonal_enable!=0)
	{
		switch (key)
		{
		case Key_Q: // ??????????
			PID_Clear(&chassis.rotate.pid);
			chassis.rotate.mode = ChassisMode_Spin;
			Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw);
			break;
		case Key_E: // ?¡¤?¨ª¡¤???
			if (chassis.rotate.pid.maxOutput == 0)
			{
				chassis.rotate.pid.maxOutput = 11;
			}
			else
			{
				chassis.rotate.pid.maxOutput = 0;
			}
			break;
		default:
			break;
		}
	}
}
void Chassis_Return_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_E:
		gimbal.yaw.targetAngle += 180;
		break;
	case Key_R:
		gimbal.yaw.targetAngle -= 180;
	default:
		break;
	}
}

/************************freertos????**********************
??????????freertos??¡Á¡Â?????¡Â??
**********************************************************/
uint8_t spin_switch = 0;

// ?¡Á?????????¡Â????
void Task_Chassis_Callback()
{
	chassis.rotate.nowAngle = gimbal.yawMotor_M4005.angle;
	chassis.rotate.relativeAngle = ((chassis.rotate.nowAngle - chassis.rotate.InitAngle));
    if (rcInfo.wheel > 600) // ?????¡Â????????????????/?¨¹??????
        chassis.rockerCtrl = true;
    else if (rcInfo.wheel < -600)
        chassis.rockerCtrl = false;

    if (chassis.rockerCtrl) // ????????
        Chassis_RockerCtrl();

    last_left_state = rcInfo.left;

    /***?????¡Á??????????****/
    Chassis_UpdateSlope(); // ?¨¹???????¡À??????????

    // ?????¡§¡Á?¡À¨º???????????????????¡Á??????????(?¨´?????¡§??????)

    float gimbalAngleSin = arm_sin_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);
    float gimbalAngleCos = arm_cos_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);

    chassis.move.vx = (Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);

    chassis.move.vy = (-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
    // ?¨²??
    if (chassis.rotate.mode == ChassisMode_Follow) //????g?
    {
        Slope_SetTarget(&chassis.move.spinSlope, 0);
        if (chassis.rotate.relativeAngle > 180)
            chassis.rotate.relativeAngle -= 360;
        if (chassis.rotate.relativeAngle < -180)
            chassis.rotate.relativeAngle += 360;
        if (YawLost == 0)
        {
            PID_SingleCalc(&chassis.rotate.pid, 0, chassis.rotate.relativeAngle);
        }
        else
        {
            chassis.rotate.relativeAngle = 0;
            chassis.rotate.pid.output = 0;
        }
        chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;

        LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
    }
    else if (chassis.rotate.mode == ChassisMode_Spin) //?????g?
    {
        chassis.move.vw = -chassis.move.spinSlope.value;

        float ratio;
        if (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
            ratio = 0.6;
        else
            ratio = 1.0f;
        if (chassis.rockerCtrl)
        {
            if (rcInfo.left == 2 && rcInfo.wheel > 600 && flagRC == 1)
            {
                if (spin_switch == 0)
                    Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
                else if (spin_switch == 1)
                    Slope_SetTarget(&chassis.move.spinSlope, -chassis.move.maxVw * ratio);
                else
                    Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);

                spin_switch++;
                spin_switch = spin_switch % 2;
            }
        }
    }
}

float Foot_Target_Speed;
float Foot_Relative_Angle;
float Foot_Target_Relative_Angle;

void Foot_CallBack(void)
{
	Foot_Relative_Angle = (gimbal.yawMotor_M4005.angle)/1000.0f;
	if (rcInfo.wheel > 600) 
        chassis.rockerCtrl = true;
    else if (rcInfo.wheel < -600)
        chassis.rockerCtrl = false;

	if (chassis.rockerCtrl)
	{
		arm_atan2_f32(rcInfo.ch4, rcInfo.ch3, &Foot_Target_Relative_Angle);
		Foot_Target_Relative_Angle -= PI/2.0f;
		if(Foot_Target_Relative_Angle < -PI)
			Foot_Target_Relative_Angle += 2.0f*PI;
		if(Foot_Target_Relative_Angle <= -PI/2.0f)
			Foot_Target_Relative_Angle += PI;
		if(Foot_Target_Relative_Angle >= PI/2.0f)
			Foot_Target_Relative_Angle -= PI;

		int16_t temp = rcInfo.ch3;

			temp = 0;

		if(rcInfo.ch4 >= 0)
		Foot_Target_Speed = sqrtf((float)((temp * 3.0 / 660.0f) * (temp * 3.0 / 660.0f) + (rcInfo.ch4 * 3.0 / 660.0f) * (rcInfo.ch4 * 3.0 / 660.0f)));
		if(rcInfo.ch4 < 0)
		Foot_Target_Speed = -sqrtf((float)((temp * 3.0 / 660.0f) * (temp * 3.0 / 660.0f) + (rcInfo.ch4 * 3.0 / 660.0f) * (rcInfo.ch4 * 3.0 / 660.0f)));
		if(Foot_Target_Speed >= 2.7f)
			Foot_Target_Speed =  2.7f;
		if(Foot_Target_Speed <= -2.7f)
			Foot_Target_Speed = -2.7f;
		// if(Foot_Target_Speed == 0)
		// {
			Foot_Target_Relative_Angle = 0;
		// }
	}
	
}	

/************************freertos????**********************
??????????freertos??¡Á¡Â?????¡Â??
**********************************************************/
// ?¡Á?????????¡Â????
#ifdef EN_CHASSIS_TASK
void OS_ChassisCallback(void const *argument)
{
	osDelay(500);
	for (;;)
	{
		// Task_Chassis_Callback();
		Foot_CallBack();
		osDelay(2);
	}
}

#endif
