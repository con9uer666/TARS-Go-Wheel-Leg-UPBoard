#include "Chassis.h"
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

/********************³õÊ¼»¯************************/
void Chassis_Init()
{
	//¶Ô½Ç¶æÊ¹ÄÜ
	diagonal_enable = 1;
	chassis.rockerCtrl = false;
	// µ×ÅÌ³ß´çÐÅÏ¢£¨ÓÃÓÚ½âËãÂÖËÙ£©
	chassis.info.wheelbase = 316.57f;
	chassis.info.wheeltrack = 322.70f;
	chassis.info.wheelRadius = 60;
	chassis.info.offsetX = 0; 
	chassis.info.offsetY = 0; 
	// ÒÆ¶¯²ÎÊý³õÊ¼»¯
	chassis.move.cap_output = 0;
	// Ðý×ª²ÎÊý³õÊ¼»¯
	chassis.rotate.InitAngle = INIT_YAW_ANGLE - diagonal_enable * 8192 - 16384;	
	if(chassis.rotate.InitAngle>=65536)
		chassis.rotate.InitAngle-=65536;
	if(chassis.rotate.InitAngle<0)
		chassis.rotate.InitAngle+=65536;
	
	chassis.rotate.InitpitchAngle = 1290;
	// Æ½ÐÐÊ±yawÖáµç»ú½Ç¶È£¨0-360£©
	chassis.motors[0].TurnOffset = 210.0750f - 180.0f  ;//102
	chassis.motors[1].TurnOffset = 142;//UNUSED
	chassis.motors[2].TurnOffset = 170;//UNUSED
	chassis.motors[3].TurnOffset = 15.1391f+300.0f-60.0f;


	// Ð±ÆÂº¯Êý³õÊ¼»¯
	Slope_Init(&chassis.move.xSlope, 80, 0);
	Slope_Init(&chassis.move.ySlope, 80, 0);
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);
	// 4005×ªËÙÏ	ÞÖÆ
	chassis.motors[0].turn_speed_limit = 12000; // 0-15000;
	chassis.motors[1].turn_speed_limit = 12000; // Ä¿Ç°Ö»·¢ËÍÁË0µÄÈ«²¿Ò»Ñù
	chassis.motors[2].turn_speed_limit = 12000;
	chassis.motors[3].turn_speed_limit = 12000;

	Chassis_InitPID();
//	PowerControl_AutoUpdateParamInit();

	Chassis_RegisterEvents();
}

/************ÄÚ²¿¹¤¾ßº¯Êý*****************/
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

// ×¢²áËùÓÐÊÂ¼þ
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); // WASD??????????z?
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	// WASD????????????z?
	RC_Register(Key_Q | Key_E | Key_G, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchMode_KeyCallback);	// QER???????g?
	RC_Register(Key_V, CombineKey_None, KeyEvent_OnDown, Chassis_capOutputChange_KeyCallback);
	RC_Register(Key_E | Key_R, CombineKey_Ctrl, KeyEvent_OnDown, Chassis_Return_KeyCallback);
	//B ¶Ô½ÇÄ£Ê½¸ü»»
	RC_Register(Key_B, CombineKey_None, KeyEvent_OnDown, Chassis_ChangeDiagonal);
	// SHIFT+B
	RC_Register(Key_B, CombineKey_Shift, KeyEvent_OnDown, Chassis_Change_DebugTurn0);
//  RC_Register(Key_V,CombineKey_Shift,KeyEvent_OnDown,Chassis_capBurstChange_KeyCallback);
//	RC_Register(Key_Shift,CombineKey_None,KeyEvent_OnDown,Cap_On_KeyCallback);
	RC_Register(Key_C,CombineKey_None,KeyEvent_OnDown,UI_UPdate);
	RC_Register(Key_C,CombineKey_None,KeyEvent_OnUp,UI_UPdate);

}

// ¸üÐÂÐ¡ÍÓÂÝ×ªËÙÒÔ¼°Ð¡ÍÓÂÝÏÂÒÆ¶¯ËÙ¶È
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

// ¸üÐÂ¼ÆËãËÙ¶È
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
// Ò¡¸Ë¿ØÖÆµ×ÅÌ
void Chassis_RockerCtrl()
{
	// ×ó²¦¸Ë²¦µ½ÏÂ·½½øÈëÐ¡ÍÓÂÝ£¬²¦»ØÀ´½øÈë¸úËæÄ£Ê½
	if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2 && rcInfo.right != 1)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown); // g?????????????
	else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
		Chassis_SwitchMode_KeyCallback(Key_Q, CombineKey_Ctrl, KeyEvent_OnDown);
	  if (rcInfo.right == 3 && rcInfo.left == 1 && last_left_state == 3)
  {
      Chassis_ChangeDiagonal(Key_B, CombineKey_None, KeyEvent_OnDown);
  }
	// Éè¶¨ÒÆ¶¯·½Ïò¼°ËÙ¶È
	Slope_SetTarget(&chassis.move.xSlope, (float)rcInfo.ch3 * chassis.move.maxVx / 660); // ?????????
	Slope_SetTarget(&chassis.move.ySlope, (float)rcInfo.ch4 * chassis.move.maxVy / 660);
}	
// »ñÈ¡µ×ÅÌÄ£Ê½UIÎÄ×Ö
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
// WASDÒÆ¶¯
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

// Í£Ö¹ÒÆ¶¯°´¼ü»Øµ÷
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

// ÇÐ»»³¬µç
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
// ÇÐ»»³¬µç
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
		case Key_Q: // Ð¡ÍÓÂÝÄ£Ê½
			PID_Clear(&chassis.rotate.pid);
			chassis.rotate.mode = ChassisMode_Spin;
			Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw);
			break;
		case Key_E: // Í·Éí·ÖÀë
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

/************************freertosÈÎÎñ**********************
ÒÔÏÂÈÎÎñÊÜfreertos²Ù×÷ÏµÍ³µ÷¶È
**********************************************************/
int32_t wheelRPM[4];
float targetangle[4];
uint8_t x[4];
float wheelvx[4];
int32_t wheelvx_new[4];
float wheelvy[4];
uint8_t spin_flag[4];
uint8_t spin_switch = 0;

// µ×ÅÌÈÎÎñ»Øµ÷º¯Êý
void Task_Chassis_Callback()
{
    if (YawLost == 0)
    {
        chassis.rotate.nowAngle = gimbal.yawMotor_M4005.angle;
        chassis.rotate.relativeAngle = MOTOR_M4005_CODE2DGR((chassis.rotate.nowAngle - chassis.rotate.InitAngle));
    }
    else
    {
        chassis.rotate.relativeAngle = 0;
    }

    if (rcInfo.wheel > 600) // Ò£¿ØÆ÷²¦ÂÖÇÐ»»Ò¡¸Ë¿ØÖÆ/¼üÅÌ¿ØÖÆ
        chassis.rockerCtrl = true;
    else if (rcInfo.wheel < -600)
        chassis.rockerCtrl = false;

    if (chassis.rockerCtrl) // Ò¡¸Ë¿ØÖÆ
        Chassis_RockerCtrl();

    last_left_state = rcInfo.left;

    /***¼ÆËãµ×ÅÌÆ½ÒÆËÙ¶È****/
    Chassis_UpdateSlope(); // ¸üÐÂÔË¶¯Ð±ÆÂº¯ÊýÊý¾Ý

    // ½«ÔÆÌ¨×ø±êÏµÏÂÆ½ÒÆËÙ¶È½âËãµ½µ×ÅÌÆ½ÒÆËÙ¶È(¸ù¾ÝÔÆÌ¨Æ«Àë½Ç)

    float gimbalAngleSin = arm_sin_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);
    float gimbalAngleCos = arm_cos_f32(-(chassis.rotate.relativeAngle - diagonal_enable * 45.0f-90.0f) * PI / 180);

    chassis.move.vx = (Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);

    chassis.move.vy = (-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
    // ¸úËæ
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

/************************freertosÈÎÎñ**********************
ÒÔÏÂÈÎÎñÊÜfreertos²Ù×÷ÏµÍ³µ÷¶È
**********************************************************/
// µ×ÅÌÈÎÎñ»Øµ÷º¯Êý
#define TOQUE_CONST 600
fp32 scaled_give_power[4];
fp32 power_scale = 0;
uint16_t User_PowerLimit = 100;
float input_power = 0; // input power from battery (referee system)
uint8_t last_level = 0;
float Chassis_Power;
fp32 lastSetcurrent[4];
float scaleSetcurrent = 0;
uint16_t max_power_limit = 0;
float total_turnPower = 0;
float measurePower = 0;
extern int16_t CapMaxpower; // ³¬µç
fp32 sumPowSpeed = 0;
fp32 sumPowTorque = 0;
fp32 effetive_power = 0;

float powerPredict;
extern float paramVector[3][1];
void Chassis_PowerCtrl() // ÐÂ¹¦ÂÊ¿ØÖÆ ·¢ËÍµçÁ÷Ç°¼Ó
{
	//	uint8_t PowerList_PriPower[11] = {85, 60, 65, 70, 75, 80, 85, 90, 95, 100, 100};
	//	uint8_t PowerList_PriHP[11] = {85, 45, 50, 55, 60, 65, 70, 75, 80, 90, 100};
	float initial_give_power[4]; // initial power from PID calculation

	float initial_total_power = 0;

	fp32 chassis_power_buffer = 0.0f;
	fp32 set_buffer = 32.0f;

	fp32 toque_coefficient = 2.4324e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
//										 	fp32 a = fmaxf(1e-7, paramVector[1][0]); // k1
//										 	fp32 k2 = fmaxf(paramVector[0][0], 1e-7);
//										 	fp32 constant = fmaxf(0.7f, paramVector[2][0]);
	fp32 k2 = 1.41955923e-7;			 // RPM
	fp32 a = 1.62214803e-7;				 // Torque
	fp32 constant = 3.655650055f;		 // constant
	max_power_limit = JUDGE_GetChassisPowerLimit();
	chassis_power_buffer = JUDGE_GetPowerBuffer();
	if (max_power_limit < 15 || max_power_limit > 200)
	{
		max_power_limit = User_PowerLimit;
		set_buffer = 0;
		chassis_power_buffer = 0;
	}
	

	/////////////////////////////////////////////
//	sumPowSpeed = 0;
//	sumPowTorque = 0;
//	effetive_power = 0;
//	

//	powerPredict=0;
//	
//	for (uint8_t i = 0; i < 4; i++)
//	{
//		sumPowSpeed += chassis.motors[i].DriveSpeed * chassis.motors[i].DriveSpeed;
//		sumPowTorque += lastSetcurrent[i] * lastSetcurrent[i];
//		effetive_power += chassis.motors[i].DriveSpeed * lastSetcurrent[i] * toque_coefficient;
//	}
//	powerPredict = paramVector[1][0] * sumPowTorque + paramVector[0][0] * sumPowSpeed + 4 * paramVector[2][0] + effetive_power+total_turnPower;
//	if (effetive_power > 10 && measurePower !=cap.receive_data.bus_power / 100.0f && chassis.move.fastMode == 0&&(cap.receive_data.bus_power / 100.0f)>10)
//	{
//		measurePower = cap.receive_data.bus_power / 100.0f;
//		PowerControl_AutoUpdateParam(sumPowSpeed / 4.0f, sumPowTorque / 4.0f, 1, (measurePower - effetive_power-total_turnPower) / 4.0f);
//	}

	////////////////////////////////////////////
	PID_Init(&chassis.move.buffer_pid, 3, 0, 0, 0, 200); // »º³åÄÜÁ¿pid

	PID_SingleCalc(&chassis.move.buffer_pid, set_buffer, chassis_power_buffer);

	input_power = max_power_limit - chassis.move.buffer_pid.output - total_turnPower; // Input power floating at maximum power
	LIMIT(input_power, 5, max_power_limit + 20);
	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		if (detectList[DeviceID_ChassisMotor1 + i].isLost)
			continue;
		initial_give_power[i] = chassis.motors[i].speedPID.output * toque_coefficient * chassis.motors[i].DriveSpeed +
								k2 * chassis.motors[i].DriveSpeed * chassis.motors[i].DriveSpeed +
								a * chassis.motors[i].speedPID.output * chassis.motors[i].speedPID.output + constant;
		if (initial_give_power < 0) // negative power not included (transitory)
			continue;

		initial_total_power += initial_give_power[i];
	}

	if (chassis.move.fastMode == 1 && (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f))
	{
		if (cap.per_energy > 20 && chassis_power_buffer > 10)
			chassis.move.maxPower = input_power + 80;

		else
			chassis.move.maxPower = input_power;
	}
	else if (chassis.move.fastMode == 0)
	{
		chassis.move.maxPower = input_power;
	}
	else
	{
		chassis.move.maxPower = input_power;
	}
	if (chassis.move.maxPower <= 5)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			chassis.motors[i].speedPID.output = 0;
		}
		return;
	}
	float modelPower=fmaxf(chassis.move.maxPower,max_power_limit);
	SET_WHEELSPEED_MAX = 1000 + ((sqrt(TOQUE_CONST * toque_coefficient * TOQUE_CONST * toque_coefficient - 4 * k2 * constant + k2 * modelPower - 4 * a * k2 * TOQUE_CONST) - toque_coefficient * TOQUE_CONST) / (2.0 * k2));
//	SET_WHEELSPEED_MAX = 7500;
	if (initial_total_power > chassis.move.maxPower) // determine if larger than max power
	{
		float a0 = 0;
		float b0 = 0;
		float c0 = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			if (detectList[DeviceID_ChassisMotor1 + i].isLost)
				continue;
			a0 += a * chassis.motors[i].speedPID.output * chassis.motors[i].speedPID.output;
			b0 += toque_coefficient * chassis.motors[i].speedPID.output * chassis.motors[i].DriveSpeed;
			c0 += k2 * chassis.motors[i].DriveSpeed * chassis.motors[i].DriveSpeed + constant;
		}
		c0 -= (chassis.move.maxPower);
		float delta = b0 * b0 - 4 * a0 * c0;
		scaleSetcurrent = 0;
		if (delta < 0)
			scaleSetcurrent = 0;
		else
		{
			float temp1 = (-b0 + sqrt(delta)) / (2 * a0);
			float temp2 = (-b0 - sqrt(delta)) / (2 * a0);
			if (temp1 >= 0 && temp1 <= 1)
				scaleSetcurrent = temp1;

			if (temp2 >= 0 && temp2 <= 1)
			{
				scaleSetcurrent = fmaxf(scaleSetcurrent, temp2);
			}
		}
		if (scaleSetcurrent >= 0.0f && scaleSetcurrent <= 1.0f)
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				chassis.motors[i].speedPID.output *= scaleSetcurrent;
				LIMIT(chassis.motors[i].speedPID.output, -16000, +16000);
			}
		}
		else
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				chassis.motors[i].speedPID.output *= 0.0001f;
			}
		}
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		if (fabs(chassis.motors[i].speedPID.output) >= 16000)
		{
			chassis.motors[i].speedPID.output = chassis.motors[i].speedPID.output / fabs(chassis.motors[i].speedPID.output) * 16000;
		}
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		LIMIT(chassis.motors[i].speedPID.output, -16000, +16000);
		lastSetcurrent[i] = chassis.motors[i].speedPID.output;
	}
	if (max_power_limit > 50)
		chassis.motors[0].turn_speed_limit = 14000;
	else
		chassis.motors[0].turn_speed_limit = 8000;
}


uint8_t lastChassisOutput = 1;
uint8_t chaasisReady = 0;
#ifdef EN_CHASSIS_TASK
void OS_ChassisCallback(void const *argument)
{
	osDelay(500);
	for (;;)
	{
		Task_Chassis_Callback();
		osDelay(2);
	}
}

#endif
