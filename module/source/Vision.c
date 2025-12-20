#include "vision.h"
#include "Gimbal.h"
#include "Judge.h"
#include "RC.h"
#include "crc.h"
#include "UserFreertos.h"
#include "task.h"
#include "ins_task.h"
#include "math.h"
#include "bmi088driver.h"
#include "tim.h"
#include <arm_math.h>
#include <math.h>
#include <stdio.h>
#include <cmsis_os.h>
#include "shooter.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#define EN_VS_TASK

uint8_t Vision_Online;
VisionTransmit vision_transmit;
VisionReceive vision_receive;
Vision_Type vision;
uint8_t Rune_direction;
uint8_t Rune_stable = 0;
uint8_t Change_exposure;

float UI_x, UI_y;

extern ext_shoot_data_t ShootData;
extern ext_game_robot_status_t GameRobotStat;
extern Gimbal gimbal;
extern INS_t INS;
extern ext_game_robot_HP_t GameRobotHP;
uint8_t Vision_Mode = 0; // 0为自瞄，1为小符，2为大符
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_RegisterEvents(void);
void Vision_Reset_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_RuneDir_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_Expo_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

// 自瞄调试变量

char rotateflag = 0; // 正反标识符

float distance_weight = 0.0; // 距离权重系数
float X1 = 0.5;				 // 边界值1 由小变大
float X2 = 1.5;				 // 边界值2
float X3 = 2.5;				 // 边界值3
float X4 = 3.5;				 // 边界值4

float fire_control_angle = 0.0;
float gimbal_yaw_diff = 0.0;
float gimbal_pitch_diff = 0.0;
// 自瞄调试变量

VisionSensorInfo vision_sensor_info = {
	.yaw = 0,
	.pitch = 0,
	.found = 0,
	.fire = 0,
};

VisionSensor vision_sensor = {
	.sent_info = &vision_sensor_info, // 数据结构体
	.Init = Vision_Init,			  // 传感器初始化
	.Update = Vision_DataUpdate,
	.DataReceive = Vision_DataReceive,
	.Data_Transmit = Vision_DataTransmit,
};

void Vision_Init(void)
{
	vision_transmit.header = VISION_FRAME_HEADER_TX;
	vision_transmit.reset_tracker = 0;
	//	Slope_Init(&gimbal.yaw.visionslope,0.7,0.7);
	//	Slope_Init(&gimbal.pitch.visionslope,0.5,0.5);
	Vision_RegisterEvents();
	// HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_RESET);
}

// 注册事件
void Vision_RegisterEvents()
{
	// R键切换视觉模式
	RC_Register(Key_R, CombineKey_None, KeyEvent_OnDown, Vision_Change_KeyCallback);
	RC_Register(Key_X,CombineKey_None,KeyEvent_OnDown,Vision_RuneDir_KeyCallback);
	RC_Register(Key_A,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
	RC_Register(Key_D,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
//	RC_Register(Key_W,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Change_KeyCallback);
}
uint8_t cnt1 = 0;
uint8_t cnt0 = 0;
// 切换视觉模式
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
		Vision_Mode = (Vision_Mode + 1) % 3;
		cnt0++;
//	if(combine == CombineKey_Ctrl && key == Key_W)
//	{
//		Rune_stable = 1;
//	}
}

void Vision_RuneDir_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
		Rune_direction = (Rune_direction + 1) % 2;
}

void Vision_Expo_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (combine == CombineKey_Ctrl && key == Key_A)
	{
		Change_exposure = 1;
	}
	else if(combine == CombineKey_Ctrl && key == Key_D)
	{
		Change_exposure = 2;
	}
}


// 重启NUC
void Vision_Reset_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

// 接收来自视觉的信息
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
	// 若无数据包，则不作任何处理
	// memcpy(rxsssdata,read_from_usart,10);
	if (read_from_usart == NULL)
		return;
	if (length > 50) // FIXME: 保险作用
		return;
	// 查找帧头
	while (length)
	{
		if (*read_from_usart != VISION_FRAME_HEADER_RX)
		{
			++read_from_usart;
			--length;
		}
		else
		{
			break;
		}
	}
	if (length == 0)
		return;
	// 判断帧头数据是否正确
	if (read_from_usart[0] == VISION_FRAME_HEADER_RX)
	{
		// 将数据存入接收buffer
		memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
		Vision_ParseData();
	}
}

// 对发送的数据更新
void Vision_DataUpdate(void)
{
	vision_transmit.header = 0x5A;
	vision_transmit.yaw = INS.Yaw * 1000.0f;
	vision_transmit.pitch = INS.Pitch * 1000.0f;
	vision_transmit.roll = INS.Roll * 1000.0f;
	vision_transmit.detect_color = 1; // 打红0 打蓝1
	vision_transmit.task_mode = Vision_Mode;
	vision_transmit.rune_direction = Rune_direction; //0-anti-clockwise 1-clockwise
	vision_transmit.rune_stable  = Rune_stable;	//0-unstable 1-stable 
	vision_transmit.change_exposure =   Change_exposure ; //0-stay 1-add 2-sub
	
	if ((GameRobotHP.blue_outpost_HP > 0 && vision_transmit.detect_color == 1) || (GameRobotHP.red_outpost_HP > 0 && vision_transmit.detect_color == 0))
		vision_transmit.vision_select = 0;
	else
		vision_transmit.vision_select = 1;

	Append_CRC16_Check_Sum((uint8_t *)&vision_transmit, sizeof(vision_transmit));
}

void Vision_DataTransmit(void)
{
	Vision_DataUpdate();
	CDC_Transmit_HS((uint8_t *)&vision_transmit, sizeof(vision_transmit));
	if(Change_exposure != 0)
		Change_exposure = 0;
}

float test1 = 0.025f;
float test2 = 0.02f; //2.9m 0.01 2.3m
float test3 = 0.01f;

 // 将接收的数据进行解码
void Vision_ParseData(void)
{
	vision.exposure_time = vision_receive.exposure_time;
	vision.pitch = vision_receive.pitch / 1000.0f;
	vision.yaw = vision_receive.yaw / 1000.0f;
	vision.fire = vision_receive.fire/1000.0f;
	vision.found = vision_receive.tracking;
	vision.distance = vision_receive.distance;
	vision.v_yaw = vision_receive.v_yaw;

	fire_control_angle = vision.fire-INS.Yaw/180*PI;
	gimbal_yaw_diff = vision.yaw / 180*PI-INS.Yaw /180*PI;
	gimbal_pitch_diff=vision.pitch/180* PI-INS.Pitch/180*PI;

	if (fire_control_angle > PI)
		fire_control_angle -= 2 * PI;
	if (fire_control_angle < -PI)
		fire_control_angle += 2 * PI;
	// 过零处理

	if ((rcInfo.left == 1||rcInfo.mouse.r == 1) &&visionFindAver>=0.5f)
	{
		gimbal.visionEnable = true;

		char spinFireflag = 0;
//		if (vision_receive.ispingyixuanzhuan == 1 && ABS(fire_control_angle) < test1 )//(0.0019*vision.distance*vision.distance-0.0221*vision.distance+0.0651)
//			spinFireflag = 1;
		if (ABS(fire_control_angle) < test1)
			spinFireflag = 1;
		else if (vision.distance > 6.0f && vision.distance < 8.0f && ABS(fire_control_angle) < 0.005)
			spinFireflag = 1;
		else if (vision.distance > 8.0f || vision.distance < 0.8f || ABS(fire_control_angle) > test1)
			spinFireflag = 0;
//		}
//		if(GameRobotStat.robot_level >=5){
 //				test2 = 1.5f*0.015f;
//		}
		if (ABS(gimbal_yaw_diff) < test2&&ABS(gimbal_pitch_diff) < 0.5*test2) // 跟随 0.015
			gimbal.fire = 1;
		else
			gimbal.fire = 0;
		if (Vision_Mode == 0&&gimbal.fire == 1 && spinFireflag == 1 && shooter.fricOpenFlag == 1 && shooter.workState != TRIGGER_CONTINUE && shooter.workState != TRIGGER_CLICK)
		{
			shooter.workState = TRIGGER;
		}
			//shunshizhen:miaozhongxin 100,gensui140
			//nishizhen:zhongxin 120,gensui
	}
}

#ifdef EN_VS_TASK
void OS_VisionCallback(void const *argument)
{
	vision_sensor.Init();
	for (;;)
	{
		vision_sensor.Data_Transmit();
		osDelay(1);
	}
}
#endif
