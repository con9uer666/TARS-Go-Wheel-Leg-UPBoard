#include "vision.h"
#include "Gimbal.h"
#include "chassis.h"
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
uint8_t detect_color = 0;// 打红0 打蓝1


float UI_x, UI_y;

extern ext_shoot_data_t ShootData;
extern ext_game_robot_status_t GameRobotStat;
extern Gimbal gimbal;
extern INS_t INS;
extern ext_game_robot_HP_t GameRobotHP;
uint8_t Vision_Mode = 0; // 0为空闲，1为装甲板，2为小符，3为大符
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
//下位机火控
uint8_t cmd_fire = 0;

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
	////vision_transmit.reset_tracker = 0;
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
		Vision_Mode = (Vision_Mode+1) % 4;////////////////////////////////////////
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
	vision_transmit.task_mode = Vision_Mode;
	vision_transmit.enemy_color = JUDGE_GetEnemyColor(); // 打红0 打蓝1
	vision_transmit.bullet_speed = ShootData.initial_speed;
	vision_transmit.roll = INS.Roll/180.0f*PI;
	vision_transmit.pitch = INS.Pitch/180.0f*PI;
	vision_transmit.pitch_vel = INS.Gyro[Y_axis];
	vision_transmit.yaw = INS.Yaw/180.0f*PI;
	vision_transmit.yaw_vel = INS.Gyro[Z_axis];

	if(JUDGE_GetBullet_type ())//判断是否发射弹丸
	{
		vision_transmit.bullet_id = vision.bullet_id;//如果发射弹丸返回弹丸ID
	}
	else 
	{
		vision_transmit.bullet_id = 0;//如果没发射弹丸返回0
	}

	Append_CRC16_Check_Sum((uint8_t *)&vision_transmit, sizeof(vision_transmit));
}

void Vision_DataTransmit(void)
{
	Vision_DataUpdate();
	
	CDC_Transmit_HS((uint8_t *)&vision_transmit, sizeof(vision_transmit));
	if(Change_exposure != 0)
		Change_exposure = 0;
}

float test1 = 0.03f;
float test2 = 0.015f;     //2.3m 0.01/1.4m 0.02
float test3 = 0.01f;

 // 将接收的数据进行解码
void Vision_ParseData(void)
{
	
	//new version
	vision.control = vision_receive.control;
  vision.fire_thres_yaw = vision_receive.fire_thres_yaw; // 火控阈值
  vision.fire_thres_pitch = vision_receive.fire_thres_pitch;
  vision.target_yaw = vision_receive.target_yaw;
  vision.target_pitch = vision_receive.target_pitch;
	vision.yaw = vision_receive.yaw/PI*180.0f;
	vision.yaw_vel = vision_receive.yaw_vel;
	vision.yaw_acc = vision_receive.yaw_acc;
	vision.pitch = vision_receive.pitch/PI*180.0f;
	vision.pitch_vel = vision_receive.pitch_vel;
	vision.pitch_acc = vision_receive.pitch_acc;
	vision.bullet_id = vision_receive.bullet_id;

	if (fire_control_angle > PI)
		fire_control_angle -= 2 * PI;
	if (fire_control_angle < -PI)
		fire_control_angle += 2 * PI;
	// 过零处理

//键鼠开自瞄
	if (rcInfo.mouse.r == 1 &&visionFindAver>=0.5f)
	{
		gimbal.visionEnable = true;
	}
	//下位机火控
if(ABS(gimbal.pitch.angle/180*PI-vision.target_pitch) < vision.fire_thres_pitch && ABS(gimbal.yaw.angle/180*PI-vision.target_yaw)<vision.fire_thres_yaw){
	cmd_fire = 1;
	}
else{
	cmd_fire = 0;
	}
	if (cmd_fire == 1 && shooter.fricOpenFlag == 1 && shooter.workState != TRIGGER_CONTINUE && shooter.workState != TRIGGER_CLICK && gimbal.visionEnable == true)
	{
		//上位机火控允许发弹
		shooter.workState = TRIGGER;
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
