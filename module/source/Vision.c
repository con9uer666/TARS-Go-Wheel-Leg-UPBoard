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
uint8_t detect_color = 0;


float UI_x, UI_y;

extern ext_shoot_data_t ShootData;
extern ext_game_robot_status_t GameRobotStat;
extern Gimbal gimbal;
extern INS_t INS;
extern ext_game_robot_HP_t GameRobotHP;
uint8_t Vision_Mode = 0; // 0Ϊ���飬1ΪС����2Ϊ���
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_RegisterEvents(void);
void Vision_Reset_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_RuneDir_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Vision_Expo_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

// ������Ա���

char rotateflag = 0; // ������ʶ��

float distance_weight = 0.0; // ����Ȩ��ϵ��
float X1 = 0.5;				 // �߽�ֵ1 ��С���
float X2 = 1.5;				 // �߽�ֵ2
float X3 = 2.5;				 // �߽�ֵ3
float X4 = 3.5;				 // �߽�ֵ4

float fire_control_angle = 0.0;
float gimbal_yaw_diff = 0.0;
float gimbal_pitch_diff = 0.0;
// ������Ա���

VisionSensorInfo vision_sensor_info = {
	.yaw = 0,
	.pitch = 0,
	.found = 0,
	.fire = 0,
};

VisionSensor vision_sensor = {
	.sent_info = &vision_sensor_info, // ���ݽṹ��
	.Init = Vision_Init,			  // ��������ʼ��
	.Update = Vision_DataUpdate,
	.DataReceive = Vision_DataReceive,
	.Data_Transmit = Vision_DataTransmit,
};

void Vision_Init(void)
{
	vision_transmit.header = VISION_FRAME_HEADER_TX;
	//vision_transmit.reset_tracker = 0;
	//	Slope_Init(&gimbal.yaw.visionslope,0.7,0.7);
	//	Slope_Init(&gimbal.pitch.visionslope,0.5,0.5);
	Vision_RegisterEvents();
	// HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_RESET);
}

// ע���¼�
void Vision_RegisterEvents()
{
	// R���л��Ӿ�ģʽ
	// RC_Register(Key_R, CombineKey_None, KeyEvent_OnDown, Vision_Change_KeyCallback);
	// RC_Register(Key_X,CombineKey_None,KeyEvent_OnDown,Vision_RuneDir_KeyCallback);
	// RC_Register(Key_A,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
	// RC_Register(Key_D,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
//	RC_Register(Key_W,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Change_KeyCallback);
}
uint8_t cnt1 = 0;
//uint8_t cnt0 = 0;
// �л��Ӿ�ģʽ
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
		Vision_Mode = (Vision_Mode + 1) % 3;
	//	cnt0++;
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


// ����NUC
void Vision_Reset_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

// ���������Ӿ�����Ϣ
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
	// �������ݰ��������κδ���
	// memcpy(rxsssdata,read_from_usart,10);
	if (read_from_usart == NULL)
		return;
	if (length > 50) // FIXME: ��������
		return;
	// ����֡ͷ
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
	// �ж�֡ͷ�����Ƿ���ȷ
	if (read_from_usart[0] == VISION_FRAME_HEADER_RX)
	{
		// �����ݴ������buffer
		memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
		Vision_ParseData();
	}
}

// �Է��͵����ݸ���

void Vision_DataUpdate(void)
{
    vision_transmit.header = 0x5A;
    vision_transmit.task_mode = Vision_Mode;
    vision_transmit.enemy_color = detect_color; // ���0 ����1
    vision_transmit.bullet_speed = 50;  //ShootData.initial_speed;
    vision_transmit.roll = INS.Roll/180*PI;
    vision_transmit.pitch = INS.Pitch/180*PI;
    vision_transmit.pitch_vel = INS.Gyro[Y_axis];
    vision_transmit.yaw = INS.Yaw/180*PI;
    vision_transmit.yaw_vel = INS.Gyro[Z_axis];
		//vision_transmit.bullet_id = 0;
    
//    if ((GameRobotHP.blue_outpost_HP > 0 && vision_transmit.detect_color == 1) || (GameRobotHP.red_outpost_HP > 0 && vision_transmit.detect_color == 0))
//        vision_transmit.vision_select = 0;
//    else
//        vision_transmit.vision_select = 1;

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

 // �����յ����ݽ��н���
void Vision_ParseData(void)
{
//new version
    vision.control = vision_receive.control;
    vision.fire = vision_receive.fire;
    vision.yaw = vision_receive.yaw/PI*180;
    vision.yaw_vel = vision_receive.yaw_vel/PI*180;
    vision.yaw_acc = vision_receive.yaw_acc/PI*180;
    vision.pitch = vision_receive.pitch/PI*180;
    vision.pitch_vel = vision.pitch_vel/PI*180;
    vision.pitch_acc = vision_receive.pitch_acc/PI*180;
    vision.bullet_id = vision_receive.bullet_id;

	if (fire_control_angle > PI)
		fire_control_angle -= 2 * PI;
	if (fire_control_angle < -PI)
		fire_control_angle += 2 * PI;
	// ���㴦��

	if ((rcInfo.left == 1||rcInfo.mouse.r == 1) &&visionFindAver>=0.5f)
	{
		gimbal.visionEnable = true;

//		char spinFireflag = 0;
//		if (vision_receive.ispingyixuanzhuan == 1 && ABS(fire_control_angle) < test1 )//(0.0019*vision.distance*vision.distance-0.0221*vision.distance+0.0651)
//			spinFireflag = 1;
//		if (ABS(fire_control_angle) < test1)
//			spinFireflag = 1;
//		else if (vision.distance > 6.0f && vision.distance < 8.0f && ABS(fire_control_angle) < 0.005)
//			spinFireflag = 1;
//		else if (vision.distance > 8.0f || vision.distance < 0.8f || ABS(fire_control_angle) > test1)
//			spinFireflag = 0;
//		}
//		if(GameRobotStat.robot_level >=5){
 //				test2 = 1.5f*0.015f;
//		}
		if (ABS(gimbal_yaw_diff) < test2&&ABS(gimbal_pitch_diff) < 0.5*test2) // ���� 0.015
			gimbal.fire = 1;
		else
			gimbal.fire = 0;
		if (Vision_Mode == 0&&gimbal.fire == 1 && shooter.fricOpenFlag == 1 && shooter.workState != TRIGGER_CONTINUE && shooter.workState != TRIGGER_CLICK)
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
