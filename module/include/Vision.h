#ifndef _VISION_H_
#define _VISION_H_

#include "stdint.h"

#define    VISION_FRAME_HEADER_TX  	0x5A
#define    VISION_FRAME_HEADER_RX  	0xA5


#define    VISION_INIT_TIME         4000
#define    VISION_CONTROL_TIME      2

#ifndef PI
#define PI 3.1415926535f
#endif

typedef struct
{
	uint8_t header;
	int8_t tracking ;
	uint8_t id : 3;          // 0-outpost 6-guard 7-base
	uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
	uint8_t ispingyixuanzhuan : 1;
	uint8_t reserved : 1;
	uint8_t exposure_time : 6;
	float yaw;
	float pitch;
	float fire;
	float v_yaw;
	float distance;//两车中心距离
//	float distance_to_center;//云台中心到锁定装甲板中心距离
//	float x;
//	float y;
	uint16_t checksum;

}VisionReceive;

typedef struct
{
	uint8_t header;
	uint8_t detect_color : 1;  // 0-red 1-blue
	_Bool reset_tracker : 1;
	uint8_t task_mode : 2;
	_Bool rune_direction : 1;
	_Bool rune_stable : 1;
	uint8_t change_exposure :2;//0-stay 1-add 2-sub
	float roll;
	float pitch;
	float yaw;
	int8_t vision_select;
	uint16_t checksum;
}VisionTransmit;

typedef struct vision_sensor_info_struct {
	float yaw;
	float pitch;
	uint8_t fire;
	uint8_t found;
} VisionSensorInfo;

typedef struct vision_sensor_struct {
	VisionReceive		*info;
	VisionTransmit		*transmit_info;
	VisionSensorInfo    *sent_info;
	void				(*Init)(void);
	void				(*Update)(void);
    void                (*DataReceive)(uint8_t *read_from_usart, uint32_t length);
	void                (*Data_Transmit)(void);
} VisionSensor;


typedef struct
{	
	float yaw;
	float pitch;
	uint8_t mode;
	unsigned char found;
	float fire;
	float v_yaw;
	float distance;//两车中心距离
	float distance_to_center;//云台中心到锁定装甲板中心距离
	float yaw_slope;
	float pitch_slope;
	uint8_t exposure_time;
}Vision_Type;

extern VisionReceive vision_receive;
extern VisionTransmit vision_transmit;
extern VisionSensor vision_sensor;
extern uint8_t Vision_Mode;
extern Vision_Type vision;
extern uint8_t Rune_stable;


void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length);
void Vision_DataTransmit(void);
void Vision_DataUpdate(void);
void Vision_Init(void);
void Vision_ParseData(void);



#endif

