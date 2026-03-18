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



__attribute__((__packed__))typedef struct
{
//new version
  uint8_t header ;//0xA5
  uint8_t control; // 自瞄是否控制云台 0 不控制 1 控制
  uint8_t fire;    // 是否开火 0 不开火 1 开火
  float yaw;       // 云台py角、速度、加速度(弧度制,直接发,不要乘1000)
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint32_t bullet_id; // 自增的子弹ID
  uint16_t checksum ;
} VisionReceive;

__attribute__((__packed__)) typedef struct
{
    //new version
  uint8_t header;//0x5A
  uint8_t task_mode;   // 当前自瞄模式 0 空闲 1 打装甲板 2 小符 3 大符
  uint8_t enemy_color; // 敌人颜色 0 红色 1 蓝色
  float bullet_speed;  // 弹速
  float roll;          // 云台的外旋rpy角和角速度(弧度制，直接发，不要乘1000)
  float pitch;
  float pitch_vel;
  float yaw;
  float yaw_vel;
  uint32_t bullet_id; // 打出子弹时刻返回的子弹ID（目前没用上）
  uint16_t checksum ;
}VisionTransmit;

typedef __attribute__((__packed__)) struct vision_sensor_info_struct {
    float yaw;
    float pitch;
    uint8_t fire;
    uint8_t found;
} VisionSensorInfo;

typedef struct vision_sensor_struct {
    VisionReceive        *info;
    VisionTransmit        *transmit_info;
    VisionSensorInfo    *sent_info;
    void                (*Init)(void);
    void                (*Update)(void);
    void                (*DataReceive)(uint8_t *read_from_usart, uint32_t length);
    void                (*Data_Transmit)(void);
} VisionSensor;


typedef struct
{    
    //new version
  uint8_t control; // 自瞄是否控制云台 0 不控制 1 控制
  uint8_t fire;    // 是否开火 0 不开火 1 开火
  float yaw;       // 云台py角、速度、加速度(弧度制,直接发,不要乘1000)
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint32_t bullet_id; // 自增的子弹ID
  uint16_t checksum ;
    uint8_t mode;
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

