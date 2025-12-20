#ifndef __COM_H
#define __COM_H

#include "stdint.h"
#include "Judge.h"
#include "Moto.h"

#define Send_HEADER     0xA1
#define Receive_HEADER  0x1A
#define _CMD_MOTOR   0x01
#define _CMD_REF    0x02
#define _CMD_UI     0x03

#define COM_MAX_RX_LENGTH 64
#define COM_MAX_TX_LENGTH 64
//过度封装了，为了统一接口先这样


// 电机ID枚举
typedef enum {
    MOTOR_LF_DRIVE,    // 左前驱动   

	MOTOR_LB_DRIVE,    // 左后驱动   右前drive
    MOTOR_LF_TURN,     // 左前转向   右后drive
    MOTOR_LB_TURN,     // 左后转向   

    MOTOR_RF_DRIVE,    // 右前驱动
    MOTOR_RB_DRIVE,    // 右后驱动
    MOTOR_RF_TURN,     // 右前转向
    MOTOR_RB_TURN,     // 右后转向

    MOTOR_YAW,         // YAW
    MOTOR_TRIGGER,     // 拨弹
    MOTOR_NUM          // 电机总数
} MotorID;


// 电机数据结构体
typedef __packed struct {

    uint16_t angle;       // 角度
    int16_t speed;       // 速度
    int16_t torque;      // 扭矩
    uint8_t temp;        // 温度
    //float output;        // 输出
} Motor_Data_t;


// 裁判系统数据结构体
typedef __packed struct {
    // uint16_t robot_id;     // 机器人ID
    // uint16_t hp;          // 血量
    // uint16_t shooter_heat; // 枪口热量
    // uint8_t game_status;   // 比赛状态
    // uint16_t remaining_time; // 剩余时间
    // uint8_t robot_buff;    // 机器人增益

    ext_game_robot_pos_t		GameRobotPos;
    ext_shoot_data_t			ShootData;
    ext_game_robot_status_t		GameRobotStat;
    ext_power_heat_data_t       PowerHeatData;				   // 0x0202

} Referee_Data_t;

// UI数据结构体
// extern xFrameHeader FrameHeader; // 发送帧头信息

// 串口发送数据帧头    0x1A
// 串口接收数据帧头    0xA1

typedef __packed struct {
    uint8_t header;      // 帧头
    uint16_t length;     // 数据长度
    uint8_t cmd;         // 命令字  Motor 0x01 Judge 0x02  UI 0x03
    uint8_t data[];      // 数据段
} COM_Data_t;

// 发送数据结构体
typedef  struct {

    int16_t output[MOTOR_NUM];
    JudgeTxFrame Frame;

} Send_Data_t;

// 接收数据结构体
typedef __packed struct {
    Motor_Data_t motor_info[MOTOR_NUM];
    Referee_Data_t referee;
    int8_t Rxlen;
    
} Receive_Data_t;

// 全局变量声明
extern Send_Data_t send_data;
extern Receive_Data_t receive_data;
extern uint8_t usart1RxBuf[COM_MAX_RX_LENGTH];


void COM_ReceiveData(uint8_t *data);
void COM_SendData(Send_Data_t *data);
void COM_UpdateData(void);
void COM_Init(void);
void RS485_LostCallback(void);
#endif // __COM_H
