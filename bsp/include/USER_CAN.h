#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "stdint.h"
#include "fdcan.h"
#include "cmsis_os.h"
//can的状态信息，用于debug
typedef struct
{
	HAL_StatusTypeDef can1_user_init_error,can2_user_init_error,can3_user_init_error;
	uint16_t can1_send_error,can2_send_error,can3_send_error;
	uint16_t can1_receive_error,can2_receive_error,can3_receive_error;
}CanState;

/****接口函数声明****/
//发送电机电流信息
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef *hcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
//发送翎控电机
void USER_CAN_SetMotorPosition_SingleCircle(FDCAN_HandleTypeDef *hfdcan,int16_t StdId,uint8_t spin,uint16_t speed,uint32_t target);
void USER_CAN_SetMotorPosition(FDCAN_HandleTypeDef *hfdcan,int16_t StdId,uint8_t spin,uint16_t speed, int32_t target);
void USER_CAN_SetMotorSpeed(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int32_t speed);
void USER_CAN_SetMotorPosition_7(FDCAN_HandleTypeDef *hfdcan,int16_t StdId,uint32_t target);
void USER_CAN_SetIncrAngle2(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int16_t speedlimit ,int32_t incrangle);
void USER_CAN_SetMotorTorque(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int16_t iqControl);
void start_lk_motor(FDCAN_HandleTypeDef *hfdcan, int16_t StdId);


//向视觉发送数据
void USER_CAN_SendVisionBuf(FDCAN_HandleTypeDef *hcan,uint8_t *buf,uint8_t len);
void clear_error_state( FDCAN_HandleTypeDef *hfdcan,int16_t StdId)  ;
void USER_CAN_GetCapData(FDCAN_HandleTypeDef *hcan,int16_t StdId);
void USER_CAN_SendCapData(FDCAN_HandleTypeDef *hcan,int16_t StdId,uint16_t set_target_power);
void read_lk_state2(FDCAN_HandleTypeDef *hfdcan, int16_t StdId);
void lk_motor_init(FDCAN_HandleTypeDef *hfdcan, int16_t StdId);

//can初始化
void CAN_Init(void);

#endif
