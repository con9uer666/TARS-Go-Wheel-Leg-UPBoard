/*************自定义CAN通信************/

#include "USER_CAN.h"
#include "fdcan.h"
#include "Moto.h"
#include "chassis.h"
#include "Shooter.h"
#include "Gimbal.h"
#include "Vision.h"
#include "Detect.h"
#include "super_cap.h"
#include "Judge.h"
#include "Rc.h"
uint32_t debugVisionInterval = 0;
CanState can_state;

/**************内部工具函数声明***********************/
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
// can2接收
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
// can3 ??
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
void CAN2_state_Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);

/******************初始化***************************/
// can过滤器初始化
void CAN_Init()
{
	FDCAN_FilterTypeDef filter;					   
	filter.IdType = FDCAN_STANDARD_ID;			   
	filter.FilterIndex = 0;						   
	filter.FilterType = FDCAN_FILTER_MASK;		   
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 
	filter.FilterID1 = 0x00000000;				   
	filter.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1);

	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan2);

	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan3);
}


/*********************can接受回调函数*************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_StatusTypeDef if_can_get_message_ok;
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (hfdcan == &hfdcan1)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if (if_can_get_message_ok == HAL_OK)
		{
			CAN1_Rx0Callback(&rx_header, rx_data);
		}
		else
		{
			can_state.can1_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
	else if (hfdcan == &hfdcan2)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if (HAL_OK == if_can_get_message_ok)
		{
			CAN2_Rx0Callback(&rx_header, rx_data);
		}
		else
		{
			can_state.can2_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
	else if (hfdcan == &hfdcan3)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if (HAL_OK == if_can_get_message_ok)
		{
			CAN3_Rx0Callback(&rx_header, rx_data);
		}
		else
		{
			can_state.can3_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
}
// void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
//{
//     if(hfdcan==&hfdcan1)
//     {
//         if(ErrorStatusITs==FDCAN_IT_BUS_OFF)
//         {
//             HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
//         }
//     }
// }
static void check_can_bus(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus;

	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
	if (protocolStatus.BusOff)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan == &hfdcan1)
	{
		if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
		{

			check_can_bus(hfdcan);
		}
	}
}

// can1接收结束中断
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	switch (rx_header->Identifier)
	{
		default:
			break;
	}
}
// can2接收结束中断
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
	switch (rx_header->Identifier)
	{
		case DM_PITCH_MASTER_ID:
			DMMotor_Update(&gimbal.pitchMotor.DM4310, rxdata);
			Detect_Update(DeviceID_PitchMotor);
			break;
		default:
			break;
	}
}

void CAN2_state_Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
//	uint8_t whichMotor;
	switch (rx_header->Identifier)
	{
		default:
			break;
	}
}


void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{

	switch (rx_header->Identifier)
	{
		
		case 0x202:
			Motor_Update(&shooter.fricMotor[0], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
			Detect_Update(DeviceID_FricMotor1);
			break;
	
		case 0x201:
			Motor_Update(&shooter.fricMotor[1], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
			Detect_Update(DeviceID_FricMotor2);
			break;
		default:
			break;
	}
}

/********************内部函数*******************************/
void USER_CAN_Send(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t tx_data[8])
{
	FDCAN_TxHeaderTypeDef tx_header;
	//	   ++tx_cnt;
	tx_header.Identifier = StdId;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = 8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	vTaskSuspendAll();
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data) != HAL_OK)
	{
		if (hfdcan == &hfdcan1)
		{
			can_state.can1_send_error++;
		}
		else if (hfdcan == &hfdcan2)
		{
			can_state.can2_send_error++;
		}
		else if (hfdcan == &hfdcan3)
		{
			can_state.can3_send_error++;
		}
	}
	xTaskResumeAll();
}
/********************外部调用函数*******************************/

//以下的所有外部发送函数都应该从内部函数封装成接口

// 发送电机信息

void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	uint8_t tx_data[8] = {0};
	tx_data[0] = (iq1 >> 8) & 0xff;
	tx_data[1] = (iq1) & 0xff;
	tx_data[2] = (iq2 >> 8) & 0xff;
	tx_data[3] = (iq2) & 0xff;
	tx_data[4] = (iq3 >> 8) & 0xff;
	tx_data[5] = (iq3) & 0xff;
	tx_data[6] = (iq4 >> 8) & 0xff;
	tx_data[7] = (iq4) & 0xff;

	USER_CAN_Send(hfdcan,StdId,tx_data);
}

void USER_CAN_SetMotorPosition_SingleCircle(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t spin, uint16_t speed, uint32_t target)
{	
	uint8_t tx_data[8] = {0xa6, spin, *(uint8_t *)(&speed), *((uint8_t *)(&speed) + 1), *(uint8_t *)(&target),
					   *((uint8_t *)(&target) + 1), *((uint8_t *)(&target) + 2), *((uint8_t *)(&target) + 3)};
	
	USER_CAN_Send(hfdcan,StdId,tx_data);
		
}

void USER_CAN_SetMotorPosition(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint8_t spin, uint16_t speed, int32_t target)
{

	uint8_t tx_data[8] = {0xa4, spin, *(uint8_t *)(&speed), *((uint8_t *)(&speed) + 1), *(uint8_t *)(&target),
					   *((uint8_t *)(&target) + 1), *((uint8_t *)(&target) + 2), *((uint8_t *)(&target) + 3)};

	USER_CAN_Send(hfdcan,StdId,tx_data);
}

void USER_CAN_SetMotorTorque(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int16_t iqControl)
{

	uint8_t tx_data[8] = {0xa1, 0, 0, 0,
					   *((uint8_t *)(&iqControl)), *((uint8_t *)(&iqControl) + 1), 0 ,0};
	
	USER_CAN_Send(hfdcan,StdId,tx_data);
	
}

void USER_CAN_SetMotorSpeed(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int32_t speed)
{

	uint8_t tx_data[8] = {0xa2, 0, 0, 0, *(uint8_t *)(&speed),
					   *((uint8_t *)(&speed) + 1), *((uint8_t *)(&speed) + 2), *((uint8_t *)(&speed) + 3)};
	
	USER_CAN_Send(hfdcan,StdId,tx_data);
	
}

void USER_CAN_SetIncrAngle2(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, int16_t speedlimit ,int32_t incrangle)
{

	uint8_t tx_data[8] = {0xa7, 0,  *(uint8_t *)(&speedlimit), *((uint8_t *)(&speedlimit) + 1), *(uint8_t *)(&incrangle),
					   *((uint8_t *)(&incrangle) + 1), *((uint8_t *)(&incrangle) + 2), *((uint8_t *)(&incrangle) + 3)};
	
	USER_CAN_Send(hfdcan,StdId,tx_data);
	
}

void USER_CAN_SetMotorPosition_7(FDCAN_HandleTypeDef *hfdcan, int16_t StdId, uint32_t target)
{

	uint8_t tx_data[8] = {0xa3, 00, 0, 0, *(uint8_t *)(&target),
					   *((uint8_t *)(&target) + 1), *((uint8_t *)(&target) + 2), *((uint8_t *)(&target) + 3)};

	USER_CAN_Send(hfdcan,StdId,tx_data);
}
// read state
void read_motorstate(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{

	uint8_t tx_data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};

	USER_CAN_Send(hfdcan,StdId,tx_data);

}

void clear_error_state(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{

	uint8_t tx_data[8] = {0x9B, 0, 0, 0, 0, 0, 0, 0};

	USER_CAN_Send(hfdcan,StdId,tx_data);

}
/********************* 达妙 DM4310 MIT 模式 *********************/

// float -> uint，按 [x_min, x_max] 线性映射到 bits 位无符号整数
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

// MIT 控制帧
void DM_MIT_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id,
                 float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t p_int  = float_to_uint(p_des, -DM4310_PMAX,  DM4310_PMAX, 16);
    uint16_t v_int  = float_to_uint(v_des, -DM4310_VMAX,  DM4310_VMAX, 12);
    uint16_t kp_int = float_to_uint(kp,     0.0f,         500.0f,      12);
    uint16_t kd_int = float_to_uint(kd,     0.0f,         5.0f,        12);
    uint16_t t_int  = float_to_uint(t_ff,  -DM4310_TMAX,  DM4310_TMAX, 12);

    uint8_t tx_data[8];
    tx_data[0] = (p_int  >> 8) & 0xFF;
    tx_data[1] =  p_int        & 0xFF;
    tx_data[2] = (v_int  >> 4) & 0xFF;
    tx_data[3] = ((v_int  & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
    tx_data[4] =  kp_int       & 0xFF;
    tx_data[5] = (kd_int >> 4) & 0xFF;
    tx_data[6] = ((kd_int & 0x0F) << 4) | ((t_int  >> 8) & 0x0F);
    tx_data[7] =  t_int        & 0xFF;

    USER_CAN_Send(hfdcan, can_id, tx_data);
}

// 特殊控制帧：D[0..6]=0xFF，D[7] 决定操作
static void DM_SendSpecial(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t cmd)
{
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    USER_CAN_Send(hfdcan, can_id, tx_data);
}

void DM_MotorEnable(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
    DM_SendSpecial(hfdcan, can_id, 0xFC);
}

void DM_MotorDisable(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
    DM_SendSpecial(hfdcan, can_id, 0xFD);
}

void DM_MotorClearError(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
    DM_SendSpecial(hfdcan, can_id, 0xFB);
}

void DM_MotorSaveZero(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
    DM_SendSpecial(hfdcan, can_id, 0xFE);
}

//TODO! 需要修改
void Cap_CanSendData()
{
//	FDCAN_TxHeaderTypeDef tx_header;
//	uint8_t tx_data[8] = {0};

//	tx_header.Identifier = 0x302;
//	tx_header.IdType = FDCAN_STANDARD_ID;
//	tx_header.TxFrameType = FDCAN_DATA_FRAME;

//	tx_data[0] = (uint16_t)(cap.target_charge_power * 100);
//	tx_data[1] = (uint16_t)(cap.target_charge_power * 100) >> 8;
//	tx_data[2] = (uint16_t)(cap.target_output_power * 100);
//	tx_data[3] = (uint16_t)(cap.target_output_power * 100) >> 8;
//	tx_data[4] = cap.power_ctrl_mode;

//	tx_data[0] = (uint16_t)(JUDGE_GetPowerBuffer());
//	tx_data[1] = (uint16_t)(JUDGE_GetPowerBuffer()) >> 8;
//	tx_data[2] = (uint16_t)(GameRobotStat.chassis_power_limit);
//	tx_data[3] = (uint16_t)(GameRobotStat.chassis_power_limit) >> 8;
//	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}

