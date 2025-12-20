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
	FDCAN_FilterTypeDef filter;					   //< ?????? can??????
	filter.IdType = FDCAN_STANDARD_ID;			   //< id?????id
	filter.FilterIndex = 0;						   //< ????????,??id??0-127
	filter.FilterType = FDCAN_FILTER_MASK;		   //< ???????????
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //< ??????????? fifo0
	filter.FilterID1 = 0x00000000;				   //< ????id
	filter.FilterID2 = 0x00000000;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter); //< ?????
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // ??fifo0????????
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1); //< ??can

	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter); //< ?????
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // ??fifo0????????
	HAL_FDCAN_Start(&hfdcan2);													//< ??can

	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter); //< ?????
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // ??fifo0????????
	HAL_FDCAN_Start(&hfdcan3);													//< ??can
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
			//            if(rx_data[0]==0xA4)
			CAN2_Rx0Callback(&rx_header, rx_data);
			//            else if(rx_data[0]==0x9A)
			//                CAN2_state_Callback(&rx_header,rx_data);
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

		
	// 未知信息
	default:
		break;
	}
}

// can2接收结束中断
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
//	uint8_t whichMotor;
	switch (rx_header->Identifier)
	{

//	case 0x141:
//	case 0x143:
//	case 0x142:
//	case 0x144:
//	{
//		whichMotor = rx_header->Identifier - 0x141;
//		Turn_Update(&chassis.motors[whichMotor], rxdata[7] << 8 | rxdata[6], rxdata[5] << 8 | rxdata[4], rxdata[3] << 8 | rxdata[2], rxdata[1]);
//	}
//	break;
	// 未知信息
	default:
		break;
	}
}

void CAN2_state_Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
//	uint8_t whichMotor;
	switch (rx_header->Identifier)
	{
//	case 0x141:
//	case 0x142:
//	case 0x143:
//	case 0x144:
//		whichMotor = rx_header->Identifier - 0x141;
//		record_state(&turn_motor_state[whichMotor], rxdata[1], rxdata[7]);
//		break;
	default:
		break;
	}
}

uint8_t open = 0;
uint8_t close = 0;
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{

	switch (rx_header->Identifier)
	{
		case 0x141:
			LKMotor_Update(&gimbal.pitchMotor.M4005,rxdata);
			Detect_Update(DeviceID_PitchMotor);
			if(rxdata[0] == 0x88){
				open =1;
				close = 0;
			}
			if(rxdata[0] == 0x80){
				close = 1;
				open = 0;
			}
		break;
		
		case 0x202:
			Motor_Update(&shooter.fricMotor[0], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
			Detect_Update(DeviceID_FricMotor1);
		break;
	
		case 0x201:
			Motor_Update(&shooter.fricMotor[1], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
			Detect_Update(DeviceID_FricMotor2);
		break;

//	case 0x203:
//		Motor_Update(&shooter.triggerMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
//					 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
//		break;
		
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

void start_lk_motor(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{
	
	uint8_t tx_data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};

	USER_CAN_Send(hfdcan,StdId,tx_data);
	
}

void read_lk_state2(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{
	
	uint8_t tx_data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};

	USER_CAN_Send(hfdcan,StdId,tx_data);
	
}

void clear_error_state(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{

	uint8_t tx_data[8] = {0x9B, 0, 0, 0, 0, 0, 0, 0};

	USER_CAN_Send(hfdcan,StdId,tx_data);

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

void lk_motor_init(FDCAN_HandleTypeDef *hfdcan, int16_t StdId)
{
	uint8_t tx_data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
	USER_CAN_Send(hfdcan,StdId,tx_data);
	uint8_t cnt = 0;
	osDelay(1);
	
	tx_data[0] = 0x80;
	while(!close){
	USER_CAN_Send(hfdcan,StdId,tx_data);
	cnt++;
	if(cnt == 50){
		cnt = 0;
		break;
	}
	osDelay(1);
	}
	
	tx_data[0] = 0x88;
	while(!open){
	USER_CAN_Send(hfdcan,StdId,tx_data);
		cnt++;
	if(cnt == 50){
		cnt = 0;
		break;
	}
	osDelay(1);
	}
	clear_error_state(hfdcan,StdId);
	osDelay(1);
	read_lk_state2(hfdcan,StdId);
	osDelay(1);
}

