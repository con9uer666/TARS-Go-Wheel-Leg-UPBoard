#include "User_TOF.h"
#include "usart.h"
#include "detect.h"

uint8_t tof_rx_buff[MAX_TOF_BUFFER];
//Tof初始化
void Tof_Init()
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
//串口1中断回调
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart)
{
	static uint8_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
	{
		__HAL_UART_CLEAR_NEFLAG(huart);
		tof_rx_buff[rxCnt++] = huart->Instance->DR;
	}
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    g_nts_frame0.UnpackData(tof_rx_buff,16);
		rxCnt = 0;
    
    Detect_Update(DeviceID_TOF);
  }
}

//距离
float Tof_GetDistance()
{
  return g_nts_frame0.result.dis;
}
//信号强度
uint16_t Tof_GetSignalStrength()
{
  return g_nts_frame0.result.signal_strength;
}
//数据是否正确
bool Tof_IsRight()
{
  return g_nts_frame0.result.dis_status;
}


