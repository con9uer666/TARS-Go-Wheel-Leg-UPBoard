#include "range.h"
#include "usart.h"
#include "cmsis_os.h"
/**
 * @brief 测距驱动
 * @details 串口1 波特率9600 8位数据位 1位起始位 1位停止位 无奇偶校验位
 * @author scpsyl
 */
float distance_m;
uint16_t distance_mm;
uint8_t rx_data[11];
SemaphoreHandle_t distanceMutex;

void TOF_single_init()
{
  uint8_t send1[4] = {0x80, 0x06, 0x02, 0x78};
  HAL_UART_Transmit_IT(&huart1, send1, sizeof(send1));
}
void TOF_series_init()
{
  uint8_t send0[4] = {0x80, 0x06, 0x03, 0x77};
  HAL_UART_Transmit_IT(&huart1, send0, sizeof(send0));
}

void Range_ISR_Init()
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void Range_Init()
{
  // 默认连续发送
  TOF_series_init();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

// 中断回调
void Range_IRQHandler(void)
{
	static uint8_t Rx_indx = 0; 
  if ((__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET ))
  {
		//__HAL_UART_CLEAR_NEFLAG(&huart1);
    rx_data[Rx_indx++] = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
		if(Rx_indx == RXBUFFERSIZE)
		{
			distance_m = 100 * (rx_data[3] - 0x30) + 10 * (rx_data[4] - 0x30) + (rx_data[5] - 0x30) + (rx_data[7] - 0x30) * 0.1 + (rx_data[8] - 0x30) * 0.01 + (rx_data[9] - 0x30) * 0.001;
      distance_mm = distance_m * 1000;
      Rx_indx = 0;
		}
	}
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC))
	{
		__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_TC);
	
		 //创建互斥锁
		 //
//		xSemaphoreTake(distanceMutex, portMAX_DELAY);
      
//		xSemaphoreGive(distanceMutex);
      
      TOF_series_init();
}
	}
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart)
{
	static uint8_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
	{
		__HAL_UART_CLEAR_NEFLAG(huart);
		rx_data[rxCnt++] = huart->Instance->DR;
	}
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    distance_m = 100 * (rx_data[3] - 0x30) + 10 * (rx_data[4] - 0x30) + (rx_data[5] - 0x30) + (rx_data[7] - 0x30) * 0.1 + (rx_data[8] - 0x30) * 0.01 + (rx_data[9] - 0x30) * 0.001;
    distance_mm = distance_m * 1000;
		rxCnt = 0;
    
    
  }
}
// 读取距离 m为单位
float Range_get_distance_m()
{
  return distance_m;
}

// 读取距离 mm为单位
uint16_t Range_get_distance_mm()
{
  return distance_mm;
}

