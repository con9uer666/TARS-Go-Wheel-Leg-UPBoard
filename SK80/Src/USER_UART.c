#include "USER_UART.h"
#include <stdint.h>

/**
 * @file USER_UART.c
 * @brief 通用串口驱动
 * @retval 根据使用需要进行后续拓展 接口默认全部引出
 */

#define UART1_ENABLE

/**
 * 创建软件FIFO缓冲区数组
 */
#ifdef UART1_ENABLE
#define UART1_RX_BUFFSIZE 128
#define UART1_TX_BUFFSIZE 128
uint8_t UART1_RxBuff[UART1_RX_BUFFSIZE]; // 接收缓冲区
uint8_t UART1_TxBuff[UART1_TX_BUFFSIZE]; // 发送缓冲区
UART_Info UART1;

void UART1_InfoInit()
{
	UART1.Rx.length = 0;
	UART1.Rx.RIndex = 0;
	UART1.Rx.WIndex = 0;
	UART1.Tx.length = 0;
	UART1.Tx.RIndex = 0;
	UART1.Tx.WIndex = 0;
	UART1.Tx.Ready = 1;
}

#endif

// 默认使用UART1作为SK80通信串口
bool UART_GetChar(UART_HandleTypeDef *huart, uint8_t *byte)
{
	if (huart == &huart1)
	{
		if (UART1.Rx.length <= 0) // 缓冲区没有待读数据
			return false;
		else
		{
			// 关断中断
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

			*byte = UART1_RxBuff[UART1.Rx.RIndex];
			UART1.Rx.length--;
			UART1.Rx.RIndex++;
			if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
			{
				UART1.Rx.RIndex = 0;
			}
			// 开启中断
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			return true;
		}
	}
	else
		return false;
}
bool UART_GetLastChar(UART_HandleTypeDef *huart, uint8_t *byte)
{
	if (huart == &huart1)
	{
		if (UART1.Rx.length <= 0) // 缓冲区没有待读数据
			return false;
		else
		{
			// 关断中断
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

			// 虚读UART1.Rx.length-1个数据
			for (uint16_t index = 0; index < UART1.Rx.length - 1; index++)
			{
				*byte = UART1_RxBuff[UART1.Rx.RIndex];
				UART1.Rx.length--;
				UART1.Rx.RIndex++;
				if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
				{
					UART1.Rx.RIndex = 0;
				}
			}
			*byte = UART1_RxBuff[UART1.Rx.RIndex];
			UART1.Rx.length--;
			UART1.Rx.RIndex++;
			if (UART1.Rx.RIndex >= UART1_RX_BUFFSIZE)
			{
				UART1.Rx.RIndex = 0;
			}
			// 开启中断
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			return true;
		}
	}
	else
		return false;
}
bool UART_GetStr(UART_HandleTypeDef *huart, uint8_t *string)
{
	do
	{
		if (!UART_GetChar(&huart1, string))
			return false;
		string++;
	} while (*(string - 1));
	return true;
}

bool UART_GetDatas(uint8_t *datas, uint16_t length)
{
	if (length > UART1.Rx.length) // 缓冲区数据不足 无法读取
		return false;
	while (length--)
	{
		if (!UART_GetChar(&huart1, datas)) // 某步读取失败
			return false;
		datas++;
	}
	return true;
}

bool UART_PutChar(UART_HandleTypeDef *huart, uint8_t byte)
{
	if (huart == &huart1)
	{
		if (UART1.Tx.Ready) // 串口空闲
		{
			HAL_UART_Transmit_IT(&huart1, &byte, sizeof(&byte));
			UART1.Tx.Ready = 0; // busy
			return true;
		}
		else
		{
			if (UART1.Tx.length >= UART1_TX_BUFFSIZE) // 发送缓冲区已满
				return false;
			else
			{
				// 关断中断
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
				UART1_TxBuff[UART1.Tx.WIndex++] = byte;
				if (UART1.Tx.WIndex >= UART1_TX_BUFFSIZE)
					UART1.Rx.WIndex = 0;
				UART1.Tx.length++;
				// 开启中断
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
				return true;
			}
		}
	}
	else
		return false;
}
void UART_ClearRXBuff(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		UART1.Rx.length = 0;
		UART1.Rx.RIndex = 0;
		UART1.Rx.WIndex = 0;
	}
}
