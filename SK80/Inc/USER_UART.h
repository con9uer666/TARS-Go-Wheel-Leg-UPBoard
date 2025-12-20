#ifndef _USER_UART_H_
#define _USER_UART_H_
#include "usart.h"
#include <stdbool.h>

typedef struct
{
	UART_HandleTypeDef huart;
	struct
	{
		uint16_t length; // 待接收字符串长度
		uint16_t RIndex; // 接收读指针
		uint16_t WIndex; // 接收写指针

	} Rx;
	struct
	{
		uint16_t length; // 待发送字符串长度
		uint16_t RIndex; // 发送读指针
		uint16_t WIndex; // 发送写指针
		uint8_t Ready;	 // 空闲标志
	} Tx;
} UART_Info;

extern UART_Info UART1;

/**
 * @brief 串口获取单个数据
 * @param[in] *byte
 * @param[out] true/false
 * @retval none
 */
bool UART_GetChar(UART_HandleTypeDef *huart, uint8_t *byte);

/**
 * @brief 串口获取最近一次数据
 * @param[in] *byte
 * @param[out] true/false
 * @retval none
 */
bool UART_GetLastChar(UART_HandleTypeDef *huart, uint8_t *byte);

/**
 * @brief 串口获取字符串
 * @param[in] *string
 * @param[out] true/false
 * @retval none
 */
bool UART_GetStr(UART_HandleTypeDef *huart, uint8_t *string);

/**
 * @brief 串口获取多个数据
 * @param[in] *datas
 * @param[out] true/false
 * @retval none
 */
bool UART_GetDatas(uint8_t *datas, uint16_t length);

/**
 * @brief 串口发送单个数据
 * @param[in] byte
 * @param[out] true/false
 * @retval none
 */
bool UART_PutChar(UART_HandleTypeDef *huart, uint8_t byte);

/**
 * @brief 清除串口接收缓冲区
 * @param[in] byte
 * @param[out] true/false
 * @retval none
 */
void UART_ClearRXBuff(UART_HandleTypeDef *huart);
#endif
