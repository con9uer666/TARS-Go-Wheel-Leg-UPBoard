#ifndef _RANGE_H_
#define _RANGE_H_
#include "main.h"
#define RXBUFFERSIZE 11 // 接收数组值

void Range_Init(void);
void Range_IRQHandler(void);

float Range_get_distance_m(void);
uint16_t Range_get_distance_mm(void);
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart);
#endif // !_RANGE_H_

