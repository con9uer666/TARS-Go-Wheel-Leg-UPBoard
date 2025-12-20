#ifndef _RS485_DRV_H_
#define _RS485_DRV_H_

#include "usart.h"

/* User Config */
#define RS485_USART         huart2
#define RS485_RE_GPIO_PORT  GPIOE
#define RS485_RE_GPIO_PIN   GPIO_PIN_13

/* RS485 TX/RX Control */
#define RS485_DE(x)   do{ x ? \
    HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
    HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
}while(0)

void RS485_Init(void);
int RS485_Transmit(uint8_t *data, uint16_t len);
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart);

#endif /* _RS485_DRV_H_ */
