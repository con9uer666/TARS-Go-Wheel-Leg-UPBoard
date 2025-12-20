#include "main.h"
#include "nlink_tofsense_frame0.h"
#include "nlink_utils.h"
#include "stdbool.h"

#define MAX_TOF_BUFFER  32

void Tof_Init(void);
void USER_USART1_IRQHandler(UART_HandleTypeDef *huart);
float Tof_GetDistance(void);
uint16_t Tof_GetSignalStrength(void);
bool Tof_IsRight(void);


