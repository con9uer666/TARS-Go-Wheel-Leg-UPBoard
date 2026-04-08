#ifndef B2B_H
#define B2B_H
#include "stdint.h"


extern uint8_t STOPFLAG;
extern uint8_t FEEDBACK;
extern uint8_t gimbal_follow_flag;

void RS485_Init(void);
void Rs485_Trans(void);
void RS485_Rec(void);



#endif
