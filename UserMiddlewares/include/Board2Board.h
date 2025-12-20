#ifndef B2B_H
#define B2B_H
#include "stdint.h"


extern uint8_t STOPFLAG;
extern uint8_t FEEDBACK;

void RS485_Init(void);
void Rs485_Trans(void);
void RS485_Rec(void);


#endif
