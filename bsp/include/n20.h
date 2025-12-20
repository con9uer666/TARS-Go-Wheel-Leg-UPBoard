#ifndef TEST1_N20_H
#define TEST1_N20_H
#include "PID.h"
#include "tim.h"
#include "moto.h"
//typedef struct{
//	float speed;
//	float angle;
//	CascadePID pid;
//	int cnt;
//	int last_cnt;
//}N20;
void N20_Init(void);
void N20_Out(void);
void N20_In(void);
void N20_Stop(void);
void N20_TaskOut(void);
void N20_TaskIn(void);


typedef struct
{
	int16_t angle;
	float speed;
	int16_t lastAngle;//记录上一次得到的角度
	
	float targetSpeed;//目标速度
	int32_t targetAngle;//目标角度(编码器值)
	
	int32_t totalAngle;//累计转过的编码器值
	
	PID speedPID;//速度pid(单级)
	CascadePID anglePID;//角度pid，串级
}N20;

extern N20 n20;
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif //TEST1_N20_H
