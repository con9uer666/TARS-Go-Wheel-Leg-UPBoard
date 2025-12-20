#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "main.h"

#define CAP_CANID 0x301

typedef struct
{
	struct
	{
		//	数值 需要/100；
		int16_t max_power; // cap_voltage*9
		int16_t cap_voltage;
		int16_t bus_power;
		int16_t L_current;
		//	状态变量
		uint8_t power_ctrl_mode; // 2-charge 1-output 0-turnoff
		uint8_t automode_stage;	 // 0 1 2

	} receive_data;
	float target_output_power;
	float cap_vot;
	float energy;
	float target_charge_power;
	float per_energy;
	float total_output;
	float cap_output;
	uint8_t power_ctrl_mode;
	uint8_t last_power_management;
} SuperCap;

void Cap_AnalysisData(void);
void Cap_CanSendData(void);

#define CAP_SET_POWER(x) (cap.target_power = x)
#define CHARGE 2
#define OUTPUT 1
#define TURNOFF 0

extern SuperCap cap;

#endif /* __SUPER_CAP_H */
// #ifndef __SUPER_CAP_H
// #define __SUPER_CAP_H

// #include "main.h"
// #include "graphics.h"
// #define CAP_SET_IN_POWER_CANID 0x601
// #define CAP_SET_OUT_VOT_CANID 0x602
// #define CAP_SET_OUT_CUR_CANID 0x603

// #define CAP_STATE_ERROR_CANID 0x610
// #define CAP_GET_IN_DATA_CANID 0x611
// #define CAP_GET_OUT_DATA_CANID 0x612

// typedef struct
//{
//	struct
//	{
//		int16_t input_pow;
//		uint16_t input_vot;
//		int16_t input_cur;
//		int16_t output_pow;
//		uint16_t output_vot;
//		int16_t output_cur;
//		uint16_t state;
//		uint16_t error;
//	}receive_data;
//	uint16_t set_target_power;
//	float energy;
//	float target_output_pow;
//	float target_input_pow;
//	float output_vot;
//	float output_pow;
//	float input_pow;
//	float per_energy;
//
// } SuperCap;

// #define CAP_SET_POWER(x) (cap.target_power = x)

// extern SuperCap cap;

// #endif /* __SUPER_CAP_H */
