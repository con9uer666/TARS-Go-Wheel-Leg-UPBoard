#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "struct_typedef.h"
#include "tim.h"
extern void imu_pwm_set(uint16_t pwm);
#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#endif
