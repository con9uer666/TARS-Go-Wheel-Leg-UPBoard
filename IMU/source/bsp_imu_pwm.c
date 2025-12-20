#include "bsp_imu_pwm.h"
#include "main.h"
#include "pid.h"
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm) 

static uint8_t first_temperate;
PID imu_temp_pid;

extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}

static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_SingleCalc(&imu_temp_pid,45,temp);
		
        if (imu_temp_pid.output < 0.0f)
        {
            imu_temp_pid.output = 0.0f;
        }
		
        tempPWM = (uint16_t)imu_temp_pid.output;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45)
        {
            temp_constant_time++;
			
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.integral = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
			
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

