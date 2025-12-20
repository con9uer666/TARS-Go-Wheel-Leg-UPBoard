#ifndef _USER_LASER_H_
#define _USER_LASER_H

#include <stdint.h>
// 默认使用UART1作为SK80通信串口
// ADDR使用默认值0x80(单模块)
#define UART1_PutChar(a) UART_PutChar(&huart1, a)
#define disNum(a) ~(a) + 1 // 取补码

// Function Declaration
/**
 * @brief 测距模块初始化
 * @param[in] none
 * @param[out] none
 * @retval 设置起始点为顶端 量程20m,频率20Hz,分辨率1mm,开启激光瞄准
 */
void Laser_Init(void);

/**
 * @brief 设置起始点
 * @param[in] PositionStart
 * @param[out] none
 * @retval 0从尾端算起 1从始端算起
 */
uint8_t LASER_SetPositionStart(uint8_t start);

/**
 * @brief 设置量程
 * @param[in] Range
 * @param[out] none
 * @retval 5m,10m,30m,50m,80m
 */
uint8_t Laser_SetRange(uint8_t range);

/**
 * @brief 设置频率
 * @param[in] Frequency
 * @param[out] none
 * @retval 1(5Hz),2(10Hz),3(20Hz)
 */
uint8_t Laser_SetFreq(uint8_t freq);

/**
 * @brief 设置分辨率
 * @param[in] Solution
 * @param[out] none
 * @retval 1(1mm),2(0.1mm)
 */
uint8_t Laser_SetSolution(uint8_t solu);

/**
 * @brief 修改距离
 * @param[in] Singal Distance
 * @param[out] none
 * @retval signal:0x2B为正,0x2D为负
 * @example +23mm:FA 04 06 2B 17 BA
 */
uint8_t Laser_ChangeDistance(uint8_t signal, uint8_t dist);

/**
 * @brief 设置上电即测
 * @param[in] Start
 * @param[out] none
 * @retval 0(关闭),1(开启)
 */
uint8_t Laser_MeasuredOnPower(uint8_t start);

/**
 * @brief 是否打开激光瞄准
 * @param[in] Enable
 * @param[out] none
 * @retval 0(关闭),1(开启)
 */
uint8_t Laser_OpenLight(uint8_t enable);

/**
 * @brief 单次测量
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_MeasureSignal(void);

/**
 * @brief 单次测量-广播
 * @param[in] none
 * @param[out] none
 * @retval 通过发送单次测量-广播&读取缓存 加快测量
 */
void Laser_MeasureBroadcast(void);

/**
 * @brief 读取缓存
 * @param[in] none
 * @param[out] none
 * @retval 配合单次广播使用加快测量
 */
void Laser_ReadCache(void);

/**
 * @brief 连续测量
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_MeasureContinous(void);

/**
 * @brief 设置数据返回时间间隔
 * @param[in] MeaInterver
 * @param[out] none
 * @retval 连续测量模式下 单位ms
 */
void Laser_SetMeaInte(uint8_t Int);

/**
 * @brief 获取到目标实际距离
 * @param[in] *Distance
 * @param[out] none
 * @retval
 */
uint8_t Laser_GetDistance(uint16_t *distance);

/**
 * @brief 关机
 * @param[in] none
 * @param[out] none
 * @retval
 */
void Laser_Close(void);

#endif
