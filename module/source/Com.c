// #include "Com.h"
// #include "Judge.h"
// #include "Moto.h"
// #include "RS485.h"
// #include "Chassis.h"
// #include "Gimbal.h"
// #include "Shooter.h"
// #include "usart.h"
// #include "dma.h"
// #include "detect.h"
// #include "Beep.h"

// uint8_t usart1RxBuf[COM_MAX_RX_LENGTH];
// Send_Data_t send_data;
// Receive_Data_t receive_data;
// extern DMA_HandleTypeDef hdma_usart1_rx;
// uint8_t JUDGE_IsLost; // 裁判系统离线

// int iii;
// uint8_t sendenable = 1;
// void RS485_LostCallback()
// {
//     Beep_PlayNotes((Note[]){{T_None, D_Quarter}, {T_H1, D_Sixteenth}, {T_M5, D_Sixteenth}, {T_M3, D_Sixteenth}, {T_M1, D_Sixteenth}}, 5);
// }
// void USER_USART1_IRQHandler(UART_HandleTypeDef *huart)
// {

//     if (huart == &huart2)
//     {
//         // iii++;
//         uint8_t len = COM_MAX_RX_LENGTH - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
//         COM_ReceiveData(usart1RxBuf);
//         memset(usart1RxBuf, 0, sizeof(usart1RxBuf));

//         sendenable = 1;
//     }
// }
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     //    if (huart == &huart1)
//     //    {
//     //        iii++;
//     //        COM_ReceiveData(usart1RxBuf);
//     //        memset(usart1RxBuf, 0, sizeof(usart1RxBuf));
//     //        sendenable = 1;
//     //        HAL_UART_Receive_DMA(&RS485_USART, usart1RxBuf, COM_MAX_TX_LENGTH);
//     //    }
// }
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart == &huart2)
//     {
//         RS485_DE(0);
//     }
// }

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     /* Prevent unused argument(s) compilation warning */

//     if (huart == &huart2)
//     {
//         iii++;
//         COM_ReceiveData(usart1RxBuf);
//         memset(usart1RxBuf, 0, sizeof(usart1RxBuf));
//         HAL_UARTEx_ReceiveToIdle_DMA(&RS485_USART, usart1RxBuf, COM_MAX_TX_LENGTH);
//        // __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//         sendenable = 1;
//     }
// }
// void COM_ReceiveData(uint8_t *data)
// {

//     uint8_t index = 0;
//     for (int i = 0; i < COM_MAX_RX_LENGTH - 2; i++)
//     {
//         if (data[i] == 0xA1 && data[i + 1] == 0x00)
//         {
//             index = i;
//             break;
//         }
//     }
//     uint16_t RxLen = (data[index + 1] << 8) | data[index + 2];
//     //        if(RxLen > COM_MAX_RX_LENGTH || (index + RxLen + 4) > receive_data.Rxlen) {
//     //            break;
//     //        }

//     uint8_t cmd = data[index + 3];
//     switch (cmd)
//     {
//     case 0x01: // 电机数据
//         memcpy(&receive_data.motor_info, &data[index + 4], RxLen);
//         break;
//         // case 0x02:  // 裁判系统数据
//         //     memcpy(&receive_data., &data[index + 4], sizeof(receive_data.));
//         //     break;
//         // case 0x03:  // UI数据
//         //     memcpy(&receive_data.Frame, &data[index + 4], sizeof(receive_data.Frame));
//         //     break;
//     default:
//         break;
//     }

//     index += RxLen + 4;
//     Detect_Update(DeviceID_RS485);
// }

// void COM_SendData(Send_Data_t *data)
// {
//     static uint8_t txBuf[COM_MAX_TX_LENGTH];
//     uint16_t index = 0;

//     txBuf[index++] = 0xA1; // header
//     uint16_t len = sizeof(int16_t) * MOTOR_NUM;
//     txBuf[index++] = (len >> 8) & 0xFF;
//     txBuf[index++] = len & 0xFF;
//     txBuf[index++] = 0x01; // cmd
//     memcpy(&txBuf[index], &data->output, len);
//     index += len;

//     // txBuf[index++] = 0xA1;
//     // len = sizeof(JudgeTxFrame);
//     // txBuf[index++] = (len >> 8) & 0xFF;
//     // txBuf[index++] = len & 0xFF;
//     // txBuf[index++] = 0x02;
//     // memcpy(&txBuf[index], &data->Frame, len);
//     // index += len;

//     RS485_Transmit(txBuf, COM_MAX_TX_LENGTH);
// }

// void COM_UpdateData(void)
// {
//     // 更新电机数据
//     Turn_Update(&chassis.motors[1],
//                 receive_data.motor_info[MOTOR_LF_TURN].angle,
//                 receive_data.motor_info[MOTOR_LF_TURN].speed,
//                 receive_data.motor_info[MOTOR_LF_TURN].torque,
//                 receive_data.motor_info[MOTOR_LF_TURN].temp);
//     Turn_Update(&chassis.motors[2],
//                 receive_data.motor_info[MOTOR_RF_TURN].angle,
//                 receive_data.motor_info[MOTOR_RF_TURN].speed,
//                 receive_data.motor_info[MOTOR_RF_TURN].torque,
//                 receive_data.motor_info[MOTOR_RF_TURN].temp);
//     Turn_Update(&chassis.motors[0],
//                 receive_data.motor_info[MOTOR_LB_TURN].angle,
//                 receive_data.motor_info[MOTOR_LB_TURN].speed,
//                 receive_data.motor_info[MOTOR_LB_TURN].torque,
//                 receive_data.motor_info[MOTOR_LB_TURN].temp);
//     Turn_Update(&chassis.motors[3],
//                 receive_data.motor_info[MOTOR_RB_TURN].angle,
//                 receive_data.motor_info[MOTOR_RB_TURN].speed,
//                 receive_data.motor_info[MOTOR_RB_TURN].torque,
//                 receive_data.motor_info[MOTOR_RB_TURN].temp);
//     Drive_Update(&chassis.motors[1],
//                  receive_data.motor_info[MOTOR_LF_DRIVE].angle,
//                  receive_data.motor_info[MOTOR_LF_DRIVE].speed,
//                  receive_data.motor_info[MOTOR_LF_DRIVE].torque,
//                  receive_data.motor_info[MOTOR_LF_DRIVE].temp);
//     Drive_Update(&chassis.motors[2],
//                  receive_data.motor_info[MOTOR_RF_DRIVE].angle,
//                  receive_data.motor_info[MOTOR_RF_DRIVE].speed,
//                  receive_data.motor_info[MOTOR_RF_DRIVE].torque,
//                  receive_data.motor_info[MOTOR_RF_DRIVE].temp);
//     Drive_Update(&chassis.motors[0],
//                  receive_data.motor_info[MOTOR_LB_DRIVE].angle,
//                  receive_data.motor_info[MOTOR_LB_DRIVE].speed,
//                  receive_data.motor_info[MOTOR_LB_DRIVE].torque,
//                  receive_data.motor_info[MOTOR_LB_DRIVE].temp);
//     Drive_Update(&chassis.motors[3],
//                  receive_data.motor_info[MOTOR_RB_DRIVE].angle,
//                  receive_data.motor_info[MOTOR_RB_DRIVE].speed,
//                  receive_data.motor_info[MOTOR_RB_DRIVE].torque,
//                  receive_data.motor_info[MOTOR_RB_DRIVE].temp);

//     Motor_Update(&gimbal.yawMotor, receive_data.motor_info[MOTOR_YAW].angle,
//                  receive_data.motor_info[MOTOR_YAW].speed,
//                  receive_data.motor_info[MOTOR_YAW].torque, receive_data.motor_info[MOTOR_YAW].temp);
//     Motor_Update(&shooter.triggerMotor, receive_data.motor_info[MOTOR_TRIGGER].angle,
//                  receive_data.motor_info[MOTOR_TRIGGER].speed,
//                  receive_data.motor_info[MOTOR_TRIGGER].torque, receive_data.motor_info[MOTOR_TRIGGER].temp);

//     // 更新裁判系统数据

//     // 更新UI数据
// }

// void COM_Init(void)
// {
//     RS485_Init();
// }
// extern TIM_HandleTypeDef htim9;
// uint8_t timecnt = 0;
// void OS_ComCallBack(void const *argument)
// {
//     osDelay(5);
//     //HAL_TIM_Base_Start_IT(&htim9);
//     // __HAL_UART_ENABLE_IT(&RS485_USART, UART_IT_IDLE);
//     // HAL_UART_Receive_DMA(&RS485_USART, usart1RxBuf, COM_MAX_TX_LENGTH);
// //    HAL_UARTEx_ReceiveToIdle_DMA(&RS485_USART, usart1RxBuf, COM_MAX_TX_LENGTH);
//    // __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

//     for (;;)
//     {
//         osDelay(1);

//     }
// }
