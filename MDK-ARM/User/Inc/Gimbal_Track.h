#ifndef __GIMBAL_TRACK_H__
#define __GIMBAL_TRACK_H__

#include "main.h"
#include "PID.h"

extern uint8_t UART1_IMU_AND_Motor_RData[20];//存储串口最新数据的数组
extern uint8_t rx_done;//数据包接收完成标志

/*云台稳定PID数据结构体外部声明*/
extern PID_t Gimbal_track_PID_X_Data;
extern PID_t Gimbal_track_PID_Y_Data;

/*摄像机传来偏转数据结构体外部声明*/
extern Came_Data Gimbal_track_Came_Data;

//云台俯仰单独的稳定算法
void Gimbal_Track_PID(PID_t *ax, PID_t *ay, float x_raw, float y_raw);
//云台偏航单独的稳定算法
void Gimbal_Track_PID_X(PID_t *ax, float x_raw);
void Gimbal_Track_PID_Y(PID_t *ay, float y_raw);

//串口接收角度数据控制电机
void UART_IMU_AND_Motor(UART_HandleTypeDef *huart);

#endif
