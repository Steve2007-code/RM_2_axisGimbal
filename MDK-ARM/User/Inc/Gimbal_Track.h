#ifndef __GIMBAL_TRACK_H__
#define __GIMBAL_TRACK_H__

#include "main.h"
#include "PID.h"

/*-------------------------- 像素pid追踪目标--------------------------------*/
//extern uint16_t lost_cnt;/*目标丢失*/

//extern int16_t Pixel_x , Pixel_y;/*像素差值储存*/
//extern uint8_t Frame_OK;/*转换到一帧有效帧置 1*/

//void UART6_Data_Frame(void);/*串口接收数据解数据帧*/
//void Gimbal_Track_PID(void);/*云台双轴PID计算*/
/*-------------------------- 像素pid追踪目标--------------------------------*/

extern float flag;
extern float out1;
extern float out2;

/*云台稳定PID数据结构体外部声明*/
extern PID_t Gimbal_track_PID_X_Data;
extern PID_t Gimbal_track_PID_Y_Data;

//云台俯仰单独的稳定算法
void Gimbal_Track_PID(PID_t *ax, PID_t *ay, float x_raw, float y_raw);
//云台偏航单独的稳定算法
void Gimbal_Track_PID_X(PID_t *ax, float x_raw);
void Gimbal_Track_PID_Y(PID_t *ay, float y_raw);

#endif
