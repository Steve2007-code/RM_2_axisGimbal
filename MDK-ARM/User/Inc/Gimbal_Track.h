#ifndef __GIMBAL_TRACK_H__
#define __GIMBAL_TRACK_H__

#include "main.h"

extern uint16_t lost_cnt;/*目标丢失*/

extern int16_t Pixel_x , Pixel_y;/*像素差值储存*/
extern uint8_t Frame_OK;/*转换到一帧有效帧置 1*/

void UART6_Data_Frame(void);/*串口接收数据解数据帧*/
void Gimbal_Track_PID(void);/*云台双轴PID计算*/

#endif
