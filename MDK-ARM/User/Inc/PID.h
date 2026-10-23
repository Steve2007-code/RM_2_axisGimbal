#ifndef __PID_H
#define __PID_H

#include "main.h"

/*PID数据存储结构体*/
typedef struct {
	float Target;//目标
	float Actual;//实际
	float Out;//输出
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;//本次误差
	float Error1;//上次误差
	float ErrorInt;//误差
	
	float OutMax;//最大输出
	float OutMin;//最小输出
	
	float KiMax;//Ki最大限幅
	float KiMin;//Ki最小限幅
} PID_t;

typedef struct {
	float x_raw;//x还差的角度
	float y_raw;//y还差的角度
	float Distance;//目标距离
	uint8_t Yes_Red_Data_flag;//识别并成功解析一帧数据的标志位 1-接收一帧成功 0-接收失败/无效
} Came_Data;

/*外部可调用函数声明*/
void PID_Update(PID_t *p);

#endif
