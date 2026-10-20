#ifndef __PID_H
#define __PID_H

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
	uint8_t lost;//1 = 未检测到方块
} Came_Data;

/*外部可调用函数声明*/
void PID_Update(PID_t *p);

#endif
