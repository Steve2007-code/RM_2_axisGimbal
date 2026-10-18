#ifndef __DM_BLDC_Motor_H__
#define __DM_BLDC_Motor_H__

#include "main.h"

/*达妙电机MIT模式下的参数*/
typedef struct
	{
		CAN_HandleTypeDef *hcan;        /*电机所接的CAN线*/
		uint16_t ID;										/*电机ID*/
		float Position;									/*电机位置*/
		float Position_max;							/*电机位置_max*/
		float Position_min;							/*电机位置_min*/
		float Velocity;									/*电机转速*/
		float Velocity_max;							/*电机转速_max*/
		float Velocity_min;							/*电机转速_min*/
		float Kp;												/*Kp*/
		float Kp_max;										/*Kp_max*/
		float Kp_min;										/*Kp_min*/
		float Kd;												/*Kd*/
		float Kd_max;										/*Kd_max*/
		float Kd_min;										/*Kd_min*/
		float Torque;										/*转矩*/
		float Torque_max;								/*转矩_max*/
		float Torque_min;								/*转矩_min*/
	}DM_Motor_Data;

/*达妙电机结构体声明*/
extern DM_Motor_Data DM_Motor_4340_StructData;
extern DM_Motor_Data DM_Motor_4310_StructData;

/*电机ID*/
#define DM_Motor_4310_ID 0x001
#define DM_Motor_4340_ID 0x002

/*CAN数据帧声明*/
extern uint8_t DM_Mototr_Enable[8];
extern uint8_t DM_Mototr_Disable[8];

/*常量定义*/
#ifndef PI
#define PI 3.141592653579f
#endif

/*函数声明*/
/*DM电机初始化*/
void DM_CAN_Motor_Init(void);
/*CAN发送一帧数据给电机*/
HAL_StatusTypeDef DM_CAN_Motor_Send_Data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len);
/*CAM发送电机使/失能*/
HAL_StatusTypeDef DM_CAN_Motor_SetEnable(uint16_t id, uint8_t *data);
/*CAN发送电机旋转数据帧(弧度制)(定位置控制)*/
HAL_StatusTypeDef DM_CAN_Motor_Rotation_Rad(DM_Motor_Data *ax, float Position);
/*CAN发送电机旋转数据帧(角度制)(定位置控制)*/
HAL_StatusTypeDef DM_CAN_Motor_Rotation_deg(DM_Motor_Data *ax, float Position);
/*CAN发送电机旋转数据帧(弧度制)(定速度控制)*/
HAL_StatusTypeDef DM_CAN_Motor_Velocity_Rad(DM_Motor_Data *ax, float vel);
/*CAN发送电机MIT*/
HAL_StatusTypeDef DM_CAN_Motor_MIT(DM_Motor_Data *ax, float vel, float pos, float kp, float kd, float tor);

#endif
