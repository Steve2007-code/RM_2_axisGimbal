#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "Gimbal_Track.h"
#include "UART_Data.h"
#include "DM_BLDC_Motor.h"
#include "DM_IMU_L1_V1.0.h"

/*云台稳定PID数据结构体定义*/
//俯仰
PID_t Gimbal_track_PID_X_Data = {
	.Target = 0,
	
	.Kp = 8.0f,
	.Ki = 0.80f,
	.Kd = 0.5f,
	
	.OutMax = 2.5,
	.OutMin = -2.5,
	
	.KiMax = 100,
	.KiMin = -100,
};
//偏航
PID_t Gimbal_track_PID_Y_Data =  {
	.Target = 0,
	
	.Kp = 8.0f,
	.Ki = 0.6f,
	.Kd = 0.50f,
	
	.OutMax = 2.5,
	.OutMin = -2.5,
	
	.KiMax = 100,
	.KiMin = -100,
};

/*摄像机传来偏转数据结构体*/
Came_Data Gimbal_track_Came_Data = {
	.Distance = 0.0f,
	.x_raw = 0.0f,
	.y_raw = 0.0f,
};

/**
  * @brief  云台俯仰单独的稳定算法
  * @param  *ax PID_t类型结构体 x_raw x(俯仰)轴稳定角度
  * @note		无
  * @retval 无
  */
void Gimbal_Track_PID_X(PID_t *ax, float x_raw)
{
	ax ->Target = x_raw;
	
	//Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Euler_angle_data);
	//Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Angular_velocity_data);
	
	ax ->Actual = DM_Motor_IMU_StructData.Euler_f_roll * PI / 180.0f;
	//float Feedforward_K = 0.03f;//角速度前馈增益
	//float PID_Gyro_f_raw[3];
	//memcpy(PID_Gyro_f_raw, DM_Motor_IMU_StructData.Gyro_f_raw, sizeof(PID_Gyro_f_raw));
	//float Dist_Feedforward = -PID_Gyro_f_raw[0];// * Feedforward_K;
	
	PID_Update(ax);
	
	DM_Motor_4310_StructData.Position =ax ->Out;// + Dist_Feedforward;
	
	DM_CAN_Motor_MIT(&DM_Motor_4310_StructData, 
										DM_Motor_4310_StructData.Velocity,
										DM_Motor_4310_StructData.Position,
										DM_Motor_4310_StructData.Kp,
										DM_Motor_4310_StructData.Kd,
										DM_Motor_4310_StructData.Torque);
}

/**
  * @brief  云台偏航单独的稳定算法
  * @param  *ay PID_t类型结构体 y(横滚)轴稳定角度
  * @note		无
  * @retval 无
  */
void Gimbal_Track_PID_Y(PID_t * ay, float y_raw)
{
	ay ->Target = y_raw;
	//static float Old_Out = 0.0f;
	
	//Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Euler_angle_data);
	//Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Angular_velocity_data);
	
	ay ->Actual = DM_Motor_IMU_StructData.Euler_f_yaw * PI / 180.0f;//IMU角度转弧度
	//float Feedforward_K = 0.02f;//角速度前馈增益
	//float Feedforward_K = 0.0f;//角速度前馈增益
	//float PID_Gyro_f_raw[3];
	//memcpy(PID_Gyro_f_raw, DM_Motor_IMU_StructData.Gyro_f_raw, sizeof(PID_Gyro_f_raw));
	//float Dist_Feedforward = PID_Gyro_f_raw[2];// * Feedforward_K;
	
	PID_Update(ay);
	
	DM_Motor_4340_StructData.Position =ay ->Out;// + Dist_Feedforward;
	
	DM_CAN_Motor_MIT(&DM_Motor_4340_StructData, 
										DM_Motor_4340_StructData.Velocity,
										DM_Motor_4340_StructData.Position,
										DM_Motor_4340_StructData.Kp,
										DM_Motor_4340_StructData.Kd,
										DM_Motor_4340_StructData.Torque);
}


/**
  * @brief  串口接收角度数据控制电机
  * @param  *huart 串口句柄（建议为串口6）
  * @note		无
  * @retval 无
  */
uint8_t UART1_IMU_AND_Motor_RData[20];//存储串口最新数据的数组
uint8_t rx_done = 0;//数据包接收完成标志

void UART_IMU_AND_Motor(UART_HandleTypeDef *huart)
{
	uint8_t flag = 0;//读缓存区数据的阶段 0-等包头 1-包头第二个字节 2-包头第三个字节 3-取包数据 4-读到包尾等待新包头
	uint8_t flag_Data = 0;//串口数据储存索引
	while (UART6_RX_Rd != UART6_RX_idx)// 还有积压字节没读
		{
			uint8_t byte = UART6_RX_Fifo[UART6_RX_Rd];
			UART6_RX_Rd = (UART6_RX_Rd + 1) % 128;
			//串口接收到摄像头来的数据之后，进行数据存储（两帧数据间隔约34-36ms 数据发送持续时间约1.39ms）
			switch(flag)
			{
				case 0:
			 {
					if (byte == 0x59 && flag == 0)//等待Y
					{
						flag = 1;
					}
					else
						flag = 0;
					break;
				}
			 case 1:
			 {
					if(byte == 0x53 && flag == 1)//等待S
					{
						flag = 2;
					}
					else
						flag = 0;
					break;
				}
			 case 2:
			 {
					if(byte == 0x41 && flag == 2)//等待A
					{
						flag_Data = 0;
						flag = 3;
					}
					else
						flag = 0;
					break;
				}
			 case 3:
			 {
					if(flag == 3 && byte == 0x2C)
					{
						flag = 31;
					}
					else
						flag = 0;
					break;
				}
			 
				/*读取第一个数据*/
			 case 31:
			 {
					if(flag == 31 && byte != 0x2C)//读取X偏移数据
					{
						UART1_IMU_AND_Motor_RData[flag_Data++] = byte;//连续存储第一个数据
					}
					else//直到遇到逗号(0x2C)
					{
						UART1_IMU_AND_Motor_RData[flag_Data] = '\0';
						Gimbal_track_Came_Data.x_raw = strtof((char*)UART1_IMU_AND_Motor_RData, NULL);
						flag_Data = 0;
						flag = 32;
					}
					break;
				}
			 /*读取第二个数据*/
				case 32:
				{
					if(flag == 32 && byte != 0x2C)//读取Y偏移数据
					{
						UART1_IMU_AND_Motor_RData[flag_Data++] = byte;//连续存储第二个数据
					}
					else//直到遇到逗号(0x2C)
					{
						UART1_IMU_AND_Motor_RData[flag_Data] = '\0';
						Gimbal_track_Came_Data.y_raw = strtof((char*)UART1_IMU_AND_Motor_RData, NULL);
						flag_Data = 0;
						flag = 33;
					}
					break;
				}
				/*读取第三个数据*/
				case 33:
				{
					if(flag == 33 && byte != 0x0A)//连续存储第三个数据(目标距离)
					{
						UART1_IMU_AND_Motor_RData[flag_Data++] = byte;
					}
					else//直到遇到换行符号(0x0A)
					{
						UART1_IMU_AND_Motor_RData[flag_Data] = '\0';
						Gimbal_track_Came_Data.Distance = strtof((char*)UART1_IMU_AND_Motor_RData, NULL);
						flag_Data = 0;
						rx_done = 1;
					}
					break;
				}
			}
   }
}



