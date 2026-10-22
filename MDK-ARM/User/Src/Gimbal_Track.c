#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "Gimbal_Track.h"
#include "UART_Data.h"
#include "DM_BLDC_Motor.h"
#include "DM_IMU_L1_V1.0.h"

/*-------------------------- 像素pid追踪目标--------------------------------*/
///*像素差值储存*/
//int16_t Pixel_x , Pixel_y;
///*转换到一帧有效帧置 1*/
//uint8_t Frame_OK = 0;
///*当前解析字符串的临时储存*/
//char String_Database[24];
///*目标丢失*/
//uint16_t lost_cnt = 0;

///*
//  * @brief  将字符串转换为浮点数
//  * @param  string 字符串组
//  * @note		无
//  * @retval 成功解析两个浮点数返回 1 ,否则返回0
//  */
//uint8_t String_To_Int16(char *string, int16_t *Pixel_x, int16_t *Pixel_y)
//{
//    int ret = sscanf(string, "YSC,%hd,%hd", Pixel_x, Pixel_y);
//    if(ret == 2)
//    {
//        return 1;
//    }
//    return 0;
//}

///**
//  * @brief  串口接收数据解数据帧
//  * @param  无
//  * @note		无
//  * @retval 无
//  */
//void UART6_Data_Frame(void)
//{
//	//串口接收数据临时中转变量
//	uint8_t Flage;
//	//接收数据总量记录函数
//	static uint8_t Line_Flage;
//	
//	while(UART6_RX_idx != UART6_RX_Rd)
//	{
//		Flage = UART6_RX_Fifo[UART6_RX_Rd];
//		UART6_RX_Rd = (UART6_RX_Rd + 1) % 128;
//		
//		String_Database[Line_Flage] = Flage;
//		Line_Flage += 1;
//		if(Line_Flage >= sizeof(String_Database)-1)
//		{
//			Line_Flage = 0;
//		}
//		
//		if(Flage == '\n' || Flage == '\r')
//		{
//			String_Database[Line_Flage] = '\0';
//			Frame_OK = String_To_Int16(String_Database, &Pixel_x, &Pixel_y);
//			if(Frame_OK){ lost_cnt = 0; } else if(lost_cnt < 600){ lost_cnt++; }
//			Line_Flage = 0;
//		}
//	}
//}

///**
//  * @brief  云台双轴PID计算
//  * @param  无
//  * @note		无
//  * @retval 无
//  */
//void Gimbal_Track_PID(void)
//{
//    float Kp = 0.0080f;
//    float Kd = 0.0003f;
//    float dt = 0.0105f;
//    float target = 0.0f;
//    float error_x, error_y;
//    float out_x, out_y;
//    float pit_limit;

//    static float error_x_last = 0.0f;
//    static float error_y_last = 0.0f;
//    static float target_pit   = 0.0f;   /* 俯仰目标角 rad */

//    error_x = target - (float)Pixel_x;  /* 偏航 */
//    error_y = target - (float)Pixel_y;  /* 俯仰 */

//    out_x = Kp * error_x + Kd * (error_x - error_x_last) / dt;
//    out_y = Kp * error_y + Kd * (error_y - error_y_last) / dt;

//    error_x_last = error_x;
//    error_y_last = error_y;

//    //输出限幅
//    if (out_x >  3.0f) out_x =  3.0f;
//    if (out_x < -3.0f) out_x = -3.0f;
//    if (out_y >  3.0f) out_y =  3.0f;
//    if (out_y < -3.0f) out_y = -3.0f;
//		
//		//死区
//		if(error_x > -4 && error_x <  4) error_x = 0;
//		if(error_y > -4 && error_y <  4) error_y = 0;

//    /* 偏航 4340：速度模式*/
//    DM_CAN_Motor_Velocity_Rad(&DM_Motor_4340_StructData, out_x);

//    /* 俯仰 4310：位置模式*/
//    target_pit += out_y * dt;
//    pit_limit   = 35.0f * PI / 180.0f;
//    if (target_pit >  pit_limit) target_pit =  pit_limit;
//    if (target_pit < -pit_limit) target_pit = -pit_limit;
//    DM_CAN_Motor_Rotation_Rad(&DM_Motor_4310_StructData, target_pit);
//		Frame_OK = 0;
//}
/*-------------------------- 像素pid追踪目标--------------------------------*/

float flag;
float out1;
float out2;

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
	.Ki = 0.02f,
	.Kd = 1.0f,
	
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
	float PID_Gyro_f_raw[3];
	memcpy(PID_Gyro_f_raw, DM_Motor_IMU_StructData.Gyro_f_raw, sizeof(PID_Gyro_f_raw));
	float Dist_Feedforward = -PID_Gyro_f_raw[0];// * Feedforward_K;
	
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
	float PID_Gyro_f_raw[3];
	memcpy(PID_Gyro_f_raw, DM_Motor_IMU_StructData.Gyro_f_raw, sizeof(PID_Gyro_f_raw));
	float Dist_Feedforward = PID_Gyro_f_raw[2];// * Feedforward_K;
	
	PID_Update(ay);
	
	DM_Motor_4340_StructData.Position =ay ->Out;// + Dist_Feedforward;
	
	flag = DM_Motor_4310_StructData.Position;
	out1 = ay ->Out;
	out2 = Dist_Feedforward;
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

			switch(flag)
			{
				case 0:
			 {
					if (byte == 0x59 && flag == 0)//等待包头第一个字节
					{
						flag = 1;
					}
					else
						flag = 0;
					break;
				}
			 case 1:
			 {
					if(byte == 0x53 && flag == 1)//等待包头第二个字节
					{
						flag = 2;
					}
					else
						flag = 0;
					break;
				}
			 case 2:
			 {
					if(byte == 0x41 && flag == 2)//等待包头第三个字节
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
						flag = 0;
						rx_done = 1;
					}
					break;
				}
			}
   }
}



