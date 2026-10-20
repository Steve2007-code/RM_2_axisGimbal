#ifndef __DM_IMU_L1_V1_0_H__
#define __DM_IMU_L1_V1_0_H__

#include "main.h"
#include <string.h>

/*-------------------------------------------帧定义------------------------------------------//
请求帧格式
帧格式:数据帧 DLC:8
帧类型:标准帧 ID:上位机设置的 CAN_ID
┌────┬────┬────┬────┬────┬────┬────┬─────┬────┐
│ 数据域 │ DATA[0]│ DATA[1]│ DATA[2]│ DATA[3]│DATA[4] │DATA[5] │ DATA[6]  │ DATA[7]│
├────┼────┼────┼────┼────┼────┼────┴─────┴────┤
│  内容  │   CC   │   RID  │  读/写 │   DD   │        │             数据             │
└────┴────┴────┴────┴────┴────┴───────────────┘
应答帧格式
帧格式:数据帧 DLC:8
帧类型:标准帧 ID:上位机设置的 MST_ID
┌────┬────┬────┬────┬────┬────┬────┬─────┬────┐
│ 数据域 │ DATA[0]│ DATA[1]│ DATA[2]│ DATA[3]│DATA[4] │DATA[5] │ DATA[6]  │ DATA[7]│
├────┼────┼────┼────┼────┼────┴────┴─────┴────┤
│  内容  │   CC   │   RID  │   DD   │ 应答码 │                应答数据                │
└────┴────┴────┴────┴────┴────────────────────┘
//应答码Ack ID[HEX] 00-应答成功 01-寄存器不存在 02-数据无效 03-操作失败

CAN通信下的数据帧格式查看《达妙科技DM-IMU-L1六轴IMU模块使用说明书V1.2》
----------------------------------------------------------------------------------------------*/

//------------------------------IMU寄存器说明------------------------------//
//---------名称------HEX------------------------------------说明-读写性---//
#define Restart_IMU 0x00																//重启IMU  W
#define Acceleration_data 0x01													//加速度数据  R
#define Angular_velocity_data 0x02											//角速度数据  R
#define Euler_angle_data 0x03														//欧拉角数据  R
#define Quaternion_data 0x04														//四元数数据  R
#define Set_the_angle_to_zero 0x05											//角度置零  W
#define Calibration_of_six_surfaces 0x06								//加计六面校准  W
#define Gyroscopic_static_calibration 0x07							//陀螺静态校准  W
#define Magnetic_measurement_ellipsoid_calibration 0x08	//磁计椭球校准  W
#define Switch_communication_mode 0x09									//切换通信模式  RW
#define Set_the_interval_for_automatic_sending 0x0A			//设置主动发送间隔  RW
#define Switch_between_active_and_passive_modes 0x0B		//切换主被动模式  RW
#define Modify_the_baud_rate 0x0C												//修改波特率  RW
#define CAN_ID 0x0D																			//CAN_ID  RW
#define MST_ID 0x0E																			//MST_ID  RW
#define Output_data_selection 0x0F											//输出数据选择  RW(固件当前不支持修改)
#define Save_parameters 0xFE														//保存参数  W
#define factory_data_reset 0xFF													//恢复出厂设置  W

//------------------------------线性映射表------------------------------//
#define ACCEL_CAN_MAX 235.2f
#define ACCEL_CAN_MIN -235.2f
#define GYRO_CAN_MAX 34.88f
#define GYRO_CAN_MIN -34.88f
#define PITCH_CAN_MAX 90.0f
#define PITCH_CAN_MIN -90.0f
#define ROLL_CAN_MAX 180.0f
#define ROLL_CAN_MIN -180.0f
#define YAW_CAN_MAX 180.0f
#define YAW_CAN_MIN -180.0f
#define Quaternion_CAN_MIN -1.0f
#define Quaternion_CAN_MAX 1.0f


typedef struct
{
	CAN_HandleTypeDef *hcan;  //IMU所接的CAN线
	uint16_t ID;							//IMU ID
	uint16_t BackID;					//IMU回传ID
	uint8_t acc_raw[8];				//加速度原始数据
	uint8_t Gyro_raw[8];			//角速度原始数据
	uint8_t Euler_raw[8];			//欧拉角原始数据
	uint8_t Quaternion[8];		//四元数原始数据
	float acc_f_raw[3];				//加速度浮点数据
	float Gyro_f_raw[3];			//角速度浮点数据
	float Euler_f_pitch;			//欧拉角浮点数据 y
	float Euler_f_yaw;				//欧拉角浮点数据 z
	float Euler_f_roll;				//欧拉角浮点数据 x
	float Quaternion_f[4];		//四元数浮点数据
}DM_IMU_Data;

extern DM_IMU_Data DM_Motor_IMU_StructData;//达妙参数定义结构体

#define DM_Motor_IMU_ID 0x03 //IMUID
#define DM_Motor_IMU_BackID 0x13 //IMUID


/*----------外部可调用函数声明----------*/
//达妙IMU初始化
void DM_IMU_Init(void);
//读取加速度数据
HAL_StatusTypeDef Red_DM_IMU_Data(void);
//读取IMU加速度/角速度/欧拉角/四元数数据
HAL_StatusTypeDef Red_DM_IMU_raw(DM_IMU_Data *ax, uint8_t Data);
//将CAN发来的原始角度数据转换位浮点数
void IMU_UpdateData(uint8_t* pData);

#endif
