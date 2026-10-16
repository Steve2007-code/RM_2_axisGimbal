#ifndef __DM_IMU_L1_V1_0_H__
#define __DM_IMU_L1_V1_0_H__

#include "main.h"

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


typedef struct
{
	int16_t acc_raw[3];//加速度数据
	int16_t  Gyro_raw[3];//角速度数据
	int16_t  Pitch[3];//欧拉角数据
	int16_t  Quaternion[3];//四元数数据
}DM_IMU_Data;

HAL_StatusTypeDef Red_DM_IMU_Data(void);

#endif
