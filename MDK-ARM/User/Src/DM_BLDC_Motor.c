#include "DM_BLDC_Motor.h"
#include "can.h"

/*CAN数据帧定义*/
uint8_t DM_Mototr_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};//电机使能
uint8_t DM_Mototr_Disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};//电机失能

/*达妙电机MIT模式下的结构体定义*/
DM_Motor_Data DM_Motor_4310_StructData;//4310参数定义结构体
DM_Motor_Data DM_Motor_4340_StructData;//4340参数定义结构体

/*
  * @brief  DM电机结构体初始化
  * @param  无
  * @note		代码初始化时务必进行初始化
  * @retval 无
  */
void DM_CAN_Motor_Init(void)
{
	/*4310数据初始化*/
	DM_Motor_4310_StructData.hcan = &hcan1;
	DM_Motor_4310_StructData.ID = DM_Motor_4310_ID;
	DM_Motor_4310_StructData.Kd = 2;
	DM_Motor_4310_StructData.Kd_max = 5;
	DM_Motor_4310_StructData.Kd_min = 0;
	DM_Motor_4310_StructData.Kp = 20;
	DM_Motor_4310_StructData.Kp_max = 500;
	DM_Motor_4310_StructData.Kp_min = 0;
	DM_Motor_4310_StructData.Position = 0;
	DM_Motor_4310_StructData.Position_max = 12.5;
	DM_Motor_4310_StructData.Position_min = -12.5;
	DM_Motor_4310_StructData.Torque = 0;
	DM_Motor_4310_StructData.Torque_max = 10;
	DM_Motor_4310_StructData.Torque_min = -10;
	DM_Motor_4310_StructData.Velocity = 0;
	DM_Motor_4310_StructData.Velocity_max = 30;
	DM_Motor_4310_StructData.Velocity_min = -30;
	/*4340数据初始化*/
	DM_Motor_4340_StructData.hcan = &hcan1;
	DM_Motor_4340_StructData.ID = DM_Motor_4340_ID;
	DM_Motor_4340_StructData.Kd = 2;
	DM_Motor_4340_StructData.Kd_max = 5;
	DM_Motor_4340_StructData.Kd_min = 0;
	DM_Motor_4340_StructData.Kp = 20;
	DM_Motor_4340_StructData.Kp_max = 500;
	DM_Motor_4340_StructData.Kp_min = 0;
	DM_Motor_4340_StructData.Position = 0;
	DM_Motor_4340_StructData.Position_max = 1000;
	DM_Motor_4340_StructData.Position_min = -1000;
	DM_Motor_4340_StructData.Torque = 0;
	DM_Motor_4340_StructData.Torque_max = 28;
	DM_Motor_4340_StructData.Torque_min = -28;
	DM_Motor_4340_StructData.Velocity = 0;
	DM_Motor_4340_StructData.Velocity_max = 10;
	DM_Motor_4340_StructData.Velocity_min = -10;
}

/*
  * @brief  CAN发送一帧数据给电机
  * @param  *hcan CAN外设句柄 - id 电机id 可以是DM_Motor_4340或DM_Motor_4310 - *data:8位数据帧 - len 数据帧长度
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Send_Data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len)
{
	return CAN_Send_Data_Frame(hcan, id, data, len);
}

/*
  * @brief  CAM发送电机使/失能
  * @param  id 电机id，可以是 DM_Motor_4340_ID 或 DM_Motor_4310_ID DATA DM_Mototr_Enable DM_Mototr_Disable
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_SetEnable(uint16_t id, uint8_t *data)
{
	if(data[7] == 0xFC)//使能
	{
		return CAN_Send_Data_Frame(&hcan1, id, DM_Mototr_Enable, 8);
	}
	else if(data[7] == 0xFD)//失能
	{
		return CAN_Send_Data_Frame(&hcan1, id, DM_Mototr_Disable, 8);
	}
}

/*
  * @brief  int16转换16位无符号数
  * @param  x 目标值 xmin 可调整的最小值 xmax 可调整的最大值
  * @note		字段映射为整数
  * @retval 无
  */
static uint16_t DM_Motor_16_Data_Frame(float x, float xmin, float xmax)
{
	if(x <= xmin){x = xmin;}
	if(x >= xmax){x = xmax;}
	uint16_t value = (uint16_t)((x - xmin) / (xmax - xmin) * 65535.0f + 0.5f);
	return value;
}
/*
  * @brief  int16转换12位无符号数
  * @param  x 目标值 xmin 可调整的最小值 xmax 可调整的最大值
  * @note		字段映射为整数
  * @retval 无
  */
static uint16_t DM_Motor_12_Data_Frame(float x, float xmin, float xmax)
{
	if(x <= xmin){x = xmin;}
	if(x >= xmax){x = xmax;}
	uint16_t value = (uint16_t)((x - xmin) / (xmax - xmin) * 4095.0f + 0.5f);
	return value;
}

/*
  * @brief  CAN发送电机旋转数据帧(弧度制)
  * @param  id 电机id，可以是 DM_Motor_4310_StructData 或 DM_Motor_4340_StructData
	* @param	Position 目标位置 Velocity 目标速度 Kp 比例 Kd 微分 Torque 扭矩
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Rotation_Rad(DM_Motor_Data *ax, float Position)
{
	uint8_t Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	ax ->Position = Position;
	uint16_t Position_raw  = DM_Motor_16_Data_Frame(ax ->Position, ax ->Position_min, ax ->Position_max);
	uint16_t Velocity_raw  = DM_Motor_12_Data_Frame(ax ->Velocity, ax ->Velocity_min, ax ->Velocity_max);
	uint16_t  Kp_raw  = DM_Motor_12_Data_Frame(ax ->Kp, ax ->Kp_min, ax ->Kp_max);
	uint16_t  Kd_raw = DM_Motor_12_Data_Frame(ax ->Kd, ax ->Kd_min, ax ->Kd_max);
	uint16_t  Torque_raw= DM_Motor_12_Data_Frame(ax ->Torque, ax ->Torque_min, ax ->Torque_max);
	
	Data[0] = (uint8_t)(Position_raw >> 8);
  Data[1] = (uint8_t)(Position_raw & 0xFF);
  Data[2] = (uint8_t)(Velocity_raw >> 4);
  Data[3] = (uint8_t)(((Velocity_raw & 0x0F) << 4) | (Kp_raw >> 8));
  Data[4] = (uint8_t)(Kp_raw & 0xFF);
  Data[5] = (uint8_t)(Kd_raw >> 4);
  Data[6] = (uint8_t)(((Kd_raw & 0x0F) << 4) | (Torque_raw >> 8));
  Data[7] = (uint8_t)(Torque_raw & 0xFF);
	
	return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Data, 8);
}

/*
  * @brief  CAN发送电机旋转数据帧(角度制)
  * @param  id 电机id，可以是 DM_Motor_4310_StructData 或 DM_Motor_4340_StructData
	* @param	Position 目标位置 Velocity 目标速度 Kp 比例 Kd 微分 Torque 扭矩
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Rotation_deg(DM_Motor_Data *ax, float Position)
{
	uint8_t Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	ax ->Position = Position * PI / 180.0f;
	uint16_t Position_raw  = DM_Motor_16_Data_Frame(ax ->Position, ax ->Position_min, ax ->Position_max);
	uint16_t Velocity_raw  = DM_Motor_12_Data_Frame(ax ->Velocity, ax ->Velocity_min, ax ->Velocity_max);
	uint16_t  Kp_raw  = DM_Motor_12_Data_Frame(ax ->Kp, ax ->Kp_min, ax ->Kp_max);
	uint16_t  Kd_raw = DM_Motor_12_Data_Frame(ax ->Kd, ax ->Kd_min, ax ->Kd_max);
	uint16_t  Torque_raw= DM_Motor_12_Data_Frame(ax ->Torque, ax ->Torque_min, ax ->Torque_max);
	
	Data[0] = (uint8_t)(Position_raw >> 8);
  Data[1] = (uint8_t)(Position_raw & 0xFF);
  Data[2] = (uint8_t)(Velocity_raw >> 4);
  Data[3] = (uint8_t)(((Velocity_raw & 0x0F) << 4) | (Kp_raw >> 8));
  Data[4] = (uint8_t)(Kp_raw & 0xFF);
  Data[5] = (uint8_t)(Kd_raw >> 4);
  Data[6] = (uint8_t)(((Kd_raw & 0x0F) << 4) | (Torque_raw >> 8));
  Data[7] = (uint8_t)(Torque_raw & 0xFF);
	
	return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Data, 8);
}
	