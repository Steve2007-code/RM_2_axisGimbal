#include "DM_IMU_L1_V1.0.h"
#include "can.h"

DM_IMU_Data DM_Motor_IMU_StructData;//达妙IMU参数定义结构体

/*CAN发送读取IMU数据帧包定义*/
uint8_t Red_acc_raw[8] = {0xCC, Acceleration_data, 0x00, 0xDD, 0x00, 0x00, 0x00, 0x00};//加速度
uint8_t Red_Gyro_raw[8] = {0xCC, Angular_velocity_data, 0x00, 0xDD, 0x00, 0x00, 0x00, 0x00};//角速度
uint8_t Red_Euler_raw[8] = {0xCC, Euler_angle_data, 0x00, 0xDD, 0x00, 0x00, 0x00, 0x00};//欧拉角
uint8_t Red_Quaternion[8] = {0xCC, Quaternion_data, 0x00, 0xDD, 0x00, 0x00, 0x00, 0x00};//四元数


/**
  * @brief  浮点数转换为无符号整数函数
  * @param  x_float 要转换的浮点数 x_min 映射的最小值 x_max 映射的最大值 bits转换长度
  * @note		无
  * @retval 转换转换的无符号整型数
  */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  无符号整数转换为浮点数函数
  * @param  x_int 要转换的整数 x_min 映射的最小值 x_max 映射的最大值 bits转换长度
  * @note		无
  * @retval 转换转换的浮点数
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/* converts unsigned int to float, given range and number of bits */
float span = x_max - x_min;
float offset = x_min;
return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/*--------------------将CAN发来的原始角度数据转换位浮点数--------------------*/
/**
  * @brief  加速度解析
  * @param  *ax IMU数据结构体
  * @note		无
  * @retval 无
  */
void DM_IMU_UpdateAccel(DM_IMU_Data *ax)
{
	uint16_t accel[3];
	
	accel[0]=ax->acc_raw[3]<<8|ax->acc_raw[2];
	accel[1]=ax->acc_raw[5]<<8|ax->acc_raw[4];
	accel[2]=ax->acc_raw[7]<<8|ax->acc_raw[6];
	
	ax->acc_f_raw[0] = uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	ax->acc_f_raw[1] = uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	ax->acc_f_raw[2] = uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
}
/**
  * @brief  角速度解析
  * @param  *ax IMU数据结构体
  * @note		无
  * @retval 无
  */
void DM_IMU_UpdateGyro(DM_IMU_Data *ax)
{
	uint16_t gyro[3];
	gyro[0]=ax->Gyro_raw[3]<<8|ax->Gyro_raw[2];
	gyro[1]=ax->Gyro_raw[5]<<8|ax->Gyro_raw[4];
	gyro[2]=ax->Gyro_raw[7]<<8|ax->Gyro_raw[6];
	
	ax->Gyro_f_raw[0] = uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	ax->Gyro_f_raw[1] = uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	ax->Gyro_f_raw[2] = uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}
/**
  * @brief  欧拉角解析
  * @param  *ax IMU数据结构体
  * @note		无
  * @retval 无
  */
void DM_IMU_UpdateEuler(DM_IMU_Data *ax)
{
	int euler[3];
	euler[0]=ax->Euler_raw[3]<<8|ax->Euler_raw[2];
	euler[1]=ax->Euler_raw[5]<<8|ax->Euler_raw[4];
	euler[2]=ax->Euler_raw[7]<<8|ax->Euler_raw[6];
	
	ax->Euler_f_pitch = uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	ax->Euler_f_yaw = uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	ax->Euler_f_roll = uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}
/**
  * @brief  四元数解析
  * @param  *ax IMU数据结构体
  * @note		无
  * @retval 无
  */
void DM_IMU_UpdateQuaternion(DM_IMU_Data *ax)
{
	int w = ax ->Quaternion[1]<<6| ((ax ->Quaternion[2]&0xF8)>>2);
	int x = (ax ->Quaternion[2]&0x03)<<12|(ax ->Quaternion[3]<<4)|((ax ->Quaternion[4]&0xF0)>>4);
	int y = (ax ->Quaternion[4]&0x0F)<<10|(ax ->Quaternion[5]<<2)|(ax ->Quaternion[6]&0xC0)>>6;
	int z = (ax ->Quaternion[6]&0x3F)<<8|ax ->Quaternion[7];
	
	ax ->Quaternion_f[0] = uint_to_float(w,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
	ax ->Quaternion_f[1] = uint_to_float(x,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
	ax ->Quaternion_f[2] = uint_to_float(y,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
	ax ->Quaternion_f[3] = uint_to_float(z,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
}
/**
  * @brief  解收IMU数据解析分配函数
  * @param  *pData CAN接收到的数据
  * @note		无
  * @retval 无
  */
void IMU_UpdateData(uint8_t* pData)
{
	switch(pData[0])
	{
		case Acceleration_data/* 0x01 */:
			memcpy(DM_Motor_IMU_StructData.acc_raw, pData, 8);
      DM_IMU_UpdateAccel(&DM_Motor_IMU_StructData);
			break;
		case Angular_velocity_data/* 0x02 */:
			memcpy(DM_Motor_IMU_StructData.Gyro_raw, pData, 8);
      DM_IMU_UpdateGyro(&DM_Motor_IMU_StructData);
			break;
		case Euler_angle_data/* 0x03 */:
			memcpy(DM_Motor_IMU_StructData.Euler_raw, pData, 8);
      DM_IMU_UpdateEuler(&DM_Motor_IMU_StructData);
			break;
		case Quaternion_data/* 0x04 */:
			 memcpy(DM_Motor_IMU_StructData.Quaternion, pData, 8);
       DM_IMU_UpdateQuaternion(&DM_Motor_IMU_StructData);
			break;
	}
}
/*--------------------将CAN发来的原始角度数据转换位浮点数--------------------*/

/**
  * @brief  达妙IMU初始化
  * @param  无
  * @note   无
  * @retval 无
  */
void DM_IMU_Init(void)
{
	DM_Motor_IMU_StructData.hcan = &hcan2;
	DM_Motor_IMU_StructData.ID = DM_Motor_IMU_ID;
	DM_Motor_IMU_StructData.BackID = DM_Motor_IMU_BackID;
}

/**
  * @brief  读取IMU加速度/角速度/欧拉角/四元数数据
  * @param  *ax IMU数据结构体 DM_Motor_IMU_StructData - Data 需要获得的角数据值可以是 
	*	@param	Acceleration_data 加速度 Angular_velocity_data 角速度
	*	@param	Euler_angle_data 欧拉角 Quaternion_data 四元数
  * @note   无
  * @retval CAN发送状态
  */
HAL_StatusTypeDef Red_DM_IMU_raw(DM_IMU_Data *ax, uint8_t Data)
{
	if(Data == Acceleration_data)//读取加速度
	{
		return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Red_acc_raw, 8);
	}
	else if(Data == Angular_velocity_data)//读取角速度
	{
		return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Red_Gyro_raw, 8);
	}
	else if(Data == Euler_angle_data)//读取欧拉角
	{
		return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Red_Euler_raw, 8);
	}
	else if(Data == Quaternion_data)//读取四元数
	{
		return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Red_Quaternion, 8);
	}
	return HAL_ERROR; 
}

/**
  * @brief  单次函数一键读取IMU加速度/角速度/欧拉角/四元数数据
  * @param  无
  * @note   无
  * @retval CAN发送状态
  */
//读取加速度数据
HAL_StatusTypeDef Red_DM_IMU_accData(void)
{
	return Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Acceleration_data);
}
//读取角速度数据
HAL_StatusTypeDef Red_DM_IMU_GyroData(void)
{
	return Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Angular_velocity_data);
}
//读取欧拉角数据
HAL_StatusTypeDef Red_DM_IMU_EulerData(void)
{
	return Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Euler_angle_data);
}
//读取四元数数据
HAL_StatusTypeDef Red_DM_IMU_Quaternion(void)
{
	return Red_DM_IMU_raw(&DM_Motor_IMU_StructData, Quaternion_data);
}

