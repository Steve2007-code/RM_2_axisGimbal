#include "DM_IMU_L1_V1.0.h"
#include "can.h"

uint8_t IMU_DATA[8] = {0xCC, 0x01, 0x00, 0xDD, 0x00, 0x00, 0x00, 0x00};

HAL_StatusTypeDef Red_DM_IMU_Data(void)
{
	CAN_Send_Data_Frame(&hcan2, 0x03, IMU_DATA, 8);
}
