#include "DM_BLDC_Motor.h"
#include "can.h"

uint8_t DM_Mototr_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t DM_Mototr_Disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

/*CAN发送一帧数据给电机*/
HAL_StatusTypeDef DM_CAN_Motor_Send_Data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len)
{
	return CAN_Send_Data_Frame(hcan, id, data, len);
}
