#ifndef __DM_BLDC_Motor_H__
#define __DM_BLDC_Motor_H__

#include "main.h"

extern uint8_t DM_Mototr_Enable[8];
extern uint8_t DM_Mototr_Disable[8];

#ifndef PI
#define PI 3.141592653579
#endif

HAL_StatusTypeDef DM_CAN_Motor_Send_Data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len);/*CAN发送一帧数据给电机*/

#endif
