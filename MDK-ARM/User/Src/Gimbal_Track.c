#include "Gimbal_Track.h"
#include "UART_Data.h"
#include "DM_BLDC_Motor.h"
#include "stdio.h"

/*像素差值储存*/
int16_t Pixel_x , Pixel_y;
/*转换到一帧有效帧置 1*/
uint8_t Frame_OK = 0;
/*当前解析字符串的临时储存*/
char String_Database[24];
/*目标丢失*/
uint16_t lost_cnt = 0;

/*
  * @brief  将字符串转换为浮点数
  * @param  string 字符串组
  * @note		无
  * @retval 成功解析两个浮点数返回 1 ,否则返回0
  */
uint8_t String_To_Int16(char *string, int16_t *Pixel_x, int16_t *Pixel_y)
{
    int ret = sscanf(string, "YSC,%hd,%hd", Pixel_x, Pixel_y);
    if(ret == 2)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  串口接收数据解数据帧
  * @param  无
  * @note		无
  * @retval 无
  */
void UART6_Data_Frame(void)
{
	//串口接收数据临时中转变量
	uint8_t Flage;
	//接收数据总量记录函数
	static uint8_t Line_Flage;
	
	while(UART6_RX_idx != UART6_RX_Rd)
	{
		Flage = UART6_RX_Fifo[UART6_RX_Rd];
		UART6_RX_Rd = (UART6_RX_Rd + 1) % 128;
		
		String_Database[Line_Flage] = Flage;
		Line_Flage += 1;
		if(Line_Flage >= sizeof(String_Database)-1)
		{
			Line_Flage = 0;
		}
		
		if(Flage == '\n' || Flage == '\r')
		{
			String_Database[Line_Flage] = '\0';
			Frame_OK = String_To_Int16(String_Database, &Pixel_x, &Pixel_y);
			if(Frame_OK){ lost_cnt = 0; } else if(lost_cnt < 600){ lost_cnt++; }
			Line_Flage = 0;
		}
	}
}

/**
  * @brief  云台双轴PID计算
  * @param  无
  * @note		无
  * @retval 无
  */
void Gimbal_Track_PID(void)
{
    float Kp = 0.0080f;
    float Kd = 0.0003f;
    float dt = 0.0105f;
    float target = 0.0f;
    float error_x, error_y;
    float out_x, out_y;
    float pit_limit;

    static float error_x_last = 0.0f;
    static float error_y_last = 0.0f;
    static float target_pit   = 0.0f;   /* 俯仰目标角 rad */

    error_x = target - (float)Pixel_x;  /* 偏航 */
    error_y = target - (float)Pixel_y;  /* 俯仰 */

    out_x = Kp * error_x + Kd * (error_x - error_x_last) / dt;
    out_y = Kp * error_y + Kd * (error_y - error_y_last) / dt;

    error_x_last = error_x;
    error_y_last = error_y;

    //输出限幅
    if (out_x >  3.0f) out_x =  3.0f;
    if (out_x < -3.0f) out_x = -3.0f;
    if (out_y >  3.0f) out_y =  3.0f;
    if (out_y < -3.0f) out_y = -3.0f;
		
		//死区
		if(error_x > -4 && error_x <  4) error_x = 0;
		if(error_y > -4 && error_y <  4) error_y = 0;

    /* 偏航 4340：速度模式*/
    DM_CAN_Motor_Velocity_Rad(&DM_Motor_4340_StructData, out_x);

    /* 俯仰 4310：位置模式*/
    target_pit += out_y * dt;
    pit_limit   = 35.0f * PI / 180.0f;
    if (target_pit >  pit_limit) target_pit =  pit_limit;
    if (target_pit < -pit_limit) target_pit = -pit_limit;
    DM_CAN_Motor_Rotation_Rad(&DM_Motor_4310_StructData, target_pit);
		Frame_OK = 0;
}




