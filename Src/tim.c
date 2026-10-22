/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include <math.h>
#include "Gimbal_Track.h"
#include "DM_BLDC_Motor.h"
#include "DM_IMU_L1_V1.0.h"
#include "main.h"
#include "UART_Data.h"

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*∂® ±∆˜÷–∂œ∫Ø ˝*/
float flag_timex = 0;
uint8_t flag_statex = 0;
uint8_t flag_state_Begx = 0;

float flag_timey = 0;
uint8_t flag_statey = 0;
uint8_t flag_state_Begy = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		static uint32_t cnt;
		cnt++;
		if(cnt % 5 == 0)
		{
			Gimbal_Track_PID_X(&Gimbal_track_PID_X_Data, flag_timex);
			Gimbal_Track_PID_Y(&Gimbal_track_PID_Y_Data, flag_timey);
		}
		
				/*y÷·À≤º‰“∆∂Ø≤‚ ‘*/
		if(cnt % 450 == 0 && flag_state_Begy == 0)
		{
			if(flag_statey == 0)
			{
				flag_timey = 0.0f;
				flag_statey = 1;
			}
			else if(flag_statey == 1)
			{
				flag_timey = 0.5f;
				flag_statey = 2;
			}
			else if(flag_statey == 2)
			{
				flag_timey = 0.0f;
				flag_statey = 3;
			}
			else if(flag_statey == 3)
			{
				flag_timey = -0.5f;
				flag_statey = 4;
			}
			else if(flag_statey == 4)
			{
				flag_timey = 0.0f;
				flag_statey = 0;
				flag_state_Begy = 1;
			}
		}
		
		/*y÷·∆Ωª¨“∆∂Ø≤‚ ‘*/
		if(cnt % 10 == 0 && flag_state_Begy == 1)
		{
			if(flag_timey <= 0.5f && flag_statey == 0)
			{
				flag_timey += 0.001f;
				if(flag_timey >= 0.5f)
				{
					flag_statey = 1;
				}
			}
			else if(flag_timey > 0.0f && flag_statey == 1)
			{
				flag_timey -= 0.001f;
				if(flag_timey >= 0 || flag_timey <= 0)
				{
					flag_statey = 2;
				}
			}
			else if((flag_timey >= -0.5f ) && flag_statey == 2)
			{
				flag_timey -= 0.001f;
				if(flag_timey <= -0.5)
				{
					flag_statey = 3;
				}
			}
			else if(flag_timey < 0.0f && flag_statey == 3)
			{
				flag_timey += 0.001f;
				if(flag_timey >= 0)
				{
					flag_statey = 0;
					flag_state_Begy = 0;
				}
			}
		}
		
		/*x÷·À≤º‰“∆∂Ø≤‚ ‘*/
		if(cnt % 5000 == 0 && flag_state_Begx == 0)
		{
			if(flag_statex == 0)
			{
				flag_timex = 0.0f;
				flag_statex = 1;
			}
			else if(flag_statex == 1)
			{
				flag_timex = 0.5f;
				flag_statex = 2;
			}
			else if(flag_statex == 2)
			{
				flag_timex = 0.0f;
				flag_statex = 3;
			}
			else if(flag_statex == 3)
			{
				flag_timex = -0.5f;
				flag_statex = 4;
			}
			else if(flag_statex == 4)
			{
				flag_timex = 0.0f;
				flag_statex = 0;
				flag_state_Begx = 0;
			}
		}
			
			
		/*x÷·∆Ωª¨“∆∂Ø≤‚ ‘*/
		if(cnt % 10 == 0 && flag_state_Begx == 1)
		{
			if(flag_timex <= 0.5f && flag_statex == 0)
			{
				flag_timex += 0.001f;
				if(flag_timex >= 0.5f)
				{
					flag_statex = 1;
				}
			}
			else if(flag_timex > 0.0f && flag_statex == 1)
			{
				flag_timex -= 0.001f;
				if(flag_timex >= 0 || flag_timex <= 0)
				{
					flag_statex = 2;
				}
			}
			else if((flag_timex >= -0.5f ) && flag_statex == 2)
			{
				flag_timex -= 0.001f;
				if(flag_timex <= -0.5)
				{
					flag_statex = 3;
				}
			}
			else if(flag_timex < 0.0f && flag_statex == 3)
			{
				flag_timex += 0.001f;
				if(flag_timex >= 0)
				{
					flag_statex = 0;
					flag_state_Begx = 0;
				}
			}
		}

	}//if(htim->Instance == TIM1)≈–∂œŒ≤¿®∫≈
}//1ms÷–∂œ∫Ø ˝Œ≤¿®∫≈

/* USER CODE END 1 */

