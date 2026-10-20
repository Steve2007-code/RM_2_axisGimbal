#ifndef __UART_Data_H__
#define __UART_Data_H__

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

/*外部变量声明*/
extern volatile  uint8_t UART1_TX_Buf[64];//发送缓冲区
extern uint8_t UART1_RX_Buf[1];//中断接收单字节临时缓存
extern volatile uint8_t UART1_RX_Fifo[128];//接收环形 FIFO 缓冲区
extern volatile uint16_t UART1_RX_idx;//串口1写索引
extern volatile uint16_t UART1_RX_Rd;//串口1读索引
extern uint8_t UART1_TX_Busy;//串口1发送忙标志

extern volatile uint8_t UART6_TX_BBuf[64];//发送缓冲区
extern uint8_t UART6_RX_Buf[1];//中断接收单字节临时缓存
extern volatile uint8_t UART6_RX_Fifo[128];//接收环形 FIFO 缓冲区
extern volatile uint16_t UART6_RX_idx;//串口写6索引
extern volatile uint16_t UART6_RX_Rd;//串口6读索引
extern uint8_t UART6_TX_Busy;//串口6发送忙标志

/*串口初始化*/
void UART_Init(void);
/*串口发送数据*/
HAL_StatusTypeDef UART_Send_NonBlock(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
/*串口6收到数据向串口1转发*/
HAL_StatusTypeDef UART_Send_UART6_To_UART1(void);

#endif
