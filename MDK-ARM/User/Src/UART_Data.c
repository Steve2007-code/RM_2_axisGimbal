#include "UART_Data.h"

/*串口发/接数据定义*/
//volatile uint8_t UART1_TX_Buf[64];//发送缓冲区
uint8_t UART1_RX_Buf[1];//中断接收单字节临时缓存
volatile uint8_t UART1_RX_Fifo[128];//接收环形 FIFO 缓冲区
volatile uint16_t UART1_RX_idx = 0;//串口1写索引
volatile uint16_t UART1_RX_Rd = 0;//串口1读索引
uint8_t UART1_TX_Busy = 0;//串口1发送忙标志

//volatile uint8_t UART6_TX_Buf[64];//发送缓冲区
uint8_t UART6_RX_Buf[1];//中断接收单字节临时缓存
volatile uint8_t UART6_RX_Fifo[128];//接收环形 FIFO 缓冲区
volatile uint16_t UART6_RX_idx = 0;//串口写6索引
volatile uint16_t UART6_RX_Rd = 0;//串口6读索引
uint8_t UART6_TX_Busy = 0;//串口6发送忙标志

static uint8_t UART6_To_UART1_Buf[64];//UART6转发到UART1数据转发暂存区

/**
  * @brief  串口初始化
  * @param  无
  * @note		无
  * @retval 启动一次非阻塞接收
  */
void UART_Init(void)
{
	HAL_UART_Receive_IT(&huart1, UART1_RX_Buf, 1);
	HAL_UART_Receive_IT(&huart6, UART6_RX_Buf, 1);
}

/**
  * @brief  串口发送数据
  * @param  huart 串口句柄 data 数据 len 数据长度
  * @note		无
  * @retval 发送状态
  */
HAL_StatusTypeDef UART_Send_NonBlock(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if(huart->Instance == USART1 && UART1_TX_Busy)
        return HAL_BUSY;
    if(huart->Instance == USART6 && UART6_TX_Busy)
        return HAL_BUSY;

    HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(huart, data, len);
    if(ret == HAL_OK)
    {
        if(huart->Instance == USART1) UART1_TX_Busy = 1;
        if(huart->Instance == USART6) UART6_TX_Busy = 1;
    }
    return ret;
}

/**
  * @brief  发送完成回调函数
  * @param  无
  * @note		无
  * @retval 发送完成之后置标志位
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        UART1_TX_Busy = 0;
    }
		if(huart->Instance == USART6)
    {
        UART6_TX_Busy = 0;
    }
}

/**
  * @brief  接收回调函数
  * @param  无
  * @note		无
  * @retval 接收到数据之后数据放入缓冲区
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        UART1_RX_Fifo[UART1_RX_idx++] = UART1_RX_Buf[0];
        if(UART1_RX_idx >= 128) UART1_RX_idx = 0;
        HAL_UART_Receive_IT(&huart1, UART1_RX_Buf, 1);
    }
		if(huart->Instance == USART6)
    {
        UART6_RX_Fifo[UART6_RX_idx++] = UART6_RX_Buf[0];
        if(UART6_RX_idx >= 128) UART6_RX_idx = 0;
        HAL_UART_Receive_IT(&huart6, UART6_RX_Buf, 1);
    }
}

/**
  * @brief  UART 出错后重新挂起中断接收
  * @param  huart 触发错误的串口句柄
  * @note		出错之后中断重启接收
  * @retval 无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
        HAL_UART_Receive_IT(&huart6, UART6_RX_Buf, 1);
    if (huart->Instance == USART1)
        HAL_UART_Receive_IT(&huart1, UART1_RX_Buf, 1);
}

/**
  * @brief  串口6收到数据向串口1转发
  * @param  无
  * @note		无
  * @retval 发送状态
  */
HAL_StatusTypeDef UART_Send_UART6_To_UART1(void)
{
	uint16_t n = 0;

    if (UART1_TX_Busy) return HAL_BUSY;

    while ((UART6_RX_idx != UART6_RX_Rd) && (n < sizeof(UART6_To_UART1_Buf))) {
        UART6_To_UART1_Buf[n++] = UART6_RX_Fifo[UART6_RX_Rd];
        UART6_RX_Rd = (UART6_RX_Rd + 1) % 128;
    }
    if (n == 0) return HAL_TIMEOUT;
    return UART_Send_NonBlock(&huart1, UART6_To_UART1_Buf, n);
}
