#ifndef __RXD_H
#define __RXD_H

#include "stm32f4xx_hal.h"

/* 最大接收长度是20个字节 */
#define RXD_MAX_LEN				(20)


typedef struct {
	UART_HandleTypeDef* huart;

	/* 接收字节的长度 */
	uint8_t rx_length;

	/* DMA数据缓冲 */
	uint8_t rx_data1[RXD_MAX_LEN*2];
	uint8_t rx_data2[RXD_MAX_LEN*2];

	/* 保存目标地址 */
	uint8_t* rx_addr;

} RXD_TypeDef;


/* 用户调用 */

/* 创建一个RXD对象 */
RXD_TypeDef RXD_Open(UART_HandleTypeDef* huart, uint8_t* rx_addr, uint8_t rx_length);

/* 使能DMA传输 */
void RXD_Enable(RXD_TypeDef* R);

/* 放在it.c文件中运行 */
void RXD_RxUpdate(RXD_TypeDef* R);

/* 回调函数，用户重载 */
__weak void RXD_Callback(RXD_TypeDef* rxd);

#endif
