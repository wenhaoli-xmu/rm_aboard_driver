#ifndef __DR16_H
#define __DR16_H


#include "stm32f4xx_hal.h"

typedef struct {
	
	UART_HandleTypeDef* dr16_uart;
	
	uint8_t rx_data1[36];
	uint8_t rx_data2[36];
	
	/* 四个通道的数据 */
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	/* 两个开关 */
	int16_t sw1;
	int16_t sw2;
	
	/* 鼠标控制 */
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;

	/* 键盘 */
	uint16_t keyboard;

	/* 通过程序校对 */
	int16_t ceil[4];
	int16_t floor[4];
	
} DR16_TypeDef;

/* 实例化一个DR16接收机对象 */
DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart);

/* 使能DR16的DMA接收 */
void DR16_Enable(DR16_TypeDef* D);

/* 放在中断it.c中的USARTx_IRQHandler中 */
void DR16_RxUpdate(DR16_TypeDef* D);

/* 接收到一帧数据回调函数 */
void DR16_Callback(DR16_TypeDef* dr16);

/* 将接受的数据映射到区间(0, ceiling)中 */
void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling);


#endif
