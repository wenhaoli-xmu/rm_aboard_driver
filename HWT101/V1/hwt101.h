#ifndef __HWT101_H
#define __HWT101_H


#include "stm32f4xx_hal.h"

typedef struct {
	float wx;
	float wy;
	float wz;

	float yaw;

	uint8_t rx_buf1[44];
	uint8_t rx_buf2[44];
	uint8_t rx_cnt;
	uint8_t rx_len;

	UART_HandleTypeDef* huart;
	uint32_t sample_period;
} HWT101_TypeDef;


HWT101_TypeDef HWT101_Open(UART_HandleTypeDef* huart);

void HWT101_Enable(HWT101_TypeDef* H);

void HWT101_RxUpdate(HWT101_TypeDef* H);

void HWT101_Callback(HWT101_TypeDef* hwt);

#endif
