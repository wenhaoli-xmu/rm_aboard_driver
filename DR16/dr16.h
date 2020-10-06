#ifndef __DR16_H
#define __DR16_H


#include "stm32f4xx_hal.h"

typedef struct {
	
	UART_HandleTypeDef* dr16_uart;
	
	uint32_t tick;
	uint32_t thresh;
	
	uint8_t rx_data[18];
	uint8_t rx_cnt;
	uint8_t rx_buf;
	
	uint8_t done;
	
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	int16_t sw1;
	int16_t sw2;
	
} DR16_TypeDef;

/* 实例化一个DR16接收机对象 */
DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart);

/* 放在1ms周期的HAL_TIM_PeriodElapsedCallback中 */
void DR16_Update(DR16_TypeDef* D);

/* 回调函数，放在while(1)中 */
void DR16_Callback(DR16_TypeDef* D);

/* 放在HAL_UART_RxCpltCallback中 */
void DR16_RxUpdate(DR16_TypeDef* D, UART_HandleTypeDef* huart);

/* 将接受的数据映射到区间(0, ceiling)中 */
void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling);


#endif
