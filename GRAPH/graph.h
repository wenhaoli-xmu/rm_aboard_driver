#ifndef __GRAPH_H
#define __GRAPH_H


#include "stm32f4xx_hal.h"


#define GRAPH_READY				0
#define GRAPH_BUSY				1
#define GRAPH_IDLE				2
#define GRAPH_DONE				3


/* 设置GRAPH使用的UART */
void GRAPH_Open(UART_HandleTypeDef* huart, float* p, float* i, float* d, int* data);

/* 发布一个数据点 */
void GRAPH_Pub(int data);

/* 放在HAL_UART_RxCpltCallback()中 */
void GRAPH_RxUpdate(UART_HandleTypeDef* huart);


#endif
