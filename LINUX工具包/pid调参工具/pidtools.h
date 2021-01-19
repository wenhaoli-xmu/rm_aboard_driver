#ifndef __PID_TOOLS_H__
#define __PID_TOOLS_H__


#include "stm32f4xx_hal.h"

#define PID_TOOLS_FLOAT				(0)
#define PID_TOOLS_INT				(1)

typedef struct
{
	UART_HandleTypeDef * huart;
	uint8_t data_type;

	void * refer_add;
	void * trace_add;
	float params[5];

	char tmp[30];

	/* 接收字节的长度 */
	uint8_t rx_length;

	/* DMA数据缓冲 */
	uint8_t rx_data1[40];
	uint8_t rx_data2[40];

	/* 保存目标地址 */
	uint8_t rx_addr[40];

} PidTools_t;


//用户调用

/* 打开一个PidTools */
void PidTools_Open( PidTools_t * p, UART_HandleTypeDef * huart );

/* 放在it.c相对应的串口文件中 */
void PidTools_RxUpdate( PidTools_t * p );

/* 待发送数据的地址 */
void PidTools_SetTxDataAdd( PidTools_t * p, void * refer_data, void * trace_data, uint8_t data_type );

/* 接收到一帧数据回调函数 */
void PidTools_Callback( PidTools_t * p );

/* 获取PID和alpha的参数 */
float * PidTools_GetParams( PidTools_t * p );

/* 主程序，放在freeRTOS中不断运行 */
void PidTools_MainTask( PidTools_t * p );

#endif
