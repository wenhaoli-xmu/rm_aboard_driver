#ifndef __SR04_H
#define __SR04_H


#include "stm32h7xx_hal.h"

typedef struct {

	/* 定时器以及通道信息 */
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	/* 进行信息发送的GPIO口 */
	GPIO_TypeDef* port;
	uint16_t pin;

	/* 输入捕获信息 */
	uint16_t active_channel;
	uint8_t edge;
	uint32_t rising_time;

	/* 距离 */
	double dist;

} SR04_TypeDef;


/* 用户接口 */

/* 创建一个SR04对象 */
SR04_TypeDef SR04_Open(TIM_HandleTypeDef* htim, uint32_t tim_channel, GPIO_TypeDef* trig_port, uint16_t trig_pin);

/* 放在rtos中运行，以60ms为周期运行 */
void SR04_Maintask(SR04_TypeDef* S);

/* 接收到数据回调函数 */
void SR04_Callback(SR04_TypeDef* sr04);

/* 放在HAL_TIM_IC_CaptureCallback中运行  */
void SR04_IcUpdate(SR04_TypeDef* S, TIM_HandleTypeDef* htim);


#endif
