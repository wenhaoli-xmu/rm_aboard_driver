/*
 * track.h
 *
 *  Created on: 2020年11月14日
 *      Author: 李文昊
 */

#ifndef TRACK_H_
#define TRACK_H_

#include "stm32f4xx_hal.h"

/* 最多可以支持的数量 */
#define TRACK_MODULE_MAX				(10)

typedef struct {

	/* 记录测量信息 */
	uint8_t state[TRACK_MODULE_MAX];
	uint8_t active_level;

	/* 记录端口信息 */
	GPIO_TypeDef* port[TRACK_MODULE_MAX];
	uint16_t pin[TRACK_MODULE_MAX];
	uint8_t cnt;
} TRACK_TypeDef;



/* 用户接口 */

/* 打开一个循迹模块组 */
TRACK_TypeDef TRACK_Open();

/* 向循迹模块组中添加一个循迹模块 */
void TRACK_AppendModule(TRACK_TypeDef* T, GPIO_TypeDef* port, uint16_t pin);

/* 主任务，放在freeRtos中运行 */
void TRACK_MainTask(TRACK_TypeDef* T);


#endif /* TRACK_H_ */
