/*
 * track.c
 *
 *  Created on: 2020年11月14日
 *      Author: 李文昊
 */

#include "track.h"
#include "string.h"

TRACK_TypeDef TRACK_Open() {
	TRACK_TypeDef T;
	memset(&T, 0, sizeof(TRACK_TypeDef));
	return T;
}

void TRACK_AppendModule(TRACK_TypeDef* T, GPIO_TypeDef* port, uint16_t pin) {
	T->port[T->cnt] = port;
	T->pin[T->cnt++] = pin;
}

void TRACK_MainTask(TRACK_TypeDef* T) {
	static uint8_t i;

	T->active_level = 0;

	for (i = 0; i < T->cnt; i ++) {
		T->state[i] = HAL_GPIO_ReadPin(T->port[i], T->pin[i]);
		if (T->state[i]) ++ T->active_level;
	}
}
