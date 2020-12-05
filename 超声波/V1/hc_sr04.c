/*
 * hc_sr04.c
 *
 *  Created on: 2020年11月11日
 *      Author: 李文昊
 */

#include "hc_sr04.h"
#include "string.h"

SR04_TypeDef SR04_Open(TIM_HandleTypeDef* htim, uint32_t tim_channel, GPIO_TypeDef* trig_port, uint16_t trig_pin) {
	SR04_TypeDef S;
	memset(&S, 0, sizeof(SR04_TypeDef));
	S.htim = htim;
	S.channel = tim_channel;
	S.port = trig_port;
	S.pin = trig_pin;
	if (tim_channel == TIM_CHANNEL_1) S.active_channel = HAL_TIM_ACTIVE_CHANNEL_1;
	else if (tim_channel == TIM_CHANNEL_2) S.active_channel = HAL_TIM_ACTIVE_CHANNEL_2;
	else if (tim_channel == TIM_CHANNEL_3) S.active_channel = HAL_TIM_ACTIVE_CHANNEL_3;
	else if (tim_channel == TIM_CHANNEL_4) S.active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
	HAL_TIM_Base_Start_IT(htim);
	HAL_TIM_IC_Start_IT(htim, tim_channel);
	return S;
}

static void delay_us(uint32_t us) {
	uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
	while (delay--) ;
}

void SR04_Maintask(SR04_TypeDef* S) {
	HAL_GPIO_WritePin(S->port, S->pin, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin(S->port, S->pin, GPIO_PIN_RESET);
}

__weak void SR04_Callback(SR04_TypeDef* sr04) {

}

void SR04_IcUpdate(SR04_TypeDef* S, TIM_HandleTypeDef* htim) {
	if (htim == S->htim) {
		if (htim->Channel == S->active_channel) {
			if (S->edge == 0) {
				S->rising_time = HAL_TIM_ReadCapturedValue(htim, S->channel);
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, S->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(htim, S->channel);
				S->edge = 1;
			}
			else if (S->edge == 1) {
				static uint32_t high_time;
				static uint32_t falling_time;
				falling_time = HAL_TIM_ReadCapturedValue(htim, S->channel);
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, S->channel, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(htim, S->channel);
				high_time = falling_time < S->rising_time ? falling_time + 0xffff - S->rising_time + 1 : falling_time - S->rising_time;
				S->dist = (double)high_time / 1000000 * 170;
				S->edge = 0;
			}
		}
	}
}
