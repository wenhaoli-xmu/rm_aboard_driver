#include "rxd.h"
#include "stdlib.h"
#include "string.h"


RXD_TypeDef RXD_Open(UART_HandleTypeDef* huart, uint8_t* rx_addr, uint8_t rx_length) {
	RXD_TypeDef R;
	memset(&R, 0, sizeof(RXD_TypeDef));
	R.huart = huart;
	R.rx_length = rx_length;
	R.rx_addr = rx_addr;
	return R;
}

void RXD_Enable(RXD_TypeDef* R) {
	UART_HandleTypeDef* huart = R->huart;

	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_DMA_DISABLE(huart->hdmarx);
	while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
		__HAL_DMA_DISABLE(huart->hdmarx);
	}
	huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
	huart->hdmarx->Instance->M0AR = (uint32_t)(R->rx_data1);
	huart->hdmarx->Instance->M1AR = (uint32_t)(R->rx_data2);
	huart->hdmarx->Instance->NDTR = R->rx_length * 2;
	SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
	__HAL_DMA_ENABLE(huart->hdmarx);
}

__weak void RXD_Callback(RXD_TypeDef* R) {

}

void RXD_RxUpdate(RXD_TypeDef* R) {

	static uint8_t i;

	if (R->huart->Instance->SR & UART_FLAG_IDLE) {

		static uint16_t this_time_rx_len = 0;
		static UART_HandleTypeDef* huart;
		huart = R->huart;

		__HAL_UART_CLEAR_PEFLAG(huart);


		if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {

			__HAL_DMA_DISABLE(huart->hdmarx);

			this_time_rx_len = R->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = R->rx_length * 2;
			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == R->rx_length) {
				for (i = 0; i < R->rx_length; i ++) R->rx_addr[i] = R->rx_data2[i];
			}
		}

		else {
			__HAL_DMA_DISABLE(huart->hdmarx);

			this_time_rx_len = R->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = R->rx_length * 2;
			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == R->rx_length) {
				for (i = 0; i < R->rx_length; i ++) R->rx_addr[i] = R->rx_data1[i];
			}
		}

		/* 接收到一帧数据回调 */
		RXD_Callback(R);
	}
}
