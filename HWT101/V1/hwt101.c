#include "hwt101.h"
#include "string.h"

#define HWT101_RXLEN				(22)
#define HWT101_RXLEN_2				(44)

HWT101_TypeDef HWT101_Open(UART_HandleTypeDef* huart) {
	HWT101_TypeDef tmp;
	memset(&tmp, 0, sizeof(HWT101_TypeDef));
	tmp.huart = huart;
	return tmp;
}

void HWT101_Enable(HWT101_TypeDef* H) {
	UART_HandleTypeDef* huart = H->huart;

	/* 使能DMA串口接收 */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_DMA_DISABLE(huart->hdmarx);
	while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
		__HAL_DMA_DISABLE(huart->hdmarx);
	}
	huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
	huart->hdmarx->Instance->M0AR = (uint32_t)(H->rx_buf1);
	huart->hdmarx->Instance->M1AR = (uint32_t)(H->rx_buf2);
	huart->hdmarx->Instance->NDTR = HWT101_RXLEN_2;
	SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
	__HAL_DMA_ENABLE(huart->hdmarx);
}

__weak void HWT101_Callback(HWT101_TypeDef* hwt) {

}

void HWT101_RxUpdate(HWT101_TypeDef* H) {

	/* 数据接收完成 */
	if (H->huart->Instance->SR & UART_FLAG_IDLE) {

		static uint16_t this_time_rx_len = 0;
		static UART_HandleTypeDef* huart;
		huart = H->huart;

		__HAL_UART_CLEAR_PEFLAG(H->huart);


		if ((H->huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {

			__HAL_DMA_DISABLE(H->huart->hdmarx);

			this_time_rx_len = WT101_RXLEN_2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = HWT101_RXLEN_2;
			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == HWT101_RXLEN) {
				H->wx = ((H->rx_buf2[3] << 8) | H->rx_buf2[2]) / 32768.f * 2000.f;
				H->wy = ((H->rx_buf2[5] << 8) | H->rx_buf2[4]) / 32768.f * 2000.f;
				H->wz = ((H->rx_buf2[7] << 8) | H->rx_buf2[6]) / 32768.f * 2000.f;
				H->yaw = ((H->rx_buf2[18] << 8) | H->rx_buf2[17]) / 32768.f * 180.f;
				if (H->yaw > 180.f) H->yaw -= 360.f;
			}
		}

		else {
			__HAL_DMA_DISABLE(H->huart->hdmarx);

			this_time_rx_len = HWT101_RXLEN_2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = HWT101_RXLEN_2;
			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == HWT101_RXLEN) {
				H->wx = ((H->rx_buf2[3] << 8) | H->rx_buf2[2]) / 32768.f * 2000.f;
				H->wy = ((H->rx_buf2[5] << 8) | H->rx_buf2[4]) / 32768.f * 2000.f;
				H->wz = ((H->rx_buf2[7] << 8) | H->rx_buf2[6]) / 32768.f * 2000.f;
				H->yaw = ((H->rx_buf2[18] << 8) | H->rx_buf2[17]) / 32768.f * 180.f;
				if (H->yaw > 180.f) H->yaw -= 360.f;
			}
		}
		/* 接收到一帧数据回调 */
		HWT101_Callback(H);
	}
}

