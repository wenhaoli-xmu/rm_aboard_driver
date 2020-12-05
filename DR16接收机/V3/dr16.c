#include "dr16.h"
#include "stdlib.h"


DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart) {
	DR16_TypeDef tmp;
	tmp.dr16_uart = dr16_uart;
	return tmp;
}

/* 使能DR16接收 */
void DR16_Enable(DR16_TypeDef* D) {

	UART_HandleTypeDef* huart = D->dr16_uart;

	/* 使能DMA串口接收 */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_DMA_DISABLE(huart->hdmarx);
	while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
		__HAL_DMA_DISABLE(huart->hdmarx);
	}
	huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
	huart->hdmarx->Instance->M0AR = (uint32_t)(D->rx_data1);
	huart->hdmarx->Instance->M1AR = (uint32_t)(D->rx_data2);
	huart->hdmarx->Instance->NDTR = 36;
	SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
	__HAL_DMA_ENABLE(huart->hdmarx);
}

static void DR16_ParseData(DR16_TypeDef* D, uint8_t channel) {

	static int16_t buff[4];
	static uint8_t* rx_data;

	if (channel == 1)
		rx_data = D->rx_data1;
	else if (channel == 2)
		rx_data = D->rx_data2;

	buff[0] = (rx_data[0] | rx_data[1] << 8) & 0x07FF;
	buff[1] = (rx_data[1] >> 3 | rx_data[2] << 5) & 0x07FF;
	buff[2] = (rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07FF;
	buff[3] = (rx_data[4] >> 1 | rx_data[5] << 7) & 0x07FF;

	D->x = rx_data[6] | (rx_data[7] << 8);
	D->y = rx_data[8] | (rx_data[9] << 8);
	D->z = rx_data[10] | (rx_data[11] << 8);
	D->press_l = rx_data[12];
	D->press_r = rx_data[13];
	D->keyboard = rx_data[14] | (rx_data[15] << 8);

	buff[0] -= (int16_t)1024;
	buff[1] -= (int16_t)1024;
	buff[2] -= (int16_t)1024;
	buff[3] -= (int16_t)1024;
	buff[4] -= (int16_t)1024;

	D->sw1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
	D->sw2 = (rx_data[5] >> 4) & 0x0003;

	D->ch1 = buff[0];
	D->ch2 = buff[1];
	D->ch3 = buff[2];
	D->ch4 = buff[3];
}

/* 放在USARTx_IRQHandler中 */
void DR16_RxUpdate(DR16_TypeDef* D) {

	if (D->dr16_uart->Instance->SR & UART_FLAG_IDLE) {

		static uint16_t this_time_rx_len = 0;
		static UART_HandleTypeDef* huart;
		huart = D->dr16_uart;

		__HAL_UART_CLEAR_PEFLAG(huart);


		if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {

			__HAL_DMA_DISABLE(huart->hdmarx);

			this_time_rx_len = 36 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = 36;
			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == 18) {
				DR16_ParseData(D, 2);
			}
		}

		else {
			__HAL_DMA_DISABLE(huart->hdmarx);

			this_time_rx_len = 36 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = 36;
			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(huart->hdmarx);

			if (this_time_rx_len == 18) {
				DR16_ParseData(D, 1);
			}
		}

		/* 接收到一帧数据回调 */
		DR16_Callback(D);
	}
}

__weak void DR16_Callback(DR16_TypeDef* dr16) {

}

void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling) {
	
	static float ratio;
	
	ratio = ceiling / (float)1320;
	
	*data1 = ratio * (D->ch1 + 660);
	*data2 = ratio * (D->ch2 + 660);
	*data3 = ratio * (D->ch3 + 660);
	*data4 = ratio * (D->ch4 + 660);
}
