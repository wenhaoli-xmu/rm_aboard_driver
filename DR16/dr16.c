#include "dr16.h"
#include "stdlib.h"


DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart) {
	
	DR16_TypeDef tmp;
	
	tmp.dr16_uart = dr16_uart;
	tmp.thresh = 4;
	
	HAL_UART_Receive_IT(dr16_uart, &tmp.rx_buf, 1);
	
	return tmp;
}

/* 放在1ms为周期的定时器中断函数中 */
void DR16_Update(DR16_TypeDef* D) {
	D->tick ++;
}

/* 放在RxCpltCallback中 */
void DR16_RxUpdate(DR16_TypeDef* D, UART_HandleTypeDef* huart) {
	
	if (D->dr16_uart == huart && D->done == 0) {
		D->rx_data[D->rx_cnt++] = D->rx_buf;
		D->done = 1;
		
		HAL_UART_Receive_IT(D->dr16_uart, &D->rx_buf, 1);
	}
}

static void DR16_ParseData(DR16_TypeDef* D) {
	
	static int16_t buff[4];
	
	buff[0] = (D->rx_data[0] | D->rx_data[1] << 8) & 0x07FF;
	buff[0] -= 1024;
	buff[1] = (D->rx_data[1] >> 3 | D->rx_data[2] << 5) & 0x07FF;
	buff[1] -= 1024;
	buff[2] = (D->rx_data[2] >> 6 | D->rx_data[3] << 2 | D->rx_data[4] << 10) & 0x07FF;
	buff[2] -= 1024;
	buff[3] = (D->rx_data[4] >> 1 | D->rx_data[5] << 7) & 0x07FF;
	buff[3] -= 1024;

	D->sw1 = ((D->rx_data[5] >> 4) & 0x000C) >> 2;
	D->sw2 = (D->rx_data[5] >> 4) & 0x0003;
	
	if ((abs(buff[0]) < 660) && \
      (abs(buff[1]) < 660) && \
      (abs(buff[2]) < 660) && \
      (abs(buff[3]) < 660))
	{
		D->ch1 = buff[0];
		D->ch2 = buff[1];
		D->ch3 = buff[2];
		D->ch4 = buff[3];
	}	
}

void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling) {
	
	static float ratio;
	ratio = ceiling / (float)1320;
	
	*data1 = ratio * (D->ch1 + 660);
	*data2 = ratio * (D->ch2 + 660);
	*data3 = ratio * (D->ch3 + 660);
	*data4 = ratio * (D->ch4 + 660);
}

void DR16_Callback(DR16_TypeDef* D) {
	
	if (D->done == 1) {
		D->done = 0;
		DR16_ParseData(D);
	}
	
	if (D->tick >= D->thresh) {
		D->tick = 0;
		
		D->rx_cnt = 0;
	}
}
