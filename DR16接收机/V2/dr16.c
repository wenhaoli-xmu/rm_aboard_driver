#include "dr16.h"
#include "stdlib.h"


DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart) {
	DR16_TypeDef tmp;
	tmp.dr16_uart = dr16_uart;
	return tmp;
}

/* 使能DR16接收 */
void DR16_Enable(DR16_TypeDef* D) {
	D->tick = 0;
	HAL_UART_Receive_IT(D->dr16_uart, &D->rx_buf, 1);
}

/* 放在1ms为周期的定时器中断函数中 */
void DR16_Update(DR16_TypeDef* D) {
	D->tick ++;
}

/* 放在RxCpltCallback中 */
void DR16_RxUpdate(DR16_TypeDef* D, UART_HandleTypeDef* huart) {

	if (D->dr16_uart == huart) {

		/* 如果两个字节之间时间间隔过长，重新接收 */
		if (D->tick >= 10) D->rx_cnt = 0;
		D->tick = 0;

		/* 放置接收的数据 */
		D->rx_data[D->rx_cnt++] = D->rx_buf;
		HAL_UART_Receive_IT(D->dr16_uart, &D->rx_buf, 1);

		/* 如果接收完成 */
		if (D->rx_cnt >= 18 && D->done == 0) {
			D->done = 1;
			DR16_Callback(D);
		}
	}
}

__weak void DR16_Callback(DR16_TypeDef* dr16) {

}

static void DR16_ParseData(DR16_TypeDef* D) {
	
	static int16_t buff[4];
	
	buff[0] = (D->rx_data[0] | D->rx_data[1] << 8) & 0x07FF;
	buff[1] = (D->rx_data[1] >> 3 | D->rx_data[2] << 5) & 0x07FF;
	buff[2] = (D->rx_data[2] >> 6 | D->rx_data[3] << 2 | D->rx_data[4] << 10) & 0x07FF;
	buff[3] = (D->rx_data[4] >> 1 | D->rx_data[5] << 7) & 0x07FF;

	D->x = D->rx_data[6] | (D->rx_data[7] << 8);
	D->y = D->rx_data[8] | (D->rx_data[9] << 8);
	D->z = D->rx_data[10] | (D->rx_data[11] << 8);
	D->press_l = D->rx_data[12];
	D->press_r = D->rx_data[13];
	D->keyboard = D->rx_data[14] | (D->rx_data[15] << 8);

	buff[0] -= (int16_t)1024;
	buff[1] -= (int16_t)1024;
	buff[2] -= (int16_t)1024;
	buff[3] -= (int16_t)1024;
	buff[4] -= (int16_t)1024;

	D->sw1 = ((D->rx_data[5] >> 4) & 0x000C) >> 2;
	D->sw2 = (D->rx_data[5] >> 4) & 0x0003;
	
	D->ch1 = buff[0];
	D->ch2 = buff[1];
	D->ch3 = buff[2];
	D->ch4 = buff[3];
}

void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling) {
	
	static float ratio;
	
	ratio = ceiling / (float)1320;
	
	*data1 = ratio * (D->ch1 + 660);
	*data2 = ratio * (D->ch2 + 660);
	*data3 = ratio * (D->ch3 + 660);
	*data4 = ratio * (D->ch4 + 660);
}

void DR16_MainTask(DR16_TypeDef* D) {

	/* 接收完成 */
	if (D->done == 1) {
		D->done = 0;
		DR16_ParseData(D);
	}
}
