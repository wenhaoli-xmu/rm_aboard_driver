#include "graph.h"


static UART_HandleTypeDef* graph_uart;
static float* add1;
static float* add2;
static float* add3;

static uint8_t buf;
static uint8_t rx_data[12];
static uint8_t rx_cnt, cx, state = IDLE;


void GRAPH_Open(UART_HandleTypeDef* huart, float* p, float* i, float* d) {
    graph_uart = huart;
    add1 = p;
    add2 = i;
    add3 = d;
    HAL_UART_Receive_IT(graph_uart, &buf, 1);
}

void GRAPH_Pub(int data) {
	int vel = data * 100;
	uint8_t buf;
	static uint8_t i;
	uint8_t cx = 0;
	uint8_t* p = (uint8_t*)&data;

	buf = 0xaa;
	HAL_UART_Transmit(huart, &buf, 1, 0xffff);
	buf = 0x88;
	cx += 0x88;
	HAL_UART_Transmit(huart, &buf, 1, 0xffff);
	for (i = 0; i < 4; i ++) {
		buf = p[i];
		cx += buf;
		HAL_UART_Transmit(huart, &buf, 1, 0xffff);
	}
	buf = 0x55;
	for (i = 0; i < 8; i ++) {
		HAL_UART_Transmit(huart, &buf, 1, 0xffff);
		cx += buf;
	}
	buf = cx;
	HAL_UART_Transmit(huart, &buf, 1, 0xffff);
	buf = 0x2f;
	HAL_UART_Transmit(huart, &buf, 1, 0xffff);
}

void GRAPH_RxUpdate(UART_HandleTypeDef* huart) {
    if (huart == graph_uart) {
		if (buf == 0xaa) {
			rx_cnt = 0;
			state = GRAPH_READY;
		}
		else if (buf == 0x07 && state == GRAPH_READY) {
			state = GRAPH_BUSY;
		}
		else if (state == GRAPH_BUSY) {
			rx_data[rx_cnt++] = buf;
			if (rx_cnt == 12) {
				state = GRAPH_DONE;
			}
		}
		else if (state == GRAPH_DONE) {

			uint8_t i = 0;
			cx = 0;
			cx += 0x07;
			for (i = 0; i < rx_cnt; i ++) {
				cx += rx_data[i];
			}
			if (cx == buf) {
				uint8_t *p = (uint8_t*)add1;
				uint8_t i;
				for (i = 0; i < 4; i ++) p[i] = rx_data[i];
				p = (uint8_t*)add2;
				for (i = 0; i < 4; i ++) p[i] = rx_data[i+4];
				p = (uint8_t*)add3;
				for (i = 0; i < 4; i ++) p[i] = rx_data[i+8];
			}
		}
		HAL_UART_Receive_IT(graph_uart, &buf, 1);
	}
}
