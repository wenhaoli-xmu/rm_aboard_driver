# 串口数据包接收

**当前版本**V1

**V1特性**
* 使用DMA中断接收
* 使用串口空闲终端接收数据包

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/tree/master/RXD/img1)
![img2](https://github.com/RainFromCN/rm_aboard_driver/tree/master/RXD/img2)
![img3](https://github.com/RainFromCN/rm_aboard_driver/tree/master/RXD/img3)

---

## 二、创建一个RXD实例并使能

`RXD_TypeDef RXD_Open(UART_HandleTypeDef* huart, uint8_t* rx_addr, uint8_t rx_length);`
`void RXD_Enable(RXD_TypeDef* R);`

**样例代码**
```c
RXD_TypeDef R;

float dataRec;

R = RXD_Open(&huart7, (uint8_t*)&dataRec, 4)
RXD_Enable(&R);
```

---

## 三、将RxUpdate函数移植在it.c中

`void RXD_RxUpdate(RXD_TypeDef* R);`
- 这个函数需要放在中断文件it.c中的`void USARTx_IRQHandler`函数中

**使用样例**
```c
#include "rxd.h"

extern RXD_TypeDef R;

/* 原本的USARTx_IRQHandler函数的模样 */
void USART7_IRQHandler(void) {
    /* USER CODE BEGIN USART7_IRQn 0 */

    /* USER CODE END USART7_IRQn 0 */
    HAL_UART_IRQHandler(&huart7);
    /* USER CODE BEGIN USART7_IRQn 1 */

    /* USER CODE END USART7_IRQn 1 */
}

/* 删改后的USARTx_IRQHandler函数的模样 */
void USART7_IRQHandler(void) {
	RXD_RxUpdate(&R);
}
```

---

## 四、重载Callback函数（可选）

`__weak void RXD_Callback(RXD_TypeDef* rxd);`
* 每当RXD对应的串口接收到一帧的数据后，系统会自动进入该回调函数
* 可以在该回调函数中处理数据

**样例代码**
```c
float cmd_vel;

void RXD_Callback(RXD_TypeDef* rxd) {
    if (rxd == &R) {
        cmd_vel = dataRec;
    }
}
```

---



