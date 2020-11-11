# HWT101驱动

版本：V1.0
团队：哈理工RM电控组

---

## 一、配置MX
![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HWT101/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HWT101/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HWT101/img3.png)

---

## 二、创建一个HWT对象并使能DMA

**样例代码**
```c
HWT101_TypeDef H;
H = HWT101_Open(&huart6, 10);
HWT101_Enable(&H);
```

---

## 三、一直RxUpdate函数在it.c中

注意，it.c文件中有一个`void USARTx_IRQHandler()`函数，需要将RxUpdate函数放在其中，并且需要将原来IRQ里面的内容全部删除掉

**样例代码**
```c

#include "hwt101.h"

extern HWT101_TypeDef H;

/* 原本的USARTx_IRQHandler函数的模样 */
void USART6_IRQHandler(void) {
    /* USER CODE BEGIN USART6_IRQn 0 */

    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */

    /* USER CODE END USART6_IRQn 1 */
}

/* 删改后的USARTx_IRQHandler函数的模样 */
void USART6_IRQHandler(void) {
	HWT101_RxUpdate(&H);
}
```

---

## 四、获取yaw轴的数据

**样例代码**
```c
float yaw;
float wx, wy, wz;

yaw = H.yaw;
wx = H.wx;
wy = H.wy;
wz = H.wz;
```

---

## 五、使用Callback回调函数进行操作

`void HWT101_Callback(HWT101_TypeDef* hwt);`
- 每当读取到一帧的数据之后，会自动进入回调函数中去
- 可以在每次读取到数据的时候，对电机的期望速度进行更新

**使用样例**
```c
float imu_vel[4];
float cmd_vel[4];

void HWT101_Callback(HWT101_TypeDef* H) {

	static float rotate = -5 * H->yaw;

	imu_vel[0] = rotate;
	imu_vel[1] = -rotate;
	imu_vel[2] = -rotate;
	imu_vel[3] = rotate;

    M2006_CmdVel(&M,\
        cmd_vel[0] + imu_vel[0],\
        cmd_vel[1] + imu_vel[1],\
        cmd_vel[2] + imu_vel[2],\
        cmd_vel[3] + imu_vel[3]\
    );
}
```

---
