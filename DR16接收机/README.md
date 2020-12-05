# 大疆DR16接收机驱动

**当前版本**
v3

**v1特性**
- 使用中断进行逐帧接收
- 使用定时器判断串口是否空闲

**v2特性**
- 在V1的基础上添加了Callback函数

**v3特性**
- 加入了freeRTOS
- 取消了MainTask函数，直接在串口空闲中断中解析数据
- 加入了DMA

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img3.png)
![img4](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img4.png)

---

## 二、创建一个DR16接收机对象并使能

`DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart);`
`void DR16_Enable(DR16_TypeDef* D);`

**使用样例**
```c
DR16_TypeDef D;

D = DR16_Open(&huart1);
DR16_Enable(&D);
```

---

## 三、移植RxUpdate函数在it.c文件中

`void DR16_RxUpdate(DR16_TypeDef* D);`
- 这个函数需要放在中断文件it.c中的`void USARTx_IRQHandler`函数中

**使用样例**
```c
#include "dr16.h"

extern DR16_TypeDef D;

/* 原本的USARTx_IRQHandler函数的模样 */
void USART1_IRQHandler(void) {
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/* 删改后的USARTx_IRQHandler函数的模样 */
void USART1_IRQHandler(void) {
	DR16_RxUpdate(&D);
}
```

---

## 四、重载Callback函数（可选）

`__weak DR16_Callback(DR16_TypeDef* dr16)`
- 每当DR16接收到一帧的数据帧的时候，系统自动进入该回调函数

**样例代码**
```c
/* 接收到一帧数据后自动进入Callback函数 */

float cmd_vel[4]; //由DR16_Callback进行更新
float imu_vel[4]; //由IMU_Callback进行更新

void DR16_Callback(DR16_TypeDef* dr16) {
	if (dr16 == &D) {
		DR16_MappingData(&D, ch, ch + 1, ch + 2, ch + 3, 400);

		float forward = ch[1] - 200;
		float rotate  = ch[2] - 200;
		float offset  = ch[0] - 200;

		cmd_vel[0] = forward - offset - rotate * 0.5;
		cmd_vel[1] = forward + offset + rotate * 0.5;
		cmd_vel[2] = forward - offset + rotate * 0.5;
		cmd_vel[3] = forward + offset - rotate * 0.5;

		M2006_CmdVel(&M,\
			cmd_vel[0] + imu_vel[0],\
			cmd_vel[1] + imu_vel[1],\
			cmd_vel[2] + imu_vel[2],\
			cmd_vel[3] + imu_vel[3]\
		);
	}
}
```

---

## 五、MappingData的使用方法

至此DR16驱动移植完毕，此外DR16还提供了MappingData的方法，来供用户获取四个遥感通道的数据

`void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling);`

- `data1 ~ data4` 用于保存四个通道的传入数据
- `ceiling` 将四个通道的数据数据映射到 [0, ceiling] 上
- 具体的使用方法见第四部分

---