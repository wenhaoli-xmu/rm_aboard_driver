# 大疆DR16接收机驱动

版本：v2.0

团队：哈理工RM电控组

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img1.png)

![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img2.png)

![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/DR16/img3.png)


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

## 三、移植Update函数

`void DR16_Update(DR16_TypeDef* D);`
- 这个函数需要放在1ms定时器中断回调函数中

`void DR16_RxUpdate(DR16_TypeDef* D, UART_HandleTypeDef* huart);`
- 这个函数需要放在UART中断回调函数中

**使用样例**
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        DR16_Update(&D);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    DR16_RxUpdate(&D, huart);
}
```

---

## 四、移植MainTask函数

`void DR16_MainTask(DR16_TypeDef* D);`
- 此函数放在主函数的while(1)中不断运行即可，方法一要求系统任务量较小，对遥控器实时性要求不高
- 也可以放在DR16回调函数`DR16_Callback`中运行，方法二适合系统任务量大，对遥控器实时性要求高的场景

**样例代码**
```c
/* 使用方法1 */
while (1) {
	DR16_MainTask(&D);
}

/* 使用方法2 */
void DR16_Callback(DR16_TypeDef* dr16) {
	if (dr16 == &D) {
		DR16_MainTask(&D);
		DR16_MappingData(&D, ch, ch + 1, ch + 2, ch + 3, 360.f);
		GM6020_SetAngle(&M, ch[2], ch[3], 0.f, 0.f);
	}
}
```

---

## 五、重载Callback函数（可选）

`__weak DR16_Callback(DR16_TypeDef* dr16)`
- 每当DR16接收到一帧的数据帧的时候，系统自动进入该回调函数
- 进入Callback函数后，如果需要使用该帧的数据，需要先进行这一帧数据的解析(MainTask函数)

**样例代码**
```c
/* 接收到一帧数据后自动进入Callback函数 */
void DR16_Callback(DR16_TypeDef* dr16) {

	/* 判断是不是遥控器D接收的数据 */
	if (dr16 == &D) {

		/* 对该帧数据进行解析 */
		DR16_MainTask(&D);

		/* 将遥控器数据映射成为角度 */
		DR16_MappingData(&D, ch, ch + 1, ch + 2, ch + 3, 360.f);

		/* 向电机发送角度信息 */
		GM6020_SetAngle(&M, ch[2], ch[3], 0.f, 0.f);
	}
}
```

---

## 六、MappingData的使用方法

至此DR16驱动移植完毕，此外DR16还提供了MappingData的方法，来供用户获取四个遥感通道的数据

`void DR16_MappingData(DR16_TypeDef* D, float* data1, float* data2, float* data3, float* data4, float ceiling);`

- `data1 ~ data4` 用于保存四个通道的传入数据
- `ceiling` 将四个通道的数据数据映射到 [0, ceiling] 上

**使用样例**
```c

float angle1, angle2, angle3, angle4;
float angle1_smooth, angle2_smooth, angle3_smooth, angle4_smooth;

while (1) {
    /* 获取电机数据与遥控器数据 */
	MOTOR_Maintask(&M);
	DR16_Maintask(&D);
	
	/* 将遥控器获取的结果映射成0-360之间的数值 */
	DR16_MappingData(&D, &angle1, &angle2, &angle3, &angle4, 360);
	
	/* 获取滤波结果 */
	FILTER_Callback(&F);
	
	/* 设置电机的角度 */
	MOTOR_SetAngle(&M, angle1_smooth, angle4_smooth, angle3_smooth);
}
```

---