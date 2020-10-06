# 大疆DR16接收机驱动

版本：v1.0

团队：哈理工RM电控组

---

## 一、MX配置

![img1]()

![img2]()

![img3]()


---

## 二、创建一个DR16接收机对象

`DR16_TypeDef DR16_Open(UART_HandleTypeDef* dr16_uart);`

**使用样例**
```c
DR16_TypeDef D;

D = DR16_Open(&huart1);
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

## 四、移植Callback函数

`void DR16_Callback(DR16_TypeDef* D);`
- 此函数放在主函数的while(1)中不断运行即可

**样例代码**
```c
int main() {
    /* ... */

    while (1) {
        DR16_Callback(&D);
    }
}
```

---

## 五、MappingData的使用方法

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
	MOTOR_Callback(&M);
	DR16_Callback(&D);
	
	/* 将遥控器获取的结果映射成0-360之间的数值 */
	DR16_MappingData(&D, &angle1, &angle2, &angle3, &angle4, 360);
	
	/* 获取滤波结果 */
	FILTER_Callback(&F);
	
	/* 设置电机的角度 */
	MOTOR_SetAngle(&M, angle1_smooth, angle4_smooth, angle3_smooth);
}
```

---

## 获取两个按键的值的方法

**样例代码**
```c
/* 按键1的数值 */
int16_t key1 = D.sw1;

/* 按键2的数值 */
int16_t key2 = D.sw2;
```
