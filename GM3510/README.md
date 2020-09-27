# 大疆GM3510伺服电机驱动
版本：v1.0

团队：哈理工RM电控组

---

## 一、MX配置

---

### 1、CAN通讯配置
![img1]()
![img2]()

---

## 二、移植教程

---

### 1、实例化一个GM3510电机组对象

`GM3510_TypeDef GM3510_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`

- `hcan` 表示使用的CAN
- `id_group` 表示CAN通讯的ID标识符，一般为 0x1ff

**使用样例**

```c
GM3510_TypeDef M;

M = GM3510_Open(&hcan1, 0x1ff);
```

---

### 2、移植两个Update函数

`void GM3510_Update(GM3510_TypeDef* M);`
- 这个函数需要放在周期为1ms的定时器中断中运行

`void GM3510_RxUpdate(GM3510_TypeDef* M, CAN_HandleTypeDef* hcan);`
- 这个函数需要放在CAN接受中断回调函数中运行

**使用样例**
```c
/* 1ms中断周期回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        GM3510_Update(&M);
    }
}

/* CANRx接受中断回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	MOTOR_RxUpdate(&M, hcan);
} 
```

---

### 3、设置PID参数
`void GM3510_SetPID(GM3510_TypeDef* M, float kp, float ki, float kd, uint32_t sample_period, int16_t output_saturation);`
- `kp, ki, kd` 控制器参数
- `sample_period` 采样周期，一般设置为10ms就可以了
- `output_saturation` 输出抗饱和，3510的上下限为 -29000 ~ 29000

**使用样例**
```c
/* 实例化一个3510电机 */
M = MOTOR_Open(&hcan1, 0x1ff);

/* 设置PID参数，采样周期，饱和上限 */
MOTOR_SetPID(&M, 8., 0, 0, 10, 4000);
```

---

### 4、移植Callback函数

`void GM3510_Callback(GM3510_TypeDef* M);`
- 需要将Callback函数放在主函数中的while(1)中不断运行

**使用样例**
```c
while (1) {
    GM3510_Callback(&M);
}
```

---

### 5、使用 `M.loc_set` 设置电机的期望转角

**使用样例**
```c
while (1) {
    static uint8_t i = 0;
	
	MOTOR_Callback(&M);
	
	if (systick - tick1 >= 3000) {
		
		tick1 = systick;
		
		for (i = 0; i < 3; i ++) {
			M.loc_set[i] += 2000;
			M.loc_set[i] %= 8192;
		}
	}
}
```