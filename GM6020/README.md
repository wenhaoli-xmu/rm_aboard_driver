# GM6020电机驱动

版本：V1

团队：哈理工RM电控组

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/GM6020/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/GM6020/img2.png)

---

## 二、打开一组电机并配置

`GM6020_TypeDef GM6020_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`
- `id_group` 可选择0x1FF和0x2FF

**样例代码**
```c
GM6020_TypeDef M;

M = GM6020_Open(&hcan1, 0x1FF);
GM6020_SetPID(&M, 10.f, 0.f, 0.f, 10, 30000);

/* 设定四个电机的初始角度为180° */
GM6020_SetAngle(&M, 180.f, 180.f, 180.f, 180.f);
```

---

## 三、移植Update函数

`void GM6020_Update(GM6020_TypeDef* M);`
- 放在周期为1ms的定时器周期回调函数中

`void GM6020_RxUpdate(GM6020_TypeDef* M, CAN_HandleTypeDef* hcan);`
- 放在`HAL_CAN_RxFifo0MsgPendingCallback`中

**样例代码**
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == &htim6) {
		GM6020_Update(&M);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	GM6020_RxUpdate(&M, hcan);
}
```

---

## 四、向电机发送速度指令或者位置指令

`void GM6020_SendCmd(GM6020_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`
- `motor1-motor4` 的取值范围是-30000到+30000，代表输入电机的电压

`void GM6020_SetAngle(GM6020_TypeDef* M, float angle1, float angle2, float angle3, float angle4);`
- `angle1-angle4` 的取值范围是[0,360]，代表角度

**使用样例**
```c
/* 每接收到遥控器的数据之后进入该函数 */
void DR16_Callback(DR16_TypeDef* dr16) {
	if (dr16 == &D) {
		DR16_MainTask(&D);
		DR16_MappingData(&D, ch, ch + 1, ch + 2, ch + 3, 360.f);

        /* 传入ID=1和ID=2两个电机的角度 */
		GM6020_SetAngle(&M, ch[2], ch[3], 0.f, 0.f);
	}
}
```