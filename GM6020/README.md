# GM6020电机驱动

**当前版本**
V2

**V1特性**
* 使用定时器作为定时采样控制
* 建议使用5ms-10ms采样周期

**V2特性**
* 使用freeRTOS作为定时采样控制
* 提升了CAN数据解析效率
* 支持1ms采样周期
* 支持变速积分算法
* 支持不完全微分算法

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/GM6020/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/GM6020/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/GM6020/img3.png)

---

## 二、打开一组电机并配置

`GM6020_TypeDef GM6020_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`
- `id_group` 可选择0x1FF和0x2FF

`void GM6020_SetDir(GM6020_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);`
- `dir1-dir4` 电机的转向，设置1为正传，-1为反转，0为不转

`void GM6020_CtrlParams(GM6020_TypeDef* M, float kp, float ki, float kd, int16_t output_saturation);`
- `kp, ki, kd` 传统PID参数
- `output_saturation` 输出饱和，一般设置30000

`void GM6020_ExCtrlParams(GM6020_TypeDef* M, float A, float B, float alpha);`
- `A, B` 变速积分参数，先调B，再调A+B
- `alpha` 不完全微分系数，这个值越大，微分器的滤波效果越强
- 如果不想使用变速积分，那么置B为1e6，置A为0
- 如果不想使用不完全微分器，那么置 alpha 为1

**样例代码**
```c
GM6020_TypeDef G;

G = GM6020_Open(&hcan1, 0x1ff);
GM6020_SetDir(&G, 1, 1, 0, 0);

/* 设置PID参数 */
GM6020_CtrlParams(&G, 40.f, 0.f, 6000.f, 30000);

/* 设置A,B,alpha */
GM6020_ExCtrlParams(&G, 0.f, 1e6, 0.95); //采样周期为1ms的时候，建议设置alpha=0.95
```

---

## 三、根据需要，修改RxUpdate函数

```c
if (tmp == 0x205 || tmp == 0x209)
else if (tmp == 0x206 || tmp == 0x20A)
else if (tmp == 0x207 || tmp == 0x20B)
else if (tmp == 0x208)
```

倘若我们将电机的反馈数据通过电调设置为0x207和0x208，那么需要将其他的删除，以免发生冲突，删除后的RxUpdate应该如下：

```c
if (tmp == 0x207)
else if (tmp == 0x208)
```

在上面的代码中，只保留了0x207和0x208两个回馈ID

---

## 四、移植RxUpdate函数

`void GM6020_RxUpdate(GM6020_TypeDef* M, CAN_HandleTypeDef* hcan);`
- 放在`HAL_CAN_RxFifo0MsgPendingCallback`中

**样例代码**
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	GM6020_RxUpdate(&G, hcan);
}
```

---

## 五、移植MainTask函数

`void GM6020_MainTask(GM6020_TypeDef* M);`
- 放在freeRTOS的进程中运行

**样例代码**
```c
void gimbal_task(void const *argument) {
	while (1) {
		GM6020_MainTask(&G);
		osDelay(1);
	}
}
```

---

## 六、向电机发送位置指令

**使用样例**
```c
G.loc_set[0] = 0;
G.loc_set[1] = 8191;
G.loc_set[2] = 0;
G.loc_set[3] = 0;
```