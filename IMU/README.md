# 九轴传感器驱动

版本：V1

团队：哈理工RM电控组

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/IMU/img1.png)

![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/IMU/img2.png)

![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/IMU/img3.png)

![img4](https://github.com/RainFromCN/rm_aboard_driver/blob/master/IMU/img4.png)

![img5](https://github.com/RainFromCN/rm_aboard_driver/blob/master/IMU/img5.png)

---

## 二、创建一个IMU对象

`IMU_TypeDef IMU_Open(SPI_HandleTypeDef* hspi, uint32_t sample_period);`
- 该函数的作用是创建一个IMU对象
- `sample_period` 采样时间，即每间隔多少ms计算一次姿态角

`uint8_t IMU_CheckSuccess(IMU_TypeDef* I)`
- 该函数的作用通过读取设备ID（0x70）来检测MPU是否可以正常通讯

`void IMU_SetBoardState(IMU_TypeDef* I, uint8_t board_state)`
- 该函数的作用是配置开发板的方向是向上还是向下
- 如果向上，则`board_state`传入`IMU_BOARD_UP`，否则传入`IMU_BOARD_DOWN`

**使用样例**
```c
IMU_TypeDef I;

/* 设置IMU的采样周期为10ms，即每隔10ms更新一次姿态角 */
while (IMU_CheckSuccess(&I) != HAL_OK) {
    I = IMU_Open(&hspi5, 10);
}

/* 设置IMU的方向为正面朝上 */
IMU_SetBoardState(&I, IMU_BOARD_UP);
```

---

## 三、移植Update函数

`void IMU_Update(IMU_TypeDef* I)`
- 放在周期为1ms的定时器中断回调函数中运行

**样例代码**
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        IMU_Update(&I);
    }
}
```

---

## 四、移植MainTask函数

`void IMU_MainTask(IMU_TypeDef* I)`
- 放在while(1)中运行

**样例代码**
```c
while (1) {
    IMU_MainTask(&I);
}
```

---

## 五、重载Callback函数

`__weak void IMU_Callback(IMU_TypeDef* imu)`
- 如果用户需要，可以在main函数中进行重载
- 在IMU更新使能的时候进行回调

**样例代码**
```c
/* 放在Callback回调函数中运行，可以当达到采样时间后立刻运行MainTask */
void IMU_Callback(IMU_TypeDef* imu) {
    if (imu == &I) IMU_MainTask(imu);
}
```
