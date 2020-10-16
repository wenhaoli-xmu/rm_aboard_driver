# EC60电机驱动

版本：V1.0

团队：哈理工RM电控组

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/EC60/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/EC60/img2.png)

---

## 二、打开一个电机组，并设置参数

`EC60_TypeDef EC60_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`

- `id_group` EC60的代号为0x200

`void EC60_SetDir(EC60_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);`
- 设置电机转向
- `dir1-dir4` 四个电机的转向，传入1或者-1

`void EC60_CtrlParams(EC60_TypeDef* M, float a1, float a2, float a3, uint32_t sample_period, int16_t output_saturation);`
- 设置控制参数(kp, ki, kd)
- `a1-a3` 对应kp,ki,kd三个参数
- `sample_period` 采样时间，一般选择10即可，代表10ms
- `output_saturation` 输出抗饱和上限，由于EC60最大电压是5000，所以output_saturation一般取5000

**使用样例**
```c
EC60_TypeDef M;

M = EC60_Open(&hcan1, 0x200);
EC60_SetDir(&M, -1, 1, 1, -1);
EC60_CtrlParams(&M, 4000., 600, 0, 20, 5000);

/* 设置当转速误差≤2的时候启用积分 */
M.pid.ki_saturation = 2;
```

---

## 三、移植Update函数

`void EC60_Update(EC60_TypeDef* M);`
- 放在HAL_TIM_PeriodElapsedCallback中（周期1ms）

`void EC60_RxUpdate(EC60_TypeDef* M, CAN_HandleTypeDef* hcan);`
- 放在HAL_CAN_RxFifo0MsgPendingCallback中

**使用样例**
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        EC60_Update(&M);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    EC60_RxUpdate(&M, hcan);
}
```

---

## 四、移植MainTask函数

`void EC60_MainTask(EC60_TypeDef* M);`
- 放在while(1)中

**样例代码**
```c
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    EC60_MainTask(&M);
}
```

---

## 五、向电机发送指令

`void EC60_SendCmd(EC60_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`
- 向电机直接发送原始数据
- `motor1-motor4` int16_t型数据，范围-5000~+5000，表示电压数据

`void EC60_CmdVel(EC60_TypeDef* M, float vel_rpm1, float vel_rpm2, float vel_rpm3, float vel_rpm4);`
- 向电机发送速度（RPS）数据
- `vel_rpsn` 电机的期望转速，单位是RPS

**使用样例**
```c
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* EC60主程序 */
    EC60_MainTask(&M);

    /* 将DR16接收的数据映射刀[0,20]区间内 */
    DR16_MappingData(&D, ch, ch + 1, ch + 2, ch + 3, 20);

    /* 计算四个电机的转速 */
    forward = ch[1] - 10;
    rotate = ch[2] - 10;
    offset = ch[0] - 10;
    cmd_vel[0] = forward - offset - rotate * 0.5;
    cmd_vel[1] = forward + offset + rotate * 0.5;
    cmd_vel[2] = forward - offset + rotate * 0.5;
    cmd_vel[3] = forward + offset - rotate * 0.5;

    /* 向电机发送数据 */
    EC60_CmdVel(&M, cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_vel[3]);

}
```