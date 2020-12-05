# 大疆M3508电机驱动

版本：V2

团队：哈理工RM电控组

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M3508/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M3508/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M3508/img3.png)

---

## 二、创建一个电机组并设置转向

`M3508_TypeDef M3508_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`

- `id_group` 电机组的标识符，3508电机可选择0x200，0x1ff

`void M3508_SetDir(M3508_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);`

**样例代码**
```c
M3508_TypeDef M;

M = M3508_Open(&hcan1, 0x200);

/* 一四电机反转，二三电机正转 */
M3508_SetDir(&M, -1, 1, 1, -1);
```

---

## 三、设置控制器参数

`void M3508_CtrlParams(M3508_TypeDef* M, float kp, float ki, float kd, float A, float B, int16_t output_saturation);`

- `kp ki kd` 位置式控制器三个参数
- `A B` 变速积分的两个参数
- `output_saturation` 输出限幅，一般取最大值16000

**样例代码**
```c
/* 经典控制器参数，实测转速100转每秒之内效果优异 */
M3508_CtrlParams(&M, 80.f, 16.f, 0.f, 3.f, 15.f, 16000);
```

---

## 四、移植Update函数和MainTask函数

`void M3508_RxUpdate(M3508_TypeDef* M, CAN_HandleTypeDef* hcan);`

- 放在HAL_CAN_RxFifo0MsgPendingCallback()中

`void M3508_MainTask(M3508_TypeDef* M);`

- 放在freeRTOS的线程中不断运行

**样例代码**
```c

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    M3508_RxUpdate(&M, hcan);
}

void m3508_task(void const *argument) {
    while (1) {
        M3508_MainTask(&M);

        /* 1ms 运行一次 */
        osDelay(1);
    }
}
```

---

## 五、向电机发送指令

`void M3508_SendCmd(M3508_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`

- 作用：向电机发送原始指令，数据范围是 -16384～16384

`void M3508_CmdVel(M3508_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4);`

- 向电机发送速度指令
- 单位是RPS（转每秒）

**样例代码**
```c
float ch[4];

void DR16_Callback(DR16_TypeDef* D) {
    DR16_MainTask(&D);
    DR16_MappingData(&D, ch, ch+1, ch+2, ch+3, 100);

    M3508_CmdVel(&M, ch[0], ch[1], ch[2], ch[3]);
}
```

---
