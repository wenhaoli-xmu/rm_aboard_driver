# 混合电机驱动（速度控制）

**当前版本**V1

**V1特性**
* 可以对四个电机使用各自独立的PID控制器
* 支持变速积分PID和不完全微分PID
* 变速积分器配备参数A、B。不完全微分器配备参数α
* 使用freeRTOS
* 当前最大支持四个电机为一组，使用FIFO0作为接收器
* 四个电机必须具有同一个ID组，例如M2006和M3508都是0x200

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/tree/master/%E6%B7%B7%E5%90%88%E7%94%B5%E6%9C%BA%EF%BC%88%E9%80%9F%E5%BA%A6%E6%8E%A7%E5%88%B6%EF%BC%89/img1)
![img2](https://github.com/RainFromCN/rm_aboard_driver/tree/master/%E6%B7%B7%E5%90%88%E7%94%B5%E6%9C%BA%EF%BC%88%E9%80%9F%E5%BA%A6%E6%8E%A7%E5%88%B6%EF%BC%89/img2)
![img3](https://github.com/RainFromCN/rm_aboard_driver/tree/master/%E6%B7%B7%E5%90%88%E7%94%B5%E6%9C%BA%EF%BC%88%E9%80%9F%E5%BA%A6%E6%8E%A7%E5%88%B6%EF%BC%89/img3)

---

## 二、建立并配置一个混合电机组

`SPDM_TypeDef SPDM_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`
* `hcan` 使用的CAN口
* `id_group` 电机组的ID编号，M3508和M2006都是0X200

`void SPDM_SetDir(SPDM_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);`
* 用于设置电机的转向，正传设置1，反转设置-1，不转设置0

**样例代码**
```c
SPDM_TypeDef S;

S = SPDM_Open(&hcan1, 0x200);
```

---

## 三、设置控制器参数

`void SPDM_CtrlParams(SPDM_TypeDef* M, float* kp, float* ki, float* kd, int16_t* output_saturation);`
* 设置电机的传统PID控制器参数
* M2006的输出限幅为10000、M3508的输出限幅为16000

`void SPDM_ExCtrlParams(SPDM_TypeDef* M, float* A, float* B, float* alpha);`
* 该函数选择性调用即可
* 如果只使用不完全微分器，则置A=0，B=inf
* 如果只使用变速积分器，则置alpha=0

**样例代码**
```c
S = SPDM_Open(&hcan1, 0x200);
SPDM_SetDir(&S, 1, 0, 1, 1);

/* M2006设置PID参数为kp=600,ki=60,kd=1000 */
float ctrlParams[4][4] = {
    {600.f,     0.f,    100.f,  100.f},
    {60.f,      0.f,    0.f,    0.f},
    {1000.f,    0.f,    0.f,    0.f}
};

int16_t outputSat[4] = {10000, 0, 16000, 16000};

/* M2006设置拓展参数为A=17,B=3,alpha=0.3 */
float exCtrlParams[3][4] = {
    {17.f,      0.f,    0.f,    0.f},
    {3.f,       0.f,    1e6,    1e6},
    {0.3f,      0.f,    0.f,    0.f}
};

SPDM_CtrlParams(&S, ctrlParams[0], ctrlParams[1], ctrlParams[2], outputSat);
SPDM_ExCtrlParams(&S, exCtrlParams[0], exCtrlParams[1], exCtrlParams[2]);
```

---

# 四、根据需要定制RxUpdate函数

```c
if (tmp == 0x201 || tmp == 0x205)
else if (tmp == 0x202 || tmp == 0x206)
else if (tmp == 0x203 || tmp == 0x207)
else if (tmp == 0x204 || tmp == 0x208)
```

如果第一个电机反馈ID为0x204、第二个电机反馈数据为0x206，第三个电机反馈数据为0x201，那么可以将代码更改为

```c
if (tmp == 0x204)
else if (tmp == 0x206)
else if (tmp == 0x201)
```

---

# 五、移植RxUpdate函数和MainTask函数

`void SPDM_RxUpdate(SPDM_TypeDef* M, CAN_HandleTypeDef* hcan);`
* 放在HAL_CAN_RxFifo0MsgPendingCallback()中

`void SPDM_MainTask(SPDM_TypeDef* M);`
* 放在freeRTOS中不断运行

**样例代码**
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    SPDM_RxUpdate(&M, hcan);
}

void spdm_task(void const *argument) {
    while (1) {
        SPDM_MainTask(&M);

        /* 1ms 运行一次 */
        /* 实测上面的那套控制参数搭配1ms的控制周期，可以有相当不错的控制效果 */
        osDelay(1);
    }
}
```

---

# 六、向电机发送指令

`void SPDM_SendCmd(SPDM_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`

- 作用：向电机发送原始指令，数据范围是 -16384～16384

`void SPDM_CmdVel(SPDM_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4);`

- 向电机发送速度指令
- 单位是RPS（转每秒）

**样例代码**
```c
float ch[4];

void DR16_Callback(DR16_TypeDef* D) {
    DR16_MainTask(&D);
    DR16_MappingData(&D, ch, ch+1, ch+2, ch+3, 100);

    SPDM_CmdVel(&M, ch[0], ch[1], ch[2], ch[3]);
}
```

---
