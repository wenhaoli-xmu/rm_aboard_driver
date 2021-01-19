# 大疆M2006电机驱动

**当前版本**V2.1

**V1特性**
* 在CAN中断进行初步的数据解析分离
* 使用freeRTOS进行电机控制

**V2特性**
* 改善了CAN通讯的性能，减小了CAN中断的处理时间
* 优化了不完全微分器，将拓展参数的设置分离出来
* 解决了多电机协作时出现的宏定义冲突问题

**V2.1特性**
* 分离了Pid计算和CAN信号发送，分别在两个线程中运行
* 提升了系统的稳定性

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M2006/img1.png)
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M2006/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/M2006/img3.png)

---

## 二、创建一个电机组并设置转向

`M2006_TypeDef M2006_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);`

- `id_group` 电机组的标识符，2006电机可选择0x200，0x1ff

`void M2006_SetDir(M2006_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);`

**样例代码**
```c
M2006_TypeDef M;

M = M2006_Open(&hcan1, 0x200);

/* 一四电机反转，二三电机正转 */
M2006_SetDir(&M, -1, 1, 1, -1);
```

---

## 三、设置控制器参数

`void M2006_CtrlParams(M2006_TypeDef* M, float kp, float ki, float kd, int16_t output_saturation);`
`void M2006_ExCtrlParams(M2006_TypeDef* M, float A, float B, float alpha);`

- `kp ki kd` 位置式控制器三个参数
- `A B` 变速积分的两个参数
- `alpha` 不完全微分器参数，越大越平缓
- `output_saturation` 输出限幅，一般取最大值10000

**样例代码**
```c
/* 经典控制器参数，需要搭配1ms的控制采样周期 */
M2006_CtrlParams(&M, 600.f, 60.f, 1000.f, 10000);
M2006_ExCtrlParams(&M, 17.f, 3.f, 0.3);
```

---

## 四、根据需要定制函数

```c
if (tmp == 0x201 || tmp == 0x205)
else if (tmp == 0x202 || tmp == 0x206)
else if (tmp == 0x203 || tmp == 0x207)
else if (tmp == 0x204 || tmp == 0x208)
```

倘若我们将电机的反馈数据通过电调设置为0x201和0x202，那么需要将其他的删除，以免发生冲突，删除后的RxUpdate应该如下：

```c
if (tmp == 0x201)
else if (tmp == 0x202)
```

在上面的代码中，只保留了0x201和0x202两个回馈ID

---

## 五、移植Rxpdate函数和CalcPid函数和SendCmd函数

**注意**
* `CalcPid`函数的优先级为**High**
* `SendCmd`函数的优先级为**RealTime**

**函数原型**
`void M2006_RxUpdate(M2006_TypeDef* M, CAN_HandleTypeDef* hcan);`
- 放在HAL_CAN_RxFifo0MsgPendingCallback()中

`void M2006_CalcPid(M2006_TypeDef* M);`
- 放在freeRTOS的线程中不断运行

`void M2006_SendCmd(M2006_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`
- 放在freeRTOS的线程中不断运行

**样例代码**
```c
uint32_t sample_period = 1; //采样时间1ms

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    M2006_RxUpdate(&M, hcan);
}

void m2006_pid_calc_thread( void const * argument ) 
{
    while (1) {
        M2006_CalcPid(&M);

        /* 1ms 运行一次 */
        /* 实测上面的那套控制参数搭配1ms的控制周期，可以有相当不错的控制效果 */
        osDelay( sample_period );
    }
}

void m2006_cmd_sending_thread( void const * argument )
{
    while ( 1 )
    {
        M2006_SendCmd( &M, M.volt[0], M.volt[1], M.volt[2], M.volt[3] );
        osDelay( sample_period );
    }
}
```

---

## 六、向电机发送指令

`void M2006_SendCmd(M2006_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);`

- 作用：向电机发送原始指令，数据范围是 -16384～16384

`void M2006_CmdVel(M2006_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4);`

- 向电机发送速度指令
- 单位是RPS（转每秒）

**样例代码**
```c
float ch[4];

void DR16_Callback(DR16_TypeDef* D) {
    DR16_MainTask(&D);
    DR16_MappingData(&D, ch, ch+1, ch+2, ch+3, 100);

    M2006_CmdVel(&M, ch[0], ch[1], ch[2], ch[3]);
}
```

---
