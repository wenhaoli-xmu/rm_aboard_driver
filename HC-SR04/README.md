# 平衡小车之家HC-SR04超声波传感器驱动

**当前版本** v1

**v1特性**
* 使用输入捕获来进行跳变沿的捕获
* 使用软件延时20us和GPIO_OUTPUT方式进行信息的发送

---

## 一、MX配置

**TX口GPIO配置**
![img1](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HC-SR04/img1.png)

**高级定时器input capture**
![img2](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HC-SR04/img2.png)
![img3](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HC-SR04/img3.png)
![img4](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HC-SR04/img4.png)
![img5](https://github.com/RainFromCN/rm_aboard_driver/blob/master/HC-SR04/img5.png)

---

## 二、创建一个SR04对象

`SR04_TypeDef SR04_Open(TIM_HandleTypeDef* htim, uint32_t tim_channel, GPIO_TypeDef* trig_port, uint16_t trig_pin);`
* `htim` 用于输入捕获的高级定时器
* `tim_channel` 用于输入捕获的高级定时器的通道
* `trig_port` 用于发送TX的PWM信号的GPIO引脚PORT
* `trig_pin` 用于发送TX的PWM信号的GPIO引脚的PIN

**样例代码**
```c
SR04_TypeDef S;

/* htim8：输入捕获定时器
 * TIM_CHANNEL_1：通道一作为输入捕获
 * GPIOE、GPIO_PIN_9：使用PE9接上超声波模块的TX口
 */
S = SR04_Open(&htim8, TIM_CHANNEL_1, GPIOE, GPIO_PIN_9);
```

---

## 三、移植IcUpdate函数

`void SR04_IcUpdate(SR04_TypeDef* S, TIM_HandleTypeDef* htim);`
* 将这个函数放在HAL_TIM_IC_CaptureCallback中运行

**样例代码**
```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	SR04_IcUpdate(&S, htim);
}
```

---

## 四、移植MainTask函数

`void SR04_Maintask(SR04_TypeDef* S);`
* 将其放在freertos的task中运行
* 运行完毕之后可以osDelay(60)，建议最少延时60ms之后再进行下一次检测
* 否则本次的测量会影响下一次的测量

**样例代码**
```c
void sr04_task(void const *argument) {
    while (1) {
		SR04_Maintask(&S);

        /* 采样周期60ms，推荐值 */
		osDelay(60);
	}
}
```

---

## 五、使用Callback函数

`void SR04_Callback(SR04_TypeDef* sr04);`
* 每次采集完一次超声波数据之后，会进入Callback函数

**样例代码**
```c
float x_vel[4]; //由超声波控制的电机速度
float y_vel[4]; //由超声波控制的电机速度
float imu_vel[4]; //由惯性测量单元控制的电机速度
float cmd_vel[4]; //由遥控器指令控制的电机速度

void SR04_Callback(SR04_TypeDef* sr04) {

    /* x方向上的超声波传感器 */
    if (sr04 == Sx) {
        static float vel;
        vel = 1 * sr04->dist;

        x_vel[0] = vel;
        x_vel[1] = -vel;
        x_vel[2] = vel;
        x_vel[3] = -vel;

        M2006_CmdVel(&M,\
            x_vel[0] + y_vel[0] + cmd_vel[0] + imu_vel[0],\
            x_vel[1] + y_vel[1] + cmd_vel[1] + imu_vel[1],\
            x_vel[2] + y_vel[2] + cmd_vel[2] + imu_vel[2],\
            x_vel[3] + y_vel[3] + cmd_vel[3] + imu_vel[3],\
        );
    }

    /* y方向上的超声波传感器 */
    else if (sr04 == Sy) {
        static float vel;
        vel = 1 * sr04->dist;

        y_vel[0] = vel;
        y_vel[1] = -vel;
        y_vel[2] = vel;
        y_vel[3] = -vel;

        M2006_CmdVel(&M,\
            x_vel[0] + y_vel[0] + cmd_vel[0] + imu_vel[0],\
            x_vel[1] + y_vel[1] + cmd_vel[1] + imu_vel[1],\
            x_vel[2] + y_vel[2] + cmd_vel[2] + imu_vel[2],\
            x_vel[3] + y_vel[3] + cmd_vel[3] + imu_vel[3],\
        );
    }
}
```

---
