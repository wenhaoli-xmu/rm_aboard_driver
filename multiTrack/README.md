# 多路循迹模块驱动

**当前版本**V1

**V1特性**
* 可以同时驱动多路循迹模块
* 使用freeRTOS进行采样时间控制

---

## 一、MX配置

![img1](https://github.com/RainFromCN/rm_aboard_driver/tree/master/multiTrack/img1)

---

## 二、创建循迹模块

`#define TRACK_MODULE_MAX               (10)`
* 设置支持循迹模块的最大数量

`TRACK_TypeDef TRACK_Open();`
* 创建一个循迹模块组

`void TRACK_AppendModule(TRACK_TypeDef* T, GPIO_TypeDef* port, uint16_t pin);`
* 向循迹模块组中后缀循迹模块
* `port` 循迹模块的GPIO_PORT
* `pin` 循迹模块的GPIO_PIN

**样例代码**
```c
TRACK_TypeDef T;

T = TRACK_Open();

/* 向循迹模块组后缀PI0 */
TRACK_AppendModule(&T, GPIOI, GPIO_PIN_0);
```

---

## 三、移植MainTask函数

**样例代码**
```c
void track_task(void const *argument) {
    while (1) {
        TRACK_MainTask(&T);
        osDelay(1);
    }
}
```

---