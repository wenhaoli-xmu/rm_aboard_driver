#ifndef __GM6020_H
#define __GM6020_H


#include "stm32f4xx_hal.h"


/* 电机返回机械转矩在0-8192之间 */
#define GM6020_IDGROUP1         0x1FF
#define GM6020_IDGROUP2         0x2FF

#define LOC_LOWER				0
#define LOC_UPPER				8192

#define MAX_VOLT				30000
#define MIN_VOLT				-30000


/* 位置控制PID参数 */
typedef struct {

	/* 控制参数 */
	float kp;
	float ki;
	float kd;
	
	/* 误差 */
	int16_t err[4];
	int32_t sum_err[4];
	int16_t prv_err[4];
	
	/* 采样 */
	uint32_t sample_period;
	uint32_t tick;
	
	/* 输出限幅 */
	int16_t saturation;
	
} PID_TypeDef;

typedef struct {
	
	/* CAN */
	CAN_HandleTypeDef* motor_can;
	
	/* 电机的id */
	uint16_t motor_id_group;
	
	/* 电机的机械转角和转矩 */
	int16_t angle[4];
	int16_t torque[4];
	
	/* 电机的PID位置控制器 */
	PID_TypeDef pid;
	
	/* 电机的设置位置 */
	int16_t loc_set[4];
	
	/* 接受信号缓存 */
	uint8_t motor1_rxbuffer[8];
	uint8_t motor2_rxbuffer[8];
	uint8_t motor3_rxbuffer[8];
	uint8_t motor4_rxbuffer[8];
	uint8_t active_channel[4];
	
	/* 电机的电压输入 */
	int16_t volt[4];

} GM6020_TypeDef;


/* 用户接口 */

/* 打开一个电机组 */
GM6020_TypeDef GM6020_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);

/* 放在1ms周期定时器中 */
void GM6020_Update(GM6020_TypeDef* M);

/* 放在HAL_CAN_RxFifo0MsgPendingCallback中断回调函数中 */
void GM6020_RxUpdate(GM6020_TypeDef* M, CAN_HandleTypeDef* hcan);

/* 电机控制的主任务 */
void GM6020_MainTask(GM6020_TypeDef* M);

/* 设置PID参数 */
void GM6020_SetPID(GM6020_TypeDef* M, float kp, float ki, float kd, uint32_t sample_period, int16_t output_saturation);

/* 使用CAN向电机发送命令 */
void GM6020_SendCmd(GM6020_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/* 设置电机转角 */
void GM6020_SetAngle(GM6020_TypeDef* M, float angle1, float angle2, float angle3, float angle4);

#endif
