#ifndef __EC60_H
#define __EC60_H


#include "stm32f4xx_hal.h"


/* 电机返回机械转矩在0-8192之间 */
#define EC60_PPR				8192

#define MAX_VOLT				5000
#define MIN_VOLT				-5000


/* 位置控制PID参数 */
typedef struct {

	/* 位置式PID参数 */
	float kp;
	float ki;
	float kd;
	
	/* 增量式PID参数 */
	float a1;
	float a2;
	float a3;
	
	/* 误差 */
	float cur_err[4];
	float sum_err[4];
	float lst_err[4];
	float prv_err[4];
	
	/* 采样 */
	uint32_t sample_period;
	uint32_t tick;
	
	/* 输出限幅 */
	int16_t saturation;
	float ki_saturation;
	
} PID_TypeDef;

typedef struct {
	
	/* CAN */
	CAN_HandleTypeDef* motor_can;
	
	/* 电机的id */
	uint16_t motor_id_group;
	
	/* 电机的机械转角和转矩 */
	int16_t angle[4];
	
	int16_t torque[4];
	float vel[4]; //PRS
	
	/* 电机的PID位置控制器 */
	PID_TypeDef pid;
	
	/* 电机的设置位置 */
	int16_t loc_set[4];
	float vel_set[4];
	
	/* 电机的转向 */
	int16_t dir[4];
	
	/* 接受信号缓存 */
	uint8_t motor1_rxbuffer[8];
	uint8_t motor2_rxbuffer[8];
	uint8_t motor3_rxbuffer[8];
	uint8_t motor4_rxbuffer[8];
	uint8_t active_channel[4];
	
	/* 电机的电压输入 */
	int32_t volt[4];
	
} EC60_TypeDef;


/* 用户接口 */

/* 打开一个电机组 */
EC60_TypeDef EC60_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);

/* 放在1ms周期定时器中 */
void EC60_Update(EC60_TypeDef* M);

/* 放在HAL_CAN_RxFifo0MsgPendingCallback中 */
void EC60_RxUpdate(EC60_TypeDef* M, CAN_HandleTypeDef* hcan);

/* 主任务 */
void EC60_MainTask(EC60_TypeDef* M);

/* 设置电机转向 */
void EC60_SetDir(EC60_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4);

/* 设置位置式PID参数 */
void EC60_CtrlParams(EC60_TypeDef* M, float a1, float a2, float a3, uint32_t sample_period, int16_t output_saturation);

/* 使用CAN向电机发送命令 */
void EC60_SendCmd(EC60_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/* 设置电机转速 */
void EC60_CmdVel(EC60_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4);

#endif
