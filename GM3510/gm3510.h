#ifndef __GM3510_H
#define __GM3510_H


#include "stm32f4xx_hal.h"


/* ������ػ�еת����0-8192֮�� */
#define LOC_LOWER				0
#define LOC_UPPER				8192

#define MAX_VOLT				29000
#define MIN_VOLT				-29000


/* λ�ÿ���PID���� */
typedef struct {

	/* ���Ʋ��� */
	float kp;
	float ki;
	float kd;
	
	/* ��� */
	int16_t err[3];
	int32_t sum_err[3];
	int16_t prv_err[3];
	
	/* ���� */
	uint32_t sample_period;
	uint32_t tick;
	
	/* ����޷� */
	int16_t saturation;
	
} PID_TypeDef;

typedef struct {
	
	/* CAN */
	CAN_HandleTypeDef* motor_can;
	
	/* �����id */
	uint16_t motor_id_group;
	
	/* ����Ļ�еת�Ǻ�ת�� */
	int16_t angle[3];
	int16_t torque[3];
	
	/* �����PIDλ�ÿ����� */
	PID_TypeDef pid;
	
	/* ���������λ�� */
	int16_t loc_set[3];
	
	/* �����źŻ��� */
	uint8_t motor1_rxbuffer[8];
	uint8_t motor2_rxbuffer[8];
	uint8_t motor3_rxbuffer[8];
	uint8_t active_channel[3];
	
	/* ����ĵ�ѹ���� */
	int16_t volt[3];
	
} GM3510_TypeDef;


/* �û��ӿ� */

/* ��һ������� */
GM3510_TypeDef GM3510_Open(CAN_HandleTypeDef* hcan, uint16_t id_group);

/* ����1ms���ڶ�ʱ���� */
void GM3510_Update(GM3510_TypeDef* M);

/* ���ڽ����жϻص������� */
void GM3510_RxUpdate(GM3510_TypeDef* M, CAN_HandleTypeDef* hcan);

/* �ص����� */
void GM3510_Callback(GM3510_TypeDef* M);

/* ����PID���� */
void GM3510_SetPID(GM3510_TypeDef* M, float kp, float ki, float kd, uint32_t sample_period, int16_t output_saturation);

/* ʹ��CAN������������ */
void GM3510_SendCmd(GM3510_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3);

/* 设置电机转角 */
void GM3510_SetAngle(GM3510_TypeDef* M, float angle1, float angle2, float angle3);

#endif
