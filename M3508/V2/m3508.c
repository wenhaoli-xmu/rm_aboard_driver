#include <m3508.h>
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* 是否使用ENCODER测量速度 */
//#define USE_ENCODER				(0)

/* 是否使用变速积分器 */
#define USE_VARINT				(0)

/* 开启滤波器 */
static void M3508_CANFilterEnable(CAN_HandleTypeDef* hcan) {
	CAN_FilterTypeDef CAN_FilterConfigStructure;

	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
	CAN_FilterConfigStructure.SlaveStartFilterBank = 27;

	CAN_FilterConfigStructure.FilterBank = 0;

	HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure);
}

M3508_TypeDef M3508_Open(CAN_HandleTypeDef* hcan, uint16_t id_group) {
	
	M3508_TypeDef tmp;
	
	memset(&tmp, 0, sizeof(M3508_TypeDef));

	tmp.motor_can = hcan;
	tmp.motor_id_group = id_group;
	
	M3508_CANFilterEnable(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
	
	return tmp;
}

/* 放在HAL_CAN_RxFifo0MsgPendingCallback中 */
void M3508_RxUpdate(M3508_TypeDef* M, CAN_HandleTypeDef* hcan) {
	static CAN_RxHeaderTypeDef rx_header;
	static uint16_t tmp;
	
	rx_header.StdId = (CAN_RI0R_STID & M->motor_can->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	tmp = rx_header.StdId;
	
	/* 第一个电机 */
	if (tmp == 0x201 || tmp == 0x205) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor1_rxbuffer);
		M->active_channel[0] = 1;
	}
	
	/* 第二个电机 */
	else if (tmp == 0x202 || tmp == 0x206) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor2_rxbuffer);
		M->active_channel[1] = 1;
	}
	
	/* 第三个电机 */
	else if (tmp == 0x203 || tmp == 0x207) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor3_rxbuffer);
		M->active_channel[2] = 1;
	}
	
	/* 第四个电机 */
	else if (tmp == 0x204 || tmp == 0x208) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor4_rxbuffer);
		M->active_channel[3] = 1;
	}
}

#ifdef USE_ENCODER

static float ABS(float f) {
	return f > 0 ? f : -f;
}

#endif

/* 主任务 */
void M3508_MainTask(M3508_TypeDef* M) {
	
#ifdef USE_VARINT
	static float A, B;
#else
	static float int_thresh = 0.7f;
#endif

	static float ratio;
	static float cur_err[4];

#ifdef USE_ENCODER

	static int16_t prv_angle[4] = {0, 0, 0, 0};
	static float vel[3];
	static double ratio;
	static float minvel;
	static uint8_t idx;
	static uint8_t j;

#endif

	static uint8_t i;
	
	/* 通过CAN获取电机的实际旋转情况 */
	if (M->active_channel[0] == 1) {
		M->active_channel[0] = 0;
		M->angle[0] = (int16_t)(M->motor1_rxbuffer[0] << 8 | M->motor1_rxbuffer[1]);
		M->velocity[0] = (int16_t)(M->motor1_rxbuffer[2] << 8 | M->motor1_rxbuffer[3]);
	}
	
	if (M->active_channel[1] == 1) {
		M->active_channel[1] = 0;	
		M->angle[1] = (int16_t)(M->motor2_rxbuffer[0] << 8 | M->motor2_rxbuffer[1]);
		M->velocity[1] = (int16_t)(M->motor2_rxbuffer[2] << 8 | M->motor2_rxbuffer[3]);
	}
	
	if (M->active_channel[2] == 1) {
		M->active_channel[2] = 0;
		M->angle[2] = (int16_t)(M->motor3_rxbuffer[0] << 8 | M->motor3_rxbuffer[1]);
		M->velocity[2] = (int16_t)(M->motor3_rxbuffer[2] << 8 | M->motor3_rxbuffer[3]);
	}
	
	if (M->active_channel[3] == 1) {
		M->active_channel[3] = 0;
		M->angle[3] = (int16_t)(M->motor4_rxbuffer[0] << 8 | M->motor4_rxbuffer[1]);
		M->velocity[3] = (int16_t)(M->motor4_rxbuffer[2] << 8 | M->motor4_rxbuffer[3]);
	}

#ifdef USE_ENCODER

	ratio = (1000. / M->pid.sample_period) / 8192.;

	for (i = 0; i < 4; i ++) {
		vel[0] = (double)(M->angle[i] - prv_angle[i] + 8192) * ratio;
		vel[1] = (double)(M->angle[i] - prv_angle[i]) * ratio;
		vel[2] = (double)(M->angle[i] - prv_angle[i] - 8192) * ratio;

		minvel = 1e6;
		for (j = 0; j < 3; j ++) {
			if (ABS(vel[j]) < minvel) {
				minvel = ABS(vel[j]);
				idx = j;
			}
		}

		M->vel[i] = vel[idx];
		M->vel[i] *= M->dir[i];

		prv_angle[i] = M->angle[i];
	}

#else

	M->vel[0] = M->velocity[0] / 60.f * M->dir[0];
	M->vel[1] = M->velocity[1] / 60.f * M->dir[1];
	M->vel[2] = M->velocity[2] / 60.f * M->dir[2];
	M->vel[3] = M->velocity[3] / 60.f *  M->dir[3];

#endif
	
	/* 计算误差 */
	M->pid.cur_err[0] = M->vel_set[0] - M->vel[0];
	M->pid.cur_err[1] = M->vel_set[1] - M->vel[1];
	M->pid.cur_err[2] = M->vel_set[2] - M->vel[2];
	M->pid.cur_err[3] = M->vel_set[3] - M->vel[3];

	cur_err[0] = M->pid.cur_err[0];
	cur_err[1] = M->pid.cur_err[1];
	cur_err[2] = M->pid.cur_err[2];
	cur_err[3] = M->pid.cur_err[3];
	
	for (i = 0; i < 4; i ++) {

#ifdef USE_VARINT
		A = M->pid.A;
		B = M->pid.B;

		/* 钳制积分 */
		if ((M->pid.is_sat[i] == 1 && cur_err[i] > 0) || (M->pid.is_sat[i] == -1 && cur_err[i] < 0)) ;

		/* 变速积分 */
		else {
			if (fabs(cur_err[i]) <= B) {
				M->pid.sum_err[i] += cur_err[i];
			}
			else if (fabs(cur_err[i]) >= A + B) {
				M->pid.sum_err[i] = 0;
			}
			else {
				ratio = (A + B - fabs(cur_err[i])) / A;
				M->pid.sum_err[i] += ratio * cur_err[i];
			}
		}

#else
		/* 钳制积分 */
		if ((M->pid.is_sat[i] == 1 && cur_err[i] > 0) || (M->pid.is_sat[i] == -1 && cur_err[i] < 0)) ;

		/* 积分分离 */
		else {
			ratio = M->vel_set[i] / M->vel[i];
			if (ratio > int_thresh && ratio < (2.f - int_thresh))
				M->pid.sum_err[i] += cur_err[i];
			else
				M->pid.sum_err[i] = 0;
		}

#endif


		/* 不完全微分 */
		M->pid.dif_err[i] = 0.7 * M->pid.dif_err[i] + 0.3 * (cur_err[i] - M->pid.prv_err[i]);

		/* 计算输出 */
		M->volt[i] = M->pid.a1 * cur_err[i] + M->pid.a2 * M->pid.sum_err[i] + M->pid.a3 * M->pid.dif_err[i];

		M->pid.prv_err[i] = cur_err[i];
	}

	/* 输出限幅 */
	for (i = 0; i < 4; i ++) {

		if (M->volt[i] > M->pid.saturation) {
			M->volt[i] = M->pid.saturation;
			M->pid.is_sat[i] = 1;
		}
		else M->pid.is_sat[i] = 0;


		if (M->volt[i] < -M->pid.saturation) {
			M->volt[i] = -M->pid.saturation;
			M->pid.is_sat[i] = -1;
		}
		else M->pid.is_sat[i] = 0;
	}
	
	M3508_SendCmd(M, M->volt[0], M->volt[1], M->volt[2], M->volt[3]);
}

void M3508_CtrlParams(M3508_TypeDef* M, float kp, float ki, float kd, float A, float B, int16_t output_saturation) {
	M->pid.a1 = kp;
	M->pid.a2 = ki;
	M->pid.a3 = kd;
	M->pid.A = A;
	M->pid.B = B;
	M->pid.saturation = output_saturation;
}

void M3508_CmdVel(M3508_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4) {
	
	M->vel_set[0] = vel_rps1;
	M->vel_set[1] = vel_rps2;
	M->vel_set[2] = vel_rps3;
	M->vel_set[3] = vel_rps4;
}

void M3508_SendCmd(M3508_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
	
	static uint32_t mailbox;
	static uint8_t data[8];
	static CAN_TxHeaderTypeDef header;
	
	static float ratio;
	static int16_t maxmotor;
	static int16_t thresh = MAX_VOLT;
	
	maxmotor = abs(motor1);
	if (abs(motor2) > maxmotor) maxmotor = abs(motor2);
	if (abs(motor3) > maxmotor) maxmotor = abs(motor3);
	if (abs(motor4) > maxmotor) maxmotor = abs(motor4);
	if (maxmotor > thresh) {
		ratio = (float)thresh / (float)maxmotor;
		motor1 *= ratio;
		motor2 *= ratio;
		motor3 *= ratio;
		motor4 *= ratio;
	}
	
	if (HAL_CAN_IsTxMessagePending(M->motor_can, mailbox)) return;
	
	header.StdId = M->motor_id_group;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 0x08;
	
	motor1 *= M->dir[0];
	motor2 *= M->dir[1];
	motor3 *= M->dir[2];
	motor4 *= M->dir[3];
	
	data[0] = motor1 >> 8;
	data[1] = motor1;
	data[2] = motor2 >> 8;
	data[3] = motor2;
	data[4] = motor3 >> 8;
	data[5] = motor3;
	data[6] = motor4 >> 8;
	data[7] = motor4;
	
	HAL_CAN_AddTxMessage(M->motor_can, &header, data, &mailbox);
}

void M3508_SetDir(M3508_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4) {
	M->dir[0] = dir1;
	M->dir[1] = dir2;
	M->dir[2] = dir3;
	M->dir[3] = dir4;
}
