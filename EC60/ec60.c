#include "ec60.h"
#include "stdlib.h"


/* 开启滤波器 */
static void EC60_CANFilterEnable(CAN_HandleTypeDef* hcan) {
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

EC60_TypeDef EC60_Open(CAN_HandleTypeDef* hcan, uint16_t id_group) {
	
	EC60_TypeDef tmp;
	
	tmp.motor_can = hcan;
	tmp.motor_id_group = id_group;
	
	EC60_CANFilterEnable(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
	
	return tmp;
}

/* 放在HAL_CAN_RxFifo0MsgPendingCallback中 */
void EC60_RxUpdate(EC60_TypeDef* M, CAN_HandleTypeDef* hcan) {
	CAN_RxHeaderTypeDef rx_header;
	
	rx_header.StdId = (CAN_RI0R_STID & M->motor_can->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	
	uint16_t tmp = rx_header.StdId;
	
	/* 第一个电机 */
	if (tmp == 0x201) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor1_rxbuffer);
		M->active_channel[0] = 1;
	}
	
	/* 第二个电机 */
	else if (tmp == 0x202) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor2_rxbuffer);
		M->active_channel[1] = 1;
	}
	
	/* 第三个电机 */
	else if (tmp == 0x203) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor3_rxbuffer);
		M->active_channel[2] = 1;
	}
	
	/* 第四个电机 */
	else if (tmp == 0x204) {
		HAL_CAN_GetRxMessage(M->motor_can, CAN_RX_FIFO0, &rx_header, M->motor4_rxbuffer);
		M->active_channel[3] = 1;
	}
}

/* 放在1ms周期的定时器中 */
void EC60_Update(EC60_TypeDef* M) {
	M->pid.tick += 1;
}

static float ABS(float f) {
	return f > 0 ? f : -f;
}

/* 主任务 */
void EC60_MainTask(EC60_TypeDef* M) {
	
	static int16_t prv_angle[4] = {0, 0, 0, 0};
	static float vel[3];
	static double ratio;
	static float minvel;
	static uint8_t idx;
	
	if (M->pid.tick >= M->pid.sample_period) M->pid.tick = 0;
	else return ;
	
	/* 通过CAN获取电机的实际旋转情况 */
	if (M->active_channel[0] == 1) {
		M->active_channel[0] = 0;
		M->angle[0] = (int16_t)(M->motor1_rxbuffer[0] << 8 | M->motor1_rxbuffer[1]);
	}
	
	if (M->active_channel[1] == 1) {
		M->active_channel[1] = 0;	
		M->angle[1] = (int16_t)(M->motor2_rxbuffer[0] << 8 | M->motor2_rxbuffer[1]);
	}
	
	if (M->active_channel[2] == 1) {
		M->active_channel[2] = 0;
		M->angle[2] = (int16_t)(M->motor3_rxbuffer[0] << 8 | M->motor3_rxbuffer[1]);
	}
	
	if (M->active_channel[3] == 1) {
		M->active_channel[3] = 0;
		M->angle[3] = (int16_t)(M->motor4_rxbuffer[0] << 8 | M->motor4_rxbuffer[1]);
	}
	
	ratio = (1000. / M->pid.sample_period) / 8192.;
	
	for (int i = 0; i < 4; i ++) {
		vel[0] = (double)(M->angle[i] - prv_angle[i] + 8192) * ratio;
		vel[1] = (double)(M->angle[i] - prv_angle[i]) * ratio;
		vel[2] = (double)(M->angle[i] - prv_angle[i] - 8192) * ratio;
		
		minvel = 1e6;
		for (int j = 0; j < 3; j ++) {
			if (ABS(vel[j]) < minvel) {
				minvel = ABS(vel[j]);
				idx = j;
			}
		}
		
		M->vel[i] = vel[idx];
		M->vel[i] *= M->dir[i];
		
		prv_angle[i] = M->angle[i];
	}
	
	/* 电机的PID控制 */
	M->pid.cur_err[0] = M->vel_set[0] - M->vel[0];
	M->pid.cur_err[1] = M->vel_set[1] - M->vel[1];
	M->pid.cur_err[2] = M->vel_set[2] - M->vel[2];
	M->pid.cur_err[3] = M->vel_set[3] - M->vel[3];
	
	for (int i = 0; i < 4; i ++) {
		if (M->pid.cur_err[i] < (float)0.02 && M->pid.cur_err[i] > (float)-0.02) M->pid.cur_err[i] = 0;
	}
	
	M->pid.sum_err[0] += M->pid.cur_err[0];
	M->pid.sum_err[1] += M->pid.cur_err[1];
	M->pid.sum_err[2] += M->pid.cur_err[2];
	M->pid.sum_err[3] += M->pid.cur_err[3];
	
	M->volt[0] = M->pid.a1 * M->pid.cur_err[0] + M->pid.a2 * M->pid.sum_err[0] + M->pid.a3 * (M->pid.cur_err[0] - M->pid.prv_err[0]);
	M->volt[1] = M->pid.a1 * M->pid.cur_err[1] + M->pid.a2 * M->pid.sum_err[1] + M->pid.a3 * (M->pid.cur_err[1] - M->pid.prv_err[1]);
	M->volt[2] = M->pid.a1 * M->pid.cur_err[2] + M->pid.a2 * M->pid.sum_err[2] + M->pid.a3 * (M->pid.cur_err[2] - M->pid.prv_err[2]);
	M->volt[3] = M->pid.a1 * M->pid.cur_err[3] + M->pid.a2 * M->pid.sum_err[3] + M->pid.a3 * (M->pid.cur_err[3] - M->pid.prv_err[3]);
	
	M->pid.prv_err[0] = M->pid.cur_err[0];
	M->pid.prv_err[1] = M->pid.cur_err[1];
	M->pid.prv_err[2] = M->pid.cur_err[2];
	M->pid.prv_err[3] = M->pid.cur_err[3];
	
	/* 输出限幅 */
	if (M->volt[0] > M->pid.saturation) M->volt[0] = M->pid.saturation;
	if (M->volt[0] < -M->pid.saturation) M->volt[0] = -M->pid.saturation;
	if (M->volt[1] > M->pid.saturation) M->volt[1] = M->pid.saturation;
	if (M->volt[1] < -M->pid.saturation) M->volt[1] = -M->pid.saturation;
	if (M->volt[2] > M->pid.saturation) M->volt[2] = M->pid.saturation;
	if (M->volt[2] < -M->pid.saturation) M->volt[2] = -M->pid.saturation;
	if (M->volt[3] > M->pid.saturation) M->volt[3] = M->pid.saturation;
	if (M->volt[3] < -M->pid.saturation) M->volt[3] = -M->pid.saturation;
	
	EC60_SendCmd(M, M->volt[0], M->volt[1], M->volt[2], M->volt[3]);
}

void EC60_CtrlParams(EC60_TypeDef* M, float a1, float a2, float a3, uint32_t sample_period, int16_t output_saturation) {
	M->pid.a1 = a1;
	M->pid.a2 = a2;
	M->pid.a3 = a3;
	M->pid.sample_period = sample_period;
	M->pid.saturation = output_saturation;
}

void EC60_CmdVel(EC60_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4) {
	
	M->vel_set[0] = vel_rps1;
	M->vel_set[1] = vel_rps2;
	M->vel_set[2] = vel_rps3;
	M->vel_set[3] = vel_rps4;
}

void EC60_SendCmd(EC60_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
	
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

void EC60_SetDir(EC60_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4) {
	M->dir[0] = dir1;
	M->dir[1] = dir2;
	M->dir[2] = dir3;
	M->dir[3] = dir4;
}
