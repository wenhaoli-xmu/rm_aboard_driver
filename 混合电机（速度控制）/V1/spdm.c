#include <spdm.h>
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* 开启滤波器 */
static void SPDM_CANFilterEnable(CAN_HandleTypeDef* hcan) {
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

SPDM_TypeDef SPDM_Open(CAN_HandleTypeDef* hcan, uint16_t id_group) {
	
	SPDM_TypeDef tmp;
	
	memset(&tmp, 0, sizeof(SPDM_TypeDef));

	tmp.SPDM_can = hcan;
	tmp.SPDM_id_group = id_group;
	
	SPDM_CANFilterEnable(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
	
	return tmp;
}

void SPDM_GetRxMessage(uint32_t* pL, uint32_t* pH, uint8_t aData[]) {
    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & *pL) >> CAN_RDL0R_DATA0_Pos);
    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & *pL) >> CAN_RDL0R_DATA1_Pos);
    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & *pL) >> CAN_RDL0R_DATA2_Pos);
    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & *pL) >> CAN_RDL0R_DATA3_Pos);
    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & *pH) >> CAN_RDH0R_DATA4_Pos);
    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & *pH) >> CAN_RDH0R_DATA5_Pos);
    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & *pH) >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & *pH) >> CAN_RDH0R_DATA7_Pos);
}

/* 放在HAL_CAN_RxFifo0MsgPendingCallback中 */
void SPDM_RxUpdate(SPDM_TypeDef* M, CAN_HandleTypeDef* hcan) {

	static uint16_t tmp;
	if (M->motor_can != hcan) return;
	tmp = (CAN_RI0R_STID & M->SPDM_can->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	
	/* 第一个电机 */
	if (tmp == 0x201 || tmp == 0x205) {
		M->dataL_buf[0] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		M->dataH_buf[0] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		M->active_channel[0] = 1;
		SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
	}
	
	/* 第二个电机 */
	else if (tmp == 0x202 || tmp == 0x206) {
		M->dataL_buf[1] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		M->dataH_buf[1] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		M->active_channel[1] = 1;
		SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
	}
	
	/* 第三个电机 */
	else if (tmp == 0x203 || tmp == 0x207) {
		M->dataL_buf[2] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		M->dataH_buf[2] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		M->active_channel[2] = 1;
		SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
	}
	
	/* 第四个电机 */
	else if (tmp == 0x204 || tmp == 0x208) {
		M->dataL_buf[3] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		M->dataH_buf[3] = hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		M->active_channel[3] = 1;
		SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
	}
}

/* 主任务 */
void SPDM_MainTask(SPDM_TypeDef* M) {
	static float A, B;

	static float ratio;
	static float cur_err[4];
	static float uKd;
	static uint8_t data_buffer[8];

	static uint8_t i;

	if (M->active_channel[0] == 1) {
		M->active_channel[0] = 0;
		SPDM_GetRxMessage(M->dataL_buf, M->dataH_buf, data_buffer);
		M->angle[0] = (int16_t)(data_buffer[0] << 8 | data_buffer[1]);
		M->velocity[0] = (int16_t)(data_buffer[2] << 8 | data_buffer[3]);
	}
	
	if (M->active_channel[1] == 1) {
		M->active_channel[1] = 0;
		SPDM_GetRxMessage(M->dataL_buf + 1, M->dataH_buf + 1, data_buffer);
		M->angle[1] = (int16_t)(data_buffer[0] << 8 | data_buffer[1]);
		M->velocity[1] = (int16_t)(data_buffer[2] << 8 | data_buffer[3]);
	}
	
	if (M->active_channel[2] == 1) {
		M->active_channel[2] = 0;
		SPDM_GetRxMessage(M->dataL_buf + 2, M->dataH_buf + 2, data_buffer);
		M->angle[2] = (int16_t)(data_buffer[0] << 8 | data_buffer[1]);
		M->velocity[2] = (int16_t)(data_buffer[2] << 8 | data_buffer[3]);
	}
	
	if (M->active_channel[3] == 1) {
		M->active_channel[3] = 0;
		SPDM_GetRxMessage(M->dataL_buf + 3, M->dataH_buf + 3, data_buffer);
		M->angle[3] = (int16_t)(data_buffer[0] << 8 | data_buffer[1]);
		M->velocity[3] = (int16_t)(data_buffer[2] << 8 | data_buffer[3]);
	}

	M->vel[0] = M->velocity[0] / 60.f * M->dir[0];
	M->vel[1] = M->velocity[1] / 60.f * M->dir[1];
	M->vel[2] = M->velocity[2] / 60.f * M->dir[2];
	M->vel[3] = M->velocity[3] / 60.f * M->dir[3];
	
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

		A = M->pid.A[i];
		B = M->pid.B[i];

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

		/* 不完全微分 */
		uKd = M->pid.a3[i] * (cur_err[i] - M->pid.prv_err[i]) * (1 - M->pid.alpha[i]) + M->pid.alpha[i] * M->pid.last_uKd[i];

		/* 计算输出 */
		M->volt[i] = M->pid.a1[i] * cur_err[i] + M->pid.a2[i] * M->pid.sum_err[i] + uKd;

		M->pid.prv_err[i] = cur_err[i];
		M->pid.last_uKd[i] = uKd;
	}

	/* 输出限幅 */
	for (i = 0; i < 4; i ++) {

		if (M->volt[i] > M->pid.saturation[i]) {
			M->volt[i] = M->pid.saturation[i];
			M->pid.is_sat[i] = 1;
		}
		else M->pid.is_sat[i] = 0;


		if (M->volt[i] < -M->pid.saturation[i]) {
			M->volt[i] = -M->pid.saturation[i];
			M->pid.is_sat[i] = -1;
		}
		else M->pid.is_sat[i] = 0;
	}
	
	SPDM_SendCmd(M, M->volt[0], M->volt[1], M->volt[2], M->volt[3]);
}

void SPDM_CtrlParams(SPDM_TypeDef* M, float* kp, float* ki, float* kd, int16_t* output_saturation) {

    uint8_t i;

    for (i = 0; i < 4; i ++) {
        M->pid.a1[i] = kp[i];
        M->pid.a2[i] = ki[i];
        M->pid.a3[i] = kd[i];
        M->pid.A[i] = 0;
        M->pid.B[i] = 1e6;
        M->pid.alpha[i] = 0;
        M->pid.saturation[i] = output_saturation[i];
    }
}

void SPDM_ExCtrlParams(SPDM_TypeDef* M, float* A, float* B, float* alpha) {
	M->pid.A[i] = A[i];
	M->pid.B[i] = B[i];
	M->pid.alpha[i] = alpha[i];
}

void SPDM_CmdVel(SPDM_TypeDef* M, float vel_rps1, float vel_rps2, float vel_rps3, float vel_rps4) {
	
	M->vel_set[0] = vel_rps1;
	M->vel_set[1] = vel_rps2;
	M->vel_set[2] = vel_rps3;
	M->vel_set[3] = vel_rps4;
}

void SPDM_SendCmd(SPDM_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
	
	static uint32_t mailbox;
	static uint8_t data[8];
	static CAN_TxHeaderTypeDef header;
	
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

void SPDM_SetDir(SPDM_TypeDef* M, int16_t dir1, int16_t dir2, int16_t dir3, int16_t dir4) {
	M->dir[0] = dir1;
	M->dir[1] = dir2;
	M->dir[2] = dir3;
	M->dir[3] = dir4;
}
