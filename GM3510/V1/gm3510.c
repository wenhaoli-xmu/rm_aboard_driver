#include "gm3510.h"


/* 开启滤波器 */
static void GM3510_CANFilterEnable(CAN_HandleTypeDef* hcan) {
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

GM3510_TypeDef GM3510_Open(CAN_HandleTypeDef* hcan, uint16_t id_group) {
	
	GM3510_TypeDef tmp;
	
	tmp.motor_can = hcan;
	tmp.motor_id_group = id_group;
	
	GM3510_CANFilterEnable(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(hcan);
	
	return tmp;
}

/* 放在CAN接受中断中 */
void GM3510_RxUpdate(GM3510_TypeDef* M, CAN_HandleTypeDef* hcan) {
	CAN_RxHeaderTypeDef rx_header;
	
	rx_header.StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	
	uint16_t tmp = rx_header.StdId;
	
	/* 第一个电机 */
	if (tmp == 0x205) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, M->motor1_rxbuffer);
		M->active_channel[0] = 1;
	}
	
	/* 第二个电机 */
	else if (tmp == 0x206) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, M->motor2_rxbuffer);
		M->active_channel[1] = 1;
	}
	
	/* 第三个电机 */
	else if (tmp == 0x207) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, M->motor3_rxbuffer);
		M->active_channel[2] = 1;
	}
	
}

/* 放在1ms周期的定时器中 */
void GM3510_Update(GM3510_TypeDef* M) {
	M->pid.tick += 1;
}

/* 回调函数 */
void GM3510_Callback(GM3510_TypeDef* M) {
	
	if (M->pid.tick >= M->pid.sample_period) M->pid.tick = 0;
	else return ;
	
	/* 通过CAN获取电机的实际旋转情况 */
	if (M->active_channel[0] == 1) {
		M->active_channel[0] = 0;
		
		M->angle[0] = (int16_t)(M->motor1_rxbuffer[0] << 8 | M->motor1_rxbuffer[1]);
		M->torque[0] = (int16_t)(M->motor1_rxbuffer[2] << 8 | M->motor1_rxbuffer[3]);
	}
	
	if (M->active_channel[1] == 1) {
		M->active_channel[1] = 0;
		
		M->angle[1] = (int16_t)(M->motor2_rxbuffer[0] << 8 | M->motor2_rxbuffer[1]);
		M->torque[1] = (int16_t)(M->motor2_rxbuffer[2] << 8 | M->motor2_rxbuffer[3]);
	}
	
	if (M->active_channel[2] == 1) {
		M->active_channel[2] = 0;
		
		M->angle[2] = (int16_t)(M->motor3_rxbuffer[0] << 8 | M->motor3_rxbuffer[1]);
		M->torque[2] = (int16_t)(M->motor3_rxbuffer[2] << 8 | M->motor3_rxbuffer[3]);
	}
	
	/* 电机的PID控制 */
	M->pid.err[0] = M->loc_set[0] - M->angle[0];
	M->pid.err[1] = M->loc_set[1] - M->angle[1];
	M->pid.err[2] = M->loc_set[2] - M->angle[2];
	
	M->pid.sum_err[0] += M->pid.err[0];
	M->pid.sum_err[1] += M->pid.err[1];
	M->pid.sum_err[2] += M->pid.err[2];
	
	M->volt[0] = M->pid.kp * M->pid.err[0] + M->pid.ki * M->pid.sum_err[0] + M->pid.kd * M->pid.prv_err[0];
	M->volt[1] = M->pid.kp * M->pid.err[1] + M->pid.ki * M->pid.sum_err[1] + M->pid.kd * M->pid.prv_err[1];
	M->volt[2] = M->pid.kp * M->pid.err[2] + M->pid.ki * M->pid.sum_err[2] + M->pid.kd * M->pid.prv_err[2];
	
	M->pid.prv_err[0] = M->pid.err[0];
	M->pid.prv_err[1] = M->pid.err[1];
	M->pid.prv_err[2] = M->pid.err[2];
	
	/* 输出限幅 */
	if (M->volt[0] > M->pid.saturation) M->volt[0] = M->pid.saturation;
	if (M->volt[0] < -M->pid.saturation) M->volt[0] = -M->pid.saturation;
	if (M->volt[1] > M->pid.saturation) M->volt[1] = M->pid.saturation;
	if (M->volt[1] < -M->pid.saturation) M->volt[1] = -M->pid.saturation;
	if (M->volt[2] > M->pid.saturation) M->volt[2] = M->pid.saturation;
	if (M->volt[2] < -M->pid.saturation) M->volt[2] = -M->pid.saturation;
	
	GM3510_SendCmd(M, M->volt[0], M->volt[1], M->volt[2]);
}

void GM3510_SetPID(GM3510_TypeDef* M, float kp, float ki, float kd, uint32_t sample_period, int16_t output_saturation) {
	M->pid.kp = kp;
	M->pid.ki = ki;
	M->pid.kd = kd;
	M->pid.sample_period = sample_period;
	M->pid.saturation = output_saturation;
}

void GM3510_SetAngle(GM3510_TypeDef* M, float angle1, float angle2, float angle3) {
	
	static float ratio = 8192 / 360;
	
	M->loc_set[0] = ratio * angle1;
	M->loc_set[1] = ratio * angle2;
	M->loc_set[2] = ratio * angle3;
}

void GM3510_SendCmd(GM3510_TypeDef* M, int16_t motor1, int16_t motor2, int16_t motor3) {
	
	static uint32_t mailbox;
	static uint8_t data[8];
	static CAN_TxHeaderTypeDef header;
	
	if (HAL_CAN_IsTxMessagePending(M->motor_can, mailbox)) return;
	
	header.StdId = M->motor_id_group;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 0x08;
	
	data[0] = motor1 >> 8;
	data[1] = motor1;
	data[2] = motor2 >> 8;
	data[3] = motor2;
	data[4] = motor3 >> 8;
	data[5] = motor3;
	data[6] = 0;
	data[7] = 0;
	
	HAL_CAN_AddTxMessage(M->motor_can, &header, data, &mailbox);
}
