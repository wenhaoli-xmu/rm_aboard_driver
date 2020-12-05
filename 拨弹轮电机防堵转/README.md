# 拨弹轮防堵转逻辑

---c
/* 电机防止堵转的变量定义 */
# define STALL_VOLT             (8000) //电机电压大于此值判为堵转
# define STALL_VEL              (5) //电机转速小于此值判为堵转
# define STALL_TIME             (100) //电机堵转时间超过此值则开启堵转程序
# define BACK_TIME              (500) //电机堵转后回转时间
# define FORWARD_TIME           (500) //电机堵转后回转后正传的时间

uint8_t is_motor_working = 0;
uint16_t stall_tick = 0;
uint16_t back_task = 0;
uint16_t forward_task = 0;

void spdm_task(void const *argument) {
	while (1) {
		if (abs(S.vel[0]) > STALL_VEL)
			is_motor_working = 1;

		if (back_task) {
			SPDM_CmdVel(&S, -1e6, 0.f, 0.f, 0.f);
			-- back_task;
			if (back_task == 0) forward_task = FORWARD_TIME;
		}
		else if (forward_task) {
			-- forward_task;
		}
		else {
			if (fabs(S.vel[0]) < STALL_VEL && is_motor_working && abs(S.volt[0]) > STALL_VOLT) {
				++ stall_tick;
				if (stall_tick > STALL_TIME) {
					back_task = BACK_TIME;
				}
			}
			else stall_tick = 0;
		}

		SPDM_MainTask(&S);
		osDelay(1);
	}
}
---