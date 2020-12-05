/*
 * imu.c
 *
 *  Created on: 2020年10月16日
 *      Author: 哈理工RM电控组
 */

#include "imu.h"
#include <math.h>
#include <string.h>

struct {
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;

	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data;

static uint8_t ist_buff[6];
static uint8_t mpu_buff[14];

/* 从MPU的寄存器中读取一个数据 */
uint8_t MPU_ReadByte(IMU_TypeDef* I, uint8_t const reg) {
	MPU_NSS_LOW;
	I->tx = reg | 0x80;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	MPU_NSS_HIGH;
	return I->rx;
}

/* 向MPU的寄存器中写入一个数据 */
uint8_t MPU_WriteByte(IMU_TypeDef* I, uint8_t const reg, uint8_t const data) {
	MPU_NSS_LOW;
	I->tx = reg & 0x7f;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	I->tx = data;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	MPU_NSS_HIGH;
	return 0;
}

/* 从MPU的寄存器中读取多个数据 */
uint8_t MPU_ReadBytes(IMU_TypeDef* I, uint8_t const regAddr, uint8_t* pData, uint8_t len) {
	MPU_NSS_LOW;
	I->tx = regAddr | 0x80;
	I->tx_buff[0] = I->tx;
	HAL_SPI_TransmitReceive(I->spi, &I->tx, &I->rx, 1, 55);
	HAL_SPI_TransmitReceive(I->spi, I->tx_buff, pData, len, 55);
	MPU_NSS_HIGH;
	return 0;
}

/* 用来快速计算 1/sqrt(x) */
static float inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* 设置imu的角速度测量范围 */
static uint8_t mpu_set_gyro_fsr(IMU_TypeDef* I, uint8_t fsr) {
	return MPU_WriteByte(I, MPU6500_GYRO_CONFIG, fsr << 3);
}

/* 设置imu的加速度测量范围 */
static uint8_t mpu_set_accel_fsr(IMU_TypeDef* I, uint8_t fsr) {
	return MPU_WriteByte(I, MPU6500_ACCEL_CONFIG, fsr << 3);
}

/* 向ist的寄存器写入数据 */
static void ist_reg_write_by_mpu(IMU_TypeDef* I, uint8_t addr, uint8_t data) {
	MPU_WriteByte(I, MPU6500_I2C_SLV1_CTRL, 0x00);
	HAL_Delay(2);
	MPU_WriteByte(I, MPU6500_I2C_SLV1_REG, addr);
	HAL_Delay(2);
	MPU_WriteByte(I, MPU6500_I2C_SLV1_DO, data);
	HAL_Delay(2);
	/* turn on slave 1 with one byte transmitting */
	MPU_WriteByte(I, MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	/* wait longer to ensure the data is transmitted from slave 1 */
	HAL_Delay(10);
}

/* 从ist的寄存器读取数据 */
static uint8_t ist_reg_read_by_mpu(IMU_TypeDef* I, uint8_t addr) {
    uint8_t retval;
    MPU_WriteByte(I, MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    MPU_WriteByte(I, MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    retval = MPU_ReadByte(I, MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    MPU_WriteByte(I, MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    return retval;
}

static void mpu_master_i2c_auto_read_config(IMU_TypeDef* I, uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /*
	   * configure the device address of the IST8310
     * use slave1, auto transmit single measure mode
	   */
	MPU_WriteByte(I, MPU6500_I2C_SLV1_ADDR, device_address);
	HAL_Delay(2);
    MPU_WriteByte(I, MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    MPU_WriteByte(I, MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* use slave0,auto read data */
    MPU_WriteByte(I, MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    MPU_WriteByte(I, MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* every eight mpu6500 internal samples one i2c master read */
    MPU_WriteByte(I, MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* enable slave 0 and 1 access delay */
    MPU_WriteByte(I, MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* enable slave 1 auto transmit */
    MPU_WriteByte(I, MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    HAL_Delay(6);
    /* enable slave 0 with data_num bytes reading */
    MPU_WriteByte(I, MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

/* 初始化ist模块 */
uint8_t ist8310_init(IMU_TypeDef* I) {
	/* enable iic master mode */
	MPU_WriteByte(I, MPU6500_USER_CTRL, 0x30);
	HAL_Delay(10);
	/* enable iic 400khz */
	MPU_WriteByte(I, MPU6500_I2C_MST_CTRL, 0x0d);
	HAL_Delay(10);

	/* turn on slave 1 for ist write and slave 4 to ist read */
	MPU_WriteByte(I, MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
	HAL_Delay(10);
	MPU_WriteByte(I, MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
	HAL_Delay(10);

	/* IST8310_R_CONFB 0x01 = device rst */
	ist_reg_write_by_mpu(I, IST8310_R_CONFB, 0x01);
	HAL_Delay(10);
	if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(I, IST8310_WHO_AM_I))
		return 1;

	/* soft reset */
	ist_reg_write_by_mpu(I, IST8310_R_CONFB, 0x01);
	HAL_Delay(10);

	/* config as ready mode to access register */
	ist_reg_write_by_mpu(I, IST8310_R_CONFA, 0x00);
	if (ist_reg_read_by_mpu(I, IST8310_R_CONFA) != 0x00)
		return 2;
	HAL_Delay(10);

	/* normal state, no int */
	ist_reg_write_by_mpu(I, IST8310_R_CONFB, 0x00);
	if (ist_reg_read_by_mpu(I, IST8310_R_CONFB) != 0x00)
		return 3;
	HAL_Delay(10);

	/* config low noise mode, x,y,z axis 16 time 1 avg */
	ist_reg_write_by_mpu(I, IST8310_AVGCNTL, 0x24); //100100
	if (ist_reg_read_by_mpu(I, IST8310_AVGCNTL) != 0x24)
		return 4;
	HAL_Delay(10);

	/* Set/Reset pulse duration setup,normal mode */
	ist_reg_write_by_mpu(I, IST8310_PDCNTL, 0xc0);
	if (ist_reg_read_by_mpu(I, IST8310_PDCNTL) != 0xc0)
		return 5;
	HAL_Delay(10);

	/* turn off slave1 & slave 4 */
	MPU_WriteByte(I, MPU6500_I2C_SLV1_CTRL, 0x00);
	HAL_Delay(10);
	MPU_WriteByte(I, MPU6500_I2C_SLV4_CTRL, 0x00);
	HAL_Delay(10);

	/* configure and turn on slave 0 */
	mpu_master_i2c_auto_read_config(I, IST8310_ADDRESS, IST8310_R_XL, 0x06);
	HAL_Delay(100);
	return 0;
}

/* 从ist单元获取数据 */
void ist8310_get_data(IMU_TypeDef* I, uint8_t* buff) {
    MPU_ReadBytes(I, MPU6500_EXT_SENS_DATA_00, buff, 6);
}

void IMU_GetData(IMU_TypeDef* I) {
	MPU_ReadBytes(I, MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_get_data(I, ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&I->ax, &mpu_data.ax, 6 * sizeof(int16_t));

    I->temp = 21 + mpu_data.temp / 333.87f;

	/* 2000dps -> rad/s */
    I->wx   = mpu_data.gx / 16.384f / 57.3f;
    I->wy   = mpu_data.gy / 16.384f / 57.3f;
    I->wz   = mpu_data.gz / 16.384f / 57.3f;
}

/* 从MPU6500中获取偏移数据 */
static void MPU_OffsetCall(IMU_TypeDef* I) {
	uint16_t i;
	for (i = 0; i < 300; i ++) {
		MPU_ReadBytes(I, MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		HAL_Delay(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gx_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

void IMU_InitQuaternion(IMU_TypeDef* I) {
	int16_t hx, hy;

	hx = I->mx;
	hy = I->my;

	if (I->board_state == IMU_BOARD_DOWN) {
		if (hx < 0 && hy < 0)
		{
			if (fabs(hx / hy) >= 1)
			{
				I->q0 = -0.005;
				I->q1 = -0.199;
				I->q2 = 0.979;
				I->q3 = -0.0089;
			}
			else
			{
				I->q0 = -0.008;
				I->q1 = -0.555;
				I->q2 = 0.83;
				I->q3 = -0.002;
			}

		}
		else if (hx < 0 && hy > 0)
		{
			if (fabs(hx / hy)>=1)
			{
				I->q0 = 0.005;
				I->q1 = -0.199;
				I->q2 = -0.978;
				I->q3 = 0.012;
			}
			else
			{
				I->q0 = 0.005;
				I->q1 = -0.553;
				I->q2 = -0.83;
				I->q3 = -0.0023;
			}

		}
		else if (hx > 0 && hy > 0)
		{
			if (fabs(hx / hy) >= 1)
			{
				I->q0 = 0.0012;
				I->q1 = -0.978;
				I->q2 = -0.199;
				I->q3 = -0.005;
			}
			else
			{
				I->q0 = 0.0023;
				I->q1 = -0.83;
				I->q2 = -0.553;
				I->q3 = 0.0023;
			}

		}
		else if (hx > 0 && hy < 0)
		{
			if (fabs(hx / hy) >= 1)
			{
				I->q0 = 0.0025;
				I->q1 = 0.978;
				I->q2 = -0.199;
				I->q3 = 0.008;
			}
			else
			{
				I->q0 = 0.0025;
				I->q1 = 0.83;
				I->q2 = -0.56;
				I->q3 = 0.0045;
			}
		}
	}

	else {
		if (hx < 0 && hy < 0)
		{
			if (fabs(hx / hy) >= 1)
			{
				I->q0 = 0.195;
				I->q1 = -0.015;
				I->q2 = 0.0043;
				I->q3 = 0.979;
			}
			else
			{
				I->q0 = 0.555;
				I->q1 = -0.015;
				I->q2 = 0.006;
				I->q3 = 0.829;
			}

		}
		else if (hx < 0 && hy > 0)
		{
			if(fabs(hx / hy) >= 1)
			{
				I->q0 = -0.193;
				I->q1 = -0.009;
				I->q2 = -0.006;
				I->q3 = 0.979;
			}
			else
			{
				I->q0 = -0.552;
				I->q1 = -0.0048;
				I->q2 = -0.0115;
				I->q3 = 0.8313;
			}

		}
		else if (hx > 0 && hy > 0)
		{
			if(fabs(hx / hy) >= 1)
			{
				I->q0 = -0.9785;
				I->q1 = 0.008;
				I->q2 = -0.02;
				I->q3 = 0.195;
			}
			else
			{
				I->q0 = -0.9828;
				I->q1 = 0.002;
				I->q2 = -0.0167;
				I->q3 = 0.5557;
			}

		}
		else if (hx > 0 && hy < 0)
		{
			if(fabs(hx / hy) >= 1)
			{
				I->q0 = -0.979;
				I->q1 = 0.0116;
				I->q2 = -0.0167;
				I->q3 = -0.195;
			}
			else
			{
				I->q0 = -0.83;
				I->q1 = 0.014;
				I->q2 = -0.012;
				I->q3 = -0.556;
			}
		}
	}
}

/* IMU的主任务 */
void IMU_MainTask(IMU_TypeDef* I) {
	static float norm;
	static float hx, hy, hz, bx, bz;
	static float vx, vy, vz, wx, wy, wz;
	static float ex, ey, ez, halfT;
	static float tempq0,tempq1,tempq2,tempq3;
	static float gx, gy, gz, ax, ay, az, mx, my, mz;
	static float last_update, now_update;
	static float q0q0;
	static float q0q1;
	static float q0q2;
	static float q0q3;
	static float q1q1;
	static float q1q2;
	static float q1q3;
	static float q2q2;
	static float q2q3;
	static float q3q3;

	if (I->tick_fresh == 1) I->tick_fresh = 0;
	else return;

	IMU_GetData(I);

	q0q0 = I->q0*I->q0;
	q0q1 = I->q0*I->q1;
	q0q2 = I->q0*I->q2;
	q0q3 = I->q0*I->q3;
	q1q1 = I->q1*I->q1;
	q1q2 = I->q1*I->q2;
	q1q3 = I->q1*I->q3;
	q2q2 = I->q2*I->q2;
	q2q3 = I->q2*I->q3;
	q3q3 = I->q3*I->q3;

	gx = I->wx;
	gy = I->wy;
	gz = I->wz;
	ax = I->ax;
	ay = I->ay;
	az = I->az;
	mx = I->mx;
	my = I->my;
	mz = I->mz;

	now_update = HAL_GetTick();
	halfT = ((float)(now_update - last_update) / 2000.0f);
	I->internal = now_update - last_update;
	last_update = now_update;

	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	if (I->use_ist == IMU_USE_IST8310) {
		norm = inv_sqrt(mx*mx + my*my + mz*mz);
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm;
	}
	else {
		mx = 0;
		my = 0;
		mz = 0;
	}

	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;

	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);

	/*
	 * error is sum of cross product between reference direction
	 * of fields and direction measured by sensors
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		I->exInt = I->exInt + ex * Ki * halfT;
		I->eyInt = I->eyInt + ey * Ki * halfT;
		I->ezInt = I->ezInt + ez * Ki * halfT;

		gx = gx + Kp*ex + I->exInt;
		gy = gy + Kp*ey + I->eyInt;
		gz = gz + Kp*ez + I->ezInt;
	}

	tempq0 = I->q0 + (-I->q1*gx - I->q2*gy - I->q3*gz) * halfT;
	tempq1 = I->q1 + (I->q0*gx + I->q2*gz - I->q3*gy) * halfT;
	tempq2 = I->q2 + (I->q0*gy - I->q1*gz + I->q3*gx) * halfT;
	tempq3 = I->q3 + (I->q0*gz + I->q1*gy - I->q2*gx) * halfT;

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	I->q0 = tempq0 * norm;
	I->q1 = tempq1 * norm;
	I->q2 = tempq2 * norm;
	I->q3 = tempq3 * norm;

	/* yaw    -pi----pi */
	I->yaw = -atan2(2*I->q1*I->q2 + 2*I->q0*I->q3, -2*I->q2*I->q2 - 2*I->q3*I->q3 + 1)* 57.3;
	/* pitch  -pi/2----pi/2 */
	I->pit = -asin(-2*I->q1*I->q3 + 2*I->q0*I->q2)* 57.3;
	/* roll   -pi----pi  */
	I->rol =  atan2(2*I->q2*I->q3 + 2*I->q0*I->q1, -2*I->q1*I->q1 - 2*I->q2*I->q2 + 1)* 57.3;
}

/* 放在1ms定时器周期回调函数中 */
void IMU_Update(IMU_TypeDef* I) {
	++ I->tick;
	if (I->tick >= I->sample_period) {
		I->tick = 0;
		if (I->tick_fresh == 0) {
			IMU_Callback(I);
			I->tick_fresh = 1;
		}
	}
}

/* 载入一个IMU单元 */
IMU_TypeDef IMU_Open(SPI_HandleTypeDef* hspi, uint32_t sample_period) {

	uint8_t i = 0;
	IMU_TypeDef I;
	uint8_t MPU6500_Init_Data[10][2] = {
			{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */
			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */
			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */
			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */
			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */
			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */
			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */
			{ MPU6500_USER_CTRL, 0x20 } 	  /* Enable AUX */
	};

	HAL_Delay(100);
	I.spi = hspi;
	I.id = MPU_ReadByte(&I, MPU6500_WHO_AM_I);
	I.tick = 0;

	for (i = 0; i < 10; i++) {
		MPU_WriteByte(&I, MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(1);
	}

	mpu_set_gyro_fsr(&I, 3);
	mpu_set_accel_fsr(&I, 2);
	ist8310_init(&I);
	MPU_OffsetCall(&I);
	IMU_InitQuaternion(&I);
	I.sample_period = sample_period;
	I.use_ist = IMU_USE_IST8310;
	I.board_state = IMU_BOARD_DOWN;
	I.tick_fresh = 0;

	return I;
}

/* 检查IMU是否创建成功 */
uint8_t IMU_CheckSuccess(IMU_TypeDef* I) {
	if (I->id == MPU6500_ID) return HAL_OK;
	return HAL_ERROR;
}

/* IMU更新回调函数 */
__weak void IMU_Callback(IMU_TypeDef* imu) {
	/* 用户重定义 */
}

/* IMU_BOARD_UP/DOWN */
void IMU_SetBoardState(IMU_TypeDef* I, uint8_t board_state) {
	I->board_state = board_state;
}
