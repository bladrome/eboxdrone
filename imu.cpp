#include "imu.h"
#include "filter.h"



ELPF    acc_x_lpf;
ELPF    acc_y_lpf;
ELPF    acc_z_lpf;
ELPF    gyro_x_lpf;
ELPF    gyro_y_lpf;
ELPF    gyro_z_lpf;

MPU6050 mpu(&i2c2);

imu_t	imu = {
		0
};
uint8_t imu_cali_flag = 0;

//函数名：IMU_Init(void)
//描述：姿态解算融合初始化函数
//现在使用软件解算，不再使用MPU6050的硬件解算单元DMP，IMU_SW在SysConfig.h中定义
void IMU_init(void)
{
		mpu.begin(400000);
		imu.ready=0;
		imu.caliPass=0;
		//acc filter
		acc_x_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
		acc_y_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
		acc_z_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
		//gyro filter
		gyro_x_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
		gyro_y_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
		gyro_z_lpf.reset(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ);
}


//should place to a level surface and keep it stop for 1~2 second
//return 1 when finish
uint8_t IMU_calibrate(void)
{
		//3s 
		static float accSum[3] = { 0,0,0 };
		static float gyroSum[3] = { 0,0,0 };
		static uint16_t cnt = 0;
		static uint16_t tPrev = 0;
		static uint8_t calibrating = 0;
		uint8_t ret = 0;
		uint8_t i = 0;
		uint16_t dt = 0,now = 0;

		now = millis();
		dt = now - tPrev;	

		if(calibrating == 0)
		{
				calibrating = 1;
				for(i = 0; i < 3; i++)
				{
						accSum[i] = 0;
						gyroSum[i] = 0;
						cnt = 0;
						imu.ready = 0;
				}

		}

		if(dt >= 10)		//10ms 
		{
				if(cnt<300)
				{
						for(i = 0; i < 3; i++)
						{
								accSum[i] += imu.accRaw[i];		
								gyroSum[i] += imu.gyroRaw[i];
						}
						cnt++;
						tPrev = now;
				}
				else
				{
						for(i=0;i<3;i++)
						{
								imu.accOffset[i] = accSum[i] / (float)cnt;
								imu.gyroOffset[i] = gyroSum[i] / (float)cnt;
						} 

						imu.accOffset[2] = imu.accOffset[2] - CONSTANTS_G;

						calibrating = 0;

						imu.ready = 1;

						ret = 1;
						//tobe added: write to eeprom !!
				}
		}

		return ret;
}


#define SENSOR_MAX_G	8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W	2000.0f		//deg/s
#define ACC_SCALE		(SENSOR_MAX_G / 32768.0f)
#define GYRO_SCALE		(SENSOR_MAX_W / 32768.0f)
void IMU_update_sensors(void)
{

		//read raw
		mpu.get_data(ACCEL_XOUT_H, imu.accADC, 3);
//	uart1.printf("%d \n\r", imu.accADC[0]);
//	uart1.printf("%d \n\r", imu.accADC[1]);
//	uart1.printf("%d \n\r", imu.accADC[2]);
	//delay_ms(1000);
		mpu.get_data(GYRO_XOUT_H, imu.gyroADC, 3);
		//tutn to physical
		for(int i = 0; i < 3; ++i)
		{
				imu.accRaw[i] = (float)imu.accADC[i] * ACC_SCALE * CONSTANTS_G ;
				imu.gyroRaw[i] = (float)imu.gyroADC[i] * GYRO_SCALE * PI / 180.f;		//deg/s
		}

		imu.accb[0] = acc_x_lpf.apply(imu.accRaw[0]-imu.accOffset[0]);
		imu.accb[1] = acc_y_lpf.apply(imu.accRaw[1]-imu.accOffset[1]);
		imu.accb[2] = acc_z_lpf.apply(imu.accRaw[2]-imu.accOffset[2]);

		imu.gyro[0] = gyro_x_lpf.apply(imu.gyroRaw[0]);
		imu.gyro[1] = gyro_y_lpf.apply(imu.gyroRaw[1]);
		imu.gyro[2] = gyro_z_lpf.apply(imu.gyroRaw[2]); 

} 


#define ACCZ_TILT_MAX	0.05		// m/s^2
#define CHECK_TIME		5
uint8_t IMU_check(void)
{
		uint32_t	accZSum=0;
		float		accZb=0;

		for(int i = 0; i < CHECK_TIME; ++i)
		{
				imu.accADC[2] = mpu.get_data(ACCEL_ZOUT_H);
				accZSum += imu.accADC[2];
		} 
		imu.accRaw[2] = (float)(accZSum / (float)CHECK_TIME) * ACC_SCALE * CONSTANTS_G ;
		accZb = imu.accRaw[2] - imu.accOffset[2];	

		if((accZb > CONSTANTS_G - ACCZ_TILT_MAX ) && (accZb < CONSTANTS_G + ACCZ_TILT_MAX))
				imu.caliPass = 1;
		else
				imu.caliPass = 0;

		return imu.caliPass;

}


/*
   in standard sequence , roll-pitch-yaw , x-y-z
   angle in rad
   get DCM for ground to body

*/


static void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw) 
{
		float cosx, sinx, cosy, siny, cosz, sinz;
		float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

		cosx = cosf(roll * PI / 180.0f);
		sinx = sinf(roll * PI / 180.0f);
		cosy = cosf(pitch * PI / 180.0f);
		siny = sinf(pitch * PI / 180.0f);
		cosz = cosf(yaw * PI / 180.0f);
		sinz = sinf(yaw * PI / 180.0f);

		coszcosx = cosz * cosx;
		coszcosy = cosz * cosy;
		sinzcosx = sinz * cosx;
		coszsinx = sinx * cosz;
		sinzsinx = sinx * sinz;

		DCM[0][0] = coszcosy;
		DCM[0][1] = cosy * sinz;
		DCM[0][2] = -siny;
		DCM[1][0] = -sinzcosx + (coszsinx * siny);
		DCM[1][1] = coszcosx + (sinzsinx * siny);
		DCM[1][2] = sinx * cosy;
		DCM[2][0] = (sinzsinx) + (coszcosx * siny);
		DCM[2][1] = -(coszsinx) + (sinzcosx * siny);
		DCM[2][2] = cosy * cosx;

}



uint8_t IMU_calibrate_tmp(void)
{
		//3s 
		static float accSum[3] = { 0,0,0 };
		static float gyroSum[3] = { 0,0,0 };
		int cnt = 100;
		uint8_t ret = 0;
		uint8_t i = 0;


		for(i = 0; i < 3; i++)
		{
				accSum[i] = 0;
				gyroSum[i] = 0;
				imu.ready = 0;
		}

		while( cnt-- )
		{
				mpu.get_data(ACCEL_XOUT_H, imu.accADC, 3);
				mpu.get_data(GYRO_XOUT_H, imu.gyroADC, 3);
				//tutn to physical
				for(i = 0; i < 3; ++i)
				{
						imu.accRaw[i] = (float)imu.accADC[i] * ACC_SCALE * CONSTANTS_G ;
						imu.gyroRaw[i] = (float)imu.gyroADC[i] * GYRO_SCALE * PI / 180.f;		//deg/s
				}

				for(i = 0; i < 3; i++)
				{
						accSum[i] += imu.accRaw[i];		
						gyroSum[i] += imu.gyroRaw[i];
				}

		}

		cnt = 300;

		for(i=0;i<3;i++)
		{
				imu.accOffset[i] = accSum[i] / (float)cnt;
				imu.gyroOffset[i] = gyroSum[i] / (float)cnt;
		} 

		imu.accOffset[2] = imu.accOffset[2] - CONSTANTS_G;


		imu.ready = 1;

		ret = 1;

		return ret;
}
