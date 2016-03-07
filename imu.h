#ifndef IMU_H_
#define IMU_H_
#include <math.h>
#include "ebox.h"
#include "mpu6050.h"

#define CONSTANTS_G					9.80665f		

#define IMU_SAMPLE_FREQ				100
#define IMU_CUTOFF_FREQ				30


#define ACC_CALC_TIME				3000		//ms
#define GYRO_CALC_TIME				3000000l	//us

extern MPU6050 mpu;

extern uint8_t	imu_cali_flag;

static void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw);

void	IMU_init(void);
void	IMU_update_sensors(void);
uint8_t IMU_check(void);
uint8_t IMU_calibrate(void);
uint8_t IMU_calibrate_tmp(void);


enum{
		X,
		Y,
		Z
};

typedef struct imu_tt
{
		uint8_t caliPass;
		uint8_t ready;
		int16_t accADC[3];
		int16_t gyroADC[3];
		int16_t magADC[3];
		float 	accRaw[3];		//m/s^2
		float 	gyroRaw[3];		//rad/s 
		float 	magRaw[3];		//
		float   accOffset[3];		//m/s^2
		float   gyroOffset[3]; 
		float   accb[3];		//filted, in body frame
		float   accg[3];
		float   gyro[3];
		float   DCMgb[3][3];
		float   q[4];
		float   roll;				//deg
		float   pitch;
		float 	yaw;
		float   rollRad;				//rad
		float   pitchRad;
		float 	yawRad;
}imu_t;

extern imu_t	imu;


//#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			[> kg/m^3		<]
//#define CONSTANTS_AIR_GAS_CONST				287.1f 			[> J/(kg * K)		<]
//#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		[> 癈			<]
//#define CONSTANTS_RADIUS_OF_EARTH			6371000			[> meters (m)		<]

//extern volatile float accFilted[3],gyroFilted[3];
//extern float DCMbg[3][3],DCMgb[3][3];
//extern float accZoffsetTemp;
//extern float IMU_Pitch,IMU_Roll,IMU_Yaw;


//typedef float  quad[4];
//typedef float  vector3f[3];	//不可作为返回值，指针
//typedef float  matrix3f[3][3];

//typedef struct mat3_tt
//{
		//float m[3][3];
//}mat3;

//typedef struct vec3_tt
//{
		//float v[3];
//}vec3;

#endif

