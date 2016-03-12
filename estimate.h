#ifndef ESTIMATE_H
#define	ESTIMATE_H

// TODO:
// Maybe it is neccessary to specify a calibration, such as 
//		ACC_calibration()
//		GYro_calibration()
// Make a simple but efficent interface of above.

void IMU_estimate(void);

static float	inv_sqrt(float number);

static void		IMU_esti_init(float ax, float ay, float az, 
									float mx, float my, float mz);
static void		IMU_esti_update(float gx, float gy, float gz, 
										float ax, float ay, float az, 
										float mx, float my, float mz, 
										float twoKp, float twoKi, float dt);

#endif

