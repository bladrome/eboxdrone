#include "eboxcopter.h" 
/************************************/
// MPU6050			345us
// RC 1 channel		12433us
// mypid.compute()	25us

/************************************/

static double roll_pidoutput;
static double pitch_pidoutput;
static double yaw_pitdoutput;
static double throttle_output;

void Angle_control(void);
void Rate_control(void);
void Motors_control(void);

void setup(void)
{
		ebox_init();
		uart1.begin(115200);

		IMU_init();
		// led 5s
		IMU_calibrate();
		// led again
		RC_init();

		Motors_init();

		Timer_init();

}

int main()
{
		setup();

		while(true)
		{

				if(loop_100HZ_flag)
				{
						loop_100HZ_flag = false;

						IMU_estimate();

						Angle_control();

						Motors_control();

				}




				if(loop_50HZ_flag)
				{
						loop_50HZ_flag = false;

						RCdata_compute();
						
						Rate_control();

				}


		}
}



void Angle_control(void)
{
		roll_angle_pid.compute(RCANGLE[ROLL], imu.roll);
		pitch_angle_pid.compute(RCANGLE[PITCH], imu.pitch);
}
void Rate_control(void)
{
		roll_rate_pid.compute(roll_angle_pid.get_output(), imu.gyro[ROLL] * 180 / PI);
		pitch_rate_pid.compute(pitch_angle_pid.get_output(), imu.gyro[PITCH] * 180 / PI);
		yaw_rate_pid.compute(yaw_angle_pid.get_output(), imu.gyro[YAW] * 180 / PI);
}
void Motors_control(void)
{
		int m1, m2, m3, m4;

		throttle_output = RCDATA[THROTTLE];
		roll_pidoutput = roll_rate_pid.get_output();
		pitch_pidoutput = pitch_rate_pid.get_output();
		yaw_pitdoutput = yaw_rate_pid.get_output();

		// Mix table.
		m1 = throttle_output + roll_pidoutput - pitch_pidoutput + yaw_pitdoutput;
		m2 = throttle_output - roll_pidoutput - pitch_pidoutput - yaw_pitdoutput;
		m3 = throttle_output + roll_pidoutput + pitch_pidoutput - yaw_pitdoutput;
		m4 = throttle_output - roll_pidoutput + pitch_pidoutput + yaw_pitdoutput;

		Motors_flush(m1, m2, m3, m4);

}
