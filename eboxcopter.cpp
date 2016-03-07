#include "eboxcopter.h" 
/************************************/
// MPU6050			345us
// RC 1 channel		12433us
// mypid.compute()	25us

/************************************/

static float roll_pidoutput;
static float pitch_pidoutput;
static float yaw_pidoutput;
static float throttle_output;

void Angle_control(void);
void Rate_control(void);
void Motors_control(void);

void Led_shink(void);
void Uart_debugForRead(void);
void Uart_debugForMat(void);

void setup(void)
{
		ebox_init();
		uart1.begin(256000);
		IMU_init();
//		uart1.printf("%6f%6f%6f%6f\n", pitch_pidoutput, roll_pidoutput, throttle_output, yaw_pidoutput);
		IMU_calibrate_tmp();
		imu.ready = 1;
		RC_init();
		Motors_init();
		Motors_start(0);
		Timer_init();
}

int main()
{
		setup();

		while(true)
		{
				Uart_debugForRead();

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
	float tmp1,tmp2;
		roll_angle_pid.compute(RCANGLE[ROLL], imu.roll, tmp1);
		pitch_angle_pid.compute(RCANGLE[PITCH], imu.pitch, tmp2);
}
void Rate_control(void)
{
		float tmp1, tmp2, tmp3;
		roll_rate_pid.compute(roll_angle_pid.get_output(), (double)(imu.gyro[ROLL] * 180 / PI), tmp1);
		pitch_rate_pid.compute(pitch_angle_pid.get_output(), (double)(imu.gyro[PITCH] * 180 / PI), tmp2);
		yaw_rate_pid.compute(yaw_angle_pid.get_output(), (double)(imu.gyro[YAW] * 180 / PI), tmp3);
}
void Motors_control(void)
{
		int m1, m2, m3, m4;

		throttle_output = RCDATA[THROTTLE];
		roll_pidoutput = roll_rate_pid.get_output();
		pitch_pidoutput = pitch_rate_pid.get_output();
		yaw_pidoutput = yaw_rate_pid.get_output();

		// Mix table.
		m1 = throttle_output + roll_pidoutput - pitch_pidoutput + yaw_pidoutput;
		m2 = throttle_output - roll_pidoutput - pitch_pidoutput - yaw_pidoutput;
		m3 = throttle_output + roll_pidoutput + pitch_pidoutput - yaw_pidoutput;
		m4 = throttle_output - roll_pidoutput + pitch_pidoutput + yaw_pidoutput;

		Motors_flush(m1, m2, m3, m4);

}

void Led_shink(void)

{

		PWM pwm1(&PB8);
		pwm1.begin(1000,20);
		pwm1.set_oc_polarity(1);


		pwm1.set_duty(200);
		delay_ms(200);
		pwm1.set_duty(0);
		delay_ms(200);
		pwm1.set_duty(200);
		delay_ms(200);
		pwm1.set_duty(0);
		delay_ms(200);
		pwm1.set_duty(1000);
		delay_ms(1000);
		pwm1.set_duty(0);
		delay_ms(200);

}

void Uart_debugForRead(void)
{
		uart1.printf("-------------------\n");
		uart1.printf("RCdata   :%10d       %10d        %10d       %10d\n", RCDATA[0], RCDATA[1], RCDATA[2], RCDATA[3]);
		uart1.printf("RCangle  :%10.2f     %10.2f      %10.2f     %10.2f\n", RCANGLE[0], RCANGLE[1], RCANGLE[2], RCANGLE[3]);
		uart1.printf("ADC      :%10d       %10d        %10d\n",     imu.accADC[0], imu.accADC[1], imu.accADC[2]);
		uart1.printf("Acc      :%10.2f	   %10.2f      %10.2f      \n",imu.accb[0], imu.accb[1], imu.accb[2]);
		uart1.printf("Quad	   :%10.2f     %10.2f      %10.2f     %10.2f\n",imu.q[0], imu.q[1], imu.q[2]);
		uart1.printf("Euler    :%10.2f     %10.2f      %10.2f      \n", imu.roll, imu.pitch, imu.yaw);
		uart1.printf("PIDoutput:%10.2f     %10.2f      %10.2f     %10.2f\n", pitch_pidoutput, roll_pidoutput, throttle_output, yaw_pidoutput);
		uart1.printf("Ratepid  :%10d	   %10d		   %10d        \n",roll_rate_pid.get_output(),pitch_rate_pid.get_output() , yaw_rate_pid.get_output());
		uart1.printf("-------------------\n\n\n\n");

}

void Uart_debugForMat(void)
{
}