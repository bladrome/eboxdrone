#include "ebox.h"
#include "motors.h"

#define battery (&PA4)

int min(int a, int b);
int max(int a, int b);
int Battery_motors_compensate(void);

PWM motors[4] = {
		&PA0, 
		&PA1,
		&PA2,
		&PA3
};

void Motors_init(void)
{
		battery->mode(OUTPUT_PP);
		for(int i = 0; i < 4; ++i)
		{
				motors[i].begin(50, 0);
		}
		
		return ;
}

// Write commands to motors.
void Motors_flush(int motor1,
				 int motor2,
				 int motor3,
				 int motor4)
{
		int m1, m2, m3, m4;
		int over = 0;
		int batterycom = Battery_motors_compensate();

		motor1 += batterycom;
		motor2 += batterycom;
		motor3 += batterycom;
		motor4 += batterycom;

		if( motor1 > 1000 
				|| motor2 > 1000
				|| motor3 > 1000
				|| motor4 > 1000 )
		{
				over = max(motor1 - 1000,
								max(motor2 - 1000, 
										max(motor3 - 1000,
												motor4 - 1000)));
				
		}
		
		m1 = motor1 - over;
		m2 = motor2 - over;
		m3 = motor3 - over;
		m4 = motor4 - over;

		m1 = m1 < 0 ? 0 : m1;
		m2 = m2 < 0 ? 0 : m2;
		m3 = m3 < 0 ? 0 : m3;
		m4 = m4 < 0 ? 0 : m4;

		motors[0].set_duty(m1);
		motors[1].set_duty(m2);
		motors[2].set_duty(m3);
		motors[3].set_duty(m4);

		return ;
}

// Write IDLESPEED to motors.
void Motors_start(int idlespeed)
{
		for(int i = 0; i < 4; ++i)
				motors[i].set_duty(idlespeed);

		return ;
}

// Write ZERO to motors.
void Motors_stop(void)
{
		for(int i = 0; i < 4; ++i)
		{
				motors[i].set_duty(0);
		}

		return ;
}

int min(int a, int b)
{
		return a > b ? b : a;
}

int max(int a, int b)
{
		return a > b ? a : b;
}

int Battery_motors_compensate(void)
{
		int volt = 	analog_read_voltage(battery) - MED_BATTERY;
		float rate;
		if(volt > 0)
				rate = (float)(volt / HIGH_BATTERY - MED_BATTERY);
		else
				rate = (float)(volt / MED_BATTERY - LOW_BATTERY);

		return (BATTERY_COMPENSATE_MAX * rate);
}
