#include "ebox.h"
#include "motors.h"

PWM motors[4] = {
		&PA0, 
		&PA1,
		&PA2,
		&PA3
};

void Motors_init(void)
{
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
		m1 = motor1 < 0 ? 0 : motor1 > 1000 ? 1000 : motor1;
		m2 = motor2 < 0 ? 0 : motor2 > 1000 ? 1000 : motor2;
		m3 = motor3 < 0 ? 0 : motor3 > 1000 ? 1000 : motor3;
		m4 = motor4 < 0 ? 0 : motor4 > 1000 ? 1000 : motor4;

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
