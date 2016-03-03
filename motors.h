#ifndef MOTORS_H
#define MOTORS_H

extern PWM motors[4];

void Motors_init(void);

void Motors_flush(int motor1,
				 int motor2,
				 int motor3,
				 int motor4);

void Motors_start(int idlespeed);

void Motors_stop(void);

#endif
