#ifndef MOTORS_H 
#define MOTORS_H
#include "ebox.h"

#define BATTERY_COMPENSATE_MAX	(80)
#define	LOW_BATTERY				(1500)
#define	MED_BATTERY				(2500)
#define HIGH_BATTERY			(3300)

extern PWM motors[4];

void Motors_init(void);

void Motors_flush(int motor1,
				 int motor2,
				 int motor3,
				 int motor4);

void Motors_start(int idlespeed);

void Motors_stop(void);

#endif
