#ifndef EBOXCOPTER_H
#define EBOXCOPTER_H

#include "ebox.h"
#include "timer.h"
#include "mpu6050.h"
#include "motors.h"
#include "filter.h"
#include "rc.h"
#include "imu.h"
#include "estimate.h"
#include "pidcontroller.h"


EPID pitch_angle_pid(0, 0, 0);
EPID roll_angle_pid(0, 0, 0);
EPID yaw_angle_pid(0, 0, 0);

EPID pitch_rate_pid(10, 0, 0);
EPID roll_rate_pid(10, 0, 0);
EPID yaw_rate_pid(0, 0, 0);


#endif
