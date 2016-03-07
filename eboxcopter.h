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


EPID pitch_angle_pid(3.5, 0, 0);
 EPID roll_angle_pid(3.5, 0, 0);
EPID yaw_angle_pid(1, 0.2, 0);

EPID pitch_rate_pid(0.7, 0.5, 0.03);
EPID roll_rate_pid(0.7, 0.5, 0.03);
EPID yaw_rate_pid(20, 0, 0);


#endif
