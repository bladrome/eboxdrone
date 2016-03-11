#include "ebox.h"

enum led_enum{
		IMU_CALIBRATION = 0,
		ACC_CALIBRATION = 1,
		GYRO_CALIBRATION = 2
};
extern void Led_init(void);
extern void Led_event_flasher(int delay);


