#ifndef LOG_H
#define LOG_H

#define LOG_EEPROM		(0x8001000)

struct s_log{
		float pitch_angle_pid[3];
		float roll_angle_pid[3];
		float yaw_angle_pid[3];
		float pitch_rate_pid[3];
		float roll_rate_pid[3];
		float yaw_rate_pid[3];
		float acc_offset[3];
		float gyro_offset[3];
};

extern struct s_log logtable;
extern bool Log_read(void);
extern bool Log_write(void);

#endif
