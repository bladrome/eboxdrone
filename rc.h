#ifndef RC_H
#define RC_H
#include "ebox.h"


#define  DEADBAND			(8			)
#define  ANGLE_MAX			(10			)
#define  YAW_RATE_MAX		(180  /	  PI)
#define  CHANNELS			(4			)
#define  LPF_RC_FACTOR		(2			)
#define  ROLL_ZERO			(500		)
#define  PITCH_ZERO			(500		)
#define  THROTTLE_ZERO		(0			)
#define  YAW_ZERO			(500		)


extern  int32_t RCDATA[CHANNELS];
extern  float  RCANGLE[CHANNELS];

static int RC_command_delay = 0;
static uint8_t RCcommand = 0;
enum
{
		ROLL = 0,
		PITCH = 1,
		YAW = 2,
		THROTTLE = 3
		
};

void RC_init(void);
void RCdata_compute(void);
int	RC_gesture(void);
#endif
