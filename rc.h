#ifndef RC_H
#define RC_H
#include "ebox.h"


#define  DEADBAND			(6			)
#define  ANGLE_MAX			(40			)
#define  YAW_RATE_MAX		(180  /	  PI)
#define  CHANNELS			(4			)
#define  LPF_RC_FACTOR		(3			)
#define  ROLL_ZERO			(500		)
#define  PITCH_ZERO			(500		)
#define  THROTTLE_ZERO		(0			)
#define  YAW_ZERO			(500		)


extern  int32_t RCDATA[CHANNELS];
extern  double  RCANGLE[CHANNELS];

enum
{
		ROLL = 0,
		PITCH = 1,
		YAW = 2,
		THROTTLE = 3
		
};

void RC_init(void);
void RCdata_compute(void);

#endif
