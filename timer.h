#ifndef TIMER_H_
#define TIMER_H_

#include "ebox.h"

extern TIM  timer;

extern bool loop_50HZ_flag;
extern bool loop_100HZ_flag;

extern volatile int  loop_50HZ_count;
extern volatile int  loop_100HZ_count;
void update_counter(void);
void Timer_init(void);


#endif
