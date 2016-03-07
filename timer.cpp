#include "timer.h"

//static bool loop_10HZ_flag = false;
//static bool loop_20HZ_flag = false;
bool loop_50HZ_flag = false;
bool loop_100HZ_flag = false;

volatile int  loop_50HZ_count = 0;
volatile int  loop_100HZ_count = 0;

const int	loop_50HZ_overhead = 1000 / 50;
const int	loop_100HZ_overhead = 1000 / 100;


TIM timer(TIM4);

void Timer_init(void)
{

		timer.begin(1000);
		timer.attach_interrupt(update_counter);
		timer.interrupt(ENABLE);
		timer.start();

}

void update_counter(void)
{
		if( ++loop_50HZ_count >= loop_50HZ_overhead )
		{
				loop_50HZ_flag = true;
				loop_50HZ_count = 0;
		}
		if( ++loop_100HZ_count >= loop_100HZ_overhead )
		{
				loop_100HZ_flag = true;
				loop_100HZ_count = 0;
		}
		
		return ;
}
