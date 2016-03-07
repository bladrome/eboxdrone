#include "ebox.h"
#include "rc.h"



const int fracfreq = 1;
const int offset   = 168;

static int32_t LPFRC[CHANNELS] = {
		ROLL_ZERO		* (1 << LPF_RC_FACTOR),
		PITCH_ZERO		* (1 << LPF_RC_FACTOR),
		YAW_ZERO		* (1 << LPF_RC_FACTOR),
		THROTTLE_ZERO	* (1 << LPF_RC_FACTOR)
};

 int32_t RCDATA[CHANNELS] = {
		ROLL_ZERO		,
		PITCH_ZERO		,
		YAW_ZERO		,
		THROTTLE_ZERO	
};

float RCANGLE[CHANNELS] = {
		0.0f
};

static IN_CAPTURE RcCh1(&PA6);
static IN_CAPTURE RcCh2(&PA7);
static IN_CAPTURE RcCh3(&PB0);
static IN_CAPTURE RcCh4(&PB1);


static uint32_t ic_chan1_high;
static uint32_t ic_chan1_low;
static uint32_t ic_chan2_high;
static uint32_t ic_chan2_low;
static uint32_t ic_chan3_high;
static uint32_t ic_chan3_low;
static uint32_t ic_chan4_high;
static uint32_t ic_chan4_low;
void chan1_mesure_frq(void);//输入捕获中断事件
void chan1_update_event(void);
void chan2_mesure_frq(void);//输入捕获中断事件
void chan2_update_event(void);
void chan3_mesure_frq(void);//输入捕获中断事件
void chan3_update_event(void);
void chan4_mesure_frq(void);//输入捕获中断事件
void chan4_update_event(void);

uint32_t Get_pulse(uint8_t channel);

void RCdata_compute(void);

void Data2angle(void);

double Cut_deadband(double from, double to, double deadband);

void RC_init(void)
{
		RcCh1.begin(fracfreq);
		RcCh1.attch_ic_interrupt(chan1_mesure_frq);

		RcCh2.begin(fracfreq);
		RcCh2.attch_ic_interrupt(chan2_mesure_frq);

		RcCh3.begin(fracfreq);
		RcCh3.attch_ic_interrupt(chan3_mesure_frq);

		RcCh4.begin(fracfreq);
		RcCh4.attch_ic_interrupt(chan4_mesure_frq);

}

/*
 * RC TEST
int main(void)
{
		uint64_t preT = 0;
		uint64_t elapse = 0;
		int c1, c2, c3, c4;
		RC_init();
		uart1.begin(256000);
		while(1)
		{
				
			//	elapse = millis() - preT;
			//	preT = millis();

				while((c1 = Get_pulse(1)) == 0);
				while((c2 = Get_pulse(2)) == 0);
				while((c3 = Get_pulse(3)) == 0);
				while((c4 = Get_pulse(4)) == 0);

			  c1 /= 5;
				c1 -= 1200;
				c2 /= 5;
				c2 -= 1200;
				c3 /= 5;
				c3 -= 1200;
			
				c4 /= 5;
				c4 -= 1200;
				uart1.printf("%10d%10d%10d%10d\n", c1, c2, c3, c4);
			//delay_ms(20);
			
				//c1 = Get_pulse(1);
				//c2 = Get_pulse(2);
				//c3 = Get_pulse(3);
				//c4 = Get_pulse(4);
			
				uart1.printf("Elapse: %10d\n", millis() - preT);
				uart1.printf("chan 1 pulse:%6d\n",c1); 
				uart1.printf("chan 2 pulse:%6d\n",c2);
				uart1.printf("chan 3 pulse:%6d\n",c3);
				uart1.printf("chan 4 pulse:%6d\n",c4);
			

				//if(ic_chan1_high && ic_chan1_low)
				//{
				//uart1.printf("value1 = %d\r\n",ic_chan1_high);
				//uart1.printf("value2 = %d\r\n",ic_chan1_low);
				//uart1.printf("frq = %0.0f\r\n",(72000000.0/fracfreq)/(ic_chan1_low + ic_chan1_high));
				//uart1.printf("pluse = %0.2f\r\n",value1*100.0/(ic_chan1_low + ic_chan1_high));
				//ic_chan1_high = 0;
				//ic_chan1_low  = 0;
				//}
		}
}

*/

void RCdata_compute(void)
{
		int rcraw[CHANNELS];
		int32_t avg = 0;
		for(int i = 0; i < CHANNELS; ++i)
		{
				rcraw[i] = Get_pulse(i + 1) / 5 - 1200;
				avg = LPFRC[i] >> LPF_RC_FACTOR;
				LPFRC[i] += rcraw[i] - avg;
				RCDATA[i] = rcraw[i] < avg - 3 ? avg - 2 : rcraw[i] > avg + 3 ? avg + 2 : avg;
		}
		Data2angle();

}
void Data2angle(void)
{
		RCANGLE[ROLL] = ANGLE_MAX * Cut_deadband(RCDATA[ROLL] - 500, 500, DEADBAND);
		RCANGLE[PITCH] = ANGLE_MAX * Cut_deadband(RCDATA[PITCH] - 500, 500, DEADBAND);
		RCANGLE[YAW] = ANGLE_MAX * Cut_deadband(RCDATA[YAW] - 500, 500, DEADBAND);
		RCANGLE[THROTTLE] = ANGLE_MAX * Cut_deadband(RCDATA[THROTTLE], 0, 10);
		
}

double Cut_deadband(double from, double to, double deadband)
{
		if (from > deadband)
				return (from - deadband) / (to - deadband);
		else if(from < deadband)
				return (from + deadband) / (to - deadband);
		else
				return 0.0f;
}
uint32_t Get_pulse(uint8_t channel)
{
		float ret = 0;
		switch(channel)
		{
				case 1:
						{
								if (ic_chan1_high == 0)
										return 0;
								ret = (float)ic_chan1_high / (ic_chan1_high + ic_chan1_low) * 100000;
								ic_chan1_high = 0;
								ic_chan1_low  = 0;

								break;
						}
				case 2:
						{
								if (ic_chan2_high == 0)
										return 0;
								ret =  (float)ic_chan2_high / (ic_chan2_high + ic_chan2_low) * 100000;
								ic_chan2_high = 0;
								ic_chan2_low  = 0;

								break;
						}
				case 4:
						{
								if (ic_chan3_high == 0)
										return 0;
								ret =  (float)ic_chan3_high / (ic_chan3_high + ic_chan3_low) * 100000;
								ic_chan3_high = 0;
								ic_chan3_low  = 0;

								break;
						}
				case 3:
						{
								if (ic_chan4_high == 0)
										return 0;
								ret =  (float)ic_chan4_high / (ic_chan4_high + ic_chan4_low) * 100000;
								ic_chan4_high = 0;
								ic_chan4_low  = 0;

								break;
						}
				default:
						{
								ret = 500;
								break;
						}
		}
		
		return ret;
}



void chan1_mesure_frq(void)//输入捕获中断事件
{
		if(RcCh1.polarity == TIM_ICPOLARITY_FALLING)//测量高电平时间完成
		{
				ic_chan1_high = RcCh1.get_capture() + offset;//校正值，查表可得
				RcCh1.set_polarity_rising();//切换至测量低电平时间完成
		}
		else//测量低电平时间完成
		{
				ic_chan1_low = RcCh1.get_capture() + offset;//校正值，查表可得
				RcCh1.set_polarity_falling();//切换至测量高电平时间完成
		}
}
void chan2_mesure_frq(void)//输入捕获中断事件
{
		if(RcCh2.polarity == TIM_ICPOLARITY_FALLING)//测量高电平时间完成
		{
				ic_chan2_high = RcCh2.get_capture() + offset;//校正值，查表可得
				RcCh2.set_polarity_rising();//切换至测量低电平时间完成
		}
		else//测量低电平时间完成
		{
				ic_chan2_low = RcCh2.get_capture() + offset;//校正值，查表可得
				RcCh2.set_polarity_falling();//切换至测量高电平时间完成
		}
}
void chan3_mesure_frq(void)//输入捕获中断事件
{
		if(RcCh3.polarity == TIM_ICPOLARITY_FALLING)//测量高电平时间完成
		{
				ic_chan3_high = RcCh3.get_capture() + offset;//校正值，查表可得
				RcCh3.set_polarity_rising();//切换至测量低电平时间完成
		}
		else//测量低电平时间完成
		{
				ic_chan3_low = RcCh3.get_capture() + offset;//校正值，查表可得
				RcCh3.set_polarity_falling();//切换至测量高电平时间完成
		}
}
void chan4_mesure_frq(void)//输入捕获中断事件
{
		if(RcCh4.polarity == TIM_ICPOLARITY_FALLING)//测量高电平时间完成
		{
				ic_chan4_high = RcCh4.get_capture() + offset;//校正值，查表可得
				RcCh4.set_polarity_rising();//切换至测量低电平时间完成
		}
		else//测量低电平时间完成
		{
				ic_chan4_low = RcCh4.get_capture() + offset;//校正值，查表可得
				RcCh4.set_polarity_falling();//切换至测量高电平时间完成
		}
}

