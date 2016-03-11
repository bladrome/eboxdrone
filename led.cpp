#include "led.h"

#define		SLOW_FLASH	(200)
#define		FAST_FLASH	(20)

#define led   PB8

void Led_init(void);
void Led_flasher(int delay);



void Led_init(void)
{
		led.mode(OUTPUT_PP);
		led.reset();
}

void Led_flasher(int delay)
{
		led.set();
		delay_ms(delay);
		led.reset();
		delay_ms(delay);

}

void Led_event_flasher(int event)
{
		int fast = 0;
		int slow = 0;
		switch(event)
		{
				case IMU_CALIBRATION:
						{
								slow = 1;
								while(slow--)
										Led_flasher( 6 * SLOW_FLASH);
								break;
						}
				case ACC_CALIBRATION:
						{
								fast = 0;
								slow = 5;
								while(slow--)
										Led_flasher(SLOW_FLASH);
								while(fast--)
										Led_flasher(FAST_FLASH);

								break;
						}
				case GYRO_CALIBRATION:
						{
								fast = 30;
								slow = 0;
								while(slow--)
										Led_flasher(SLOW_FLASH);
								while(fast--)
										Led_flasher(FAST_FLASH);

								break;
						}
				default:
						break;
		}
}
