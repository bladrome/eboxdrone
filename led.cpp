#include "led.h"

#define led   PB8

#define		SLOW_FLASH	(100)
#define		FAST_FLASH	(600)

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
		Led_flasher(200);
		switch(event)
		{
				case IMU_CALIBRATION:
						{
								fast = 1;
								slow = 3;
								while(slow--)
										Led_flasher(SLOW_FLASH);
								while(fast--)
										Led_flasher(FAST_FLASH);
								break;
						}
				case ACC_CALIBRATION:
						{
								fast = 2;
								slow = 3;
								while(slow--)
										Led_flasher(SLOW_FLASH);
								while(fast--)
										Led_flasher(FAST_FLASH);

								break;
						}
				case GYRO_CALIBRATION:
						{
								fast = 3;
								slow = 3;
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
