#include "ebox.h"
#include "filter.h"
#include "mpu6050.h"

MPU6050 mpu(&i2c2);
ELPF accx(100, 30);
ELPF accy(100, 30);
ELPF accz(100, 30);
ELPF gyrox(100, 30);
ELPF gyroy(100, 30);
ELPF gyroz(100, 30);

uint8_t id;
void setup()
{
		ebox_init();
		mpu.begin(400000);
		uart1.begin(256000);
		mpu.get_id(&id);

}

int16_t tmp[7];

int main()
{
		int a, b, c, d, e, f;
		setup();

		while(1)
		{

				mpu.get_data(ACCEL_XOUT_H, tmp, 7);
				a = accx.apply(tmp[0]);
				b = accx.apply(tmp[1]);
				c = accx.apply(tmp[2]);
				d = accx.apply(tmp[4]);
				e = accx.apply(tmp[5]);
				f = accx.apply(tmp[6]);
				uart1.printf("%10d%10d%10d%10d%10d%10d\n", a, b, c, d, e, f);
			
				//delay_ms(10);

				
		}
}
