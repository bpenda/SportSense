#include <stdio.h>
#include <pigpio.h>

#define LOOPS 500000

#define READ_BIT  0x80
#define MULTI_BIT 0x40

#define BW_RATE     0x2C
#define POWER_CTL   0x2D
#define DATA_FORMAT 0x31
#define DATAX0      0x32

int readBytes(int handle, char *data, int count)
{
	data[0] |= READ_BIT;
	if (count > 1) data[0] |= MULTI_BIT;
	return spiXfer(handle, data, data, count);
}

int writeBytes(int handle, char *data, int count)
{
	if (count > 1) data[0] |= MULTI_BIT;
	return spiWrite(handle, data, count);
}

int main(int argc, char *argv[])
{
	int h;
	char data[7];
	int16_t x, y, z, lx, ly, lz;
	int i;
	int bytes;
	double start, duration;

	if (gpioInitialise() < 0) return 1;

	h = spiOpen(0, 2000000, 3);

	data[0] = BW_RATE;
	data[1] = 0x0F;
	writeBytes(h, data, 2);

	data[0] = DATA_FORMAT;
	data[1] = 0x01;
	writeBytes(h, data, 2);

	data[0] = POWER_CTL;
	data[1] = 0x08;
	writeBytes(h, data, 2);

	lx = 0;
	ly = 0;
	lz = 0;

	start = time_time();

	for (i=0; i<LOOPS; i++)
	{
		data[0] = DATAX0;
		bytes = readBytes(h, data, 7);
		if (bytes == 7)
		{
			x = (data[2]<<8)|data[1];
			y = (data[4]<<8)|data[3];
			z = (data[6]<<8)|data[5];

			if ((abs(x-lx) > 30) || (abs(y-ly) > 30) || (abs(z-lz) > 30))
			{
				printf("x=%d y=%d z=%d\n", x, y, z);

				lx = x;
				ly = y;
				lz = z;
			}
		}
		else printf("spiXfer error (%d)\n", bytes);
	}

	duration = time_time() - start;

	gpioTerminate();

	printf("%d loops in %.2f seconds (%.1f sps)\n",
			LOOPS, duration, LOOPS/duration);
}
