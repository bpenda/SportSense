#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "MotionSensor.h"

#define delay_ms(a) usleep(a*1000)

int main() {
	ms_open();
    FILE * f = fopen("data.csv", "w");
    struct timeval time;
	do{
        if(!ms_update()){
            gettimeofday(&time, NULL);
            //printf("a_buf_size: %d\n", a_buf_size);
            for(int i = 0; i < a_buf_size; i += 3){
                fprintf(f, "%ld, 0, 0, 0, %d, %d, %d\n",time.tv_sec*1000000 + time.tv_usec, a_buf[i], a_buf[i+1], a_buf[i+2]);
            }
            a_buf_size = 0;
        }
	}while(1);

	return 0;
}
