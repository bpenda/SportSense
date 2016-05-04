#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "MotionSensor.h"

void * read_top_accel(void * arg){
    unsigned long * args = (unsigned long *) arg;
    ms_open();
    struct timeval time;

    FILE * f =*((FILE **) args[1]);

    while(*((unsigned long *)args[0])){
        if(!ms_update()){
            gettimeofday(&time, NULL);
            //printf("a_buf_size: %d\n", a_buf_size);
            for(int i = 0; i < a_buf_size; i += 3){
                fprintf(f, "%ld, 0, 0, 0, %d, %d, %d\n",time.tv_sec*1000000 + time.tv_usec, a_buf[i], a_buf[i+1], a_buf[i+2]);
            }
            a_buf_size = 0;
        }
    }
    return NULL;
}

int main(int argc, char * argv[]){
#ifdef ACC2
    printf("acc2\n");
#endif
    pthread_t top_accel_thread;

    unsigned long reading = 0;
    FILE * f;
    unsigned long * tathread_args[2];

    tathread_args[0] = &reading;
    tathread_args[1] = (unsigned long *)&f;

    char user_in[64];

    struct tm *tm;
    time_t t;
    char str_time[100];
    char str_date[100];


    while(1){
        t = time(NULL);
        tm = localtime(&t);

        strftime(str_time, sizeof(str_time), "%H %M %S", tm);
        strftime(str_date, sizeof(str_date), "%d %m %Y", tm);

        printf("Enter 's' to start reading\n");
        scanf("%s", user_in);

        if(user_in[0] != 's')
            continue;

        char filename[128];
        sprintf(filename, "sportsense data/%s %s %s.csv", argv[1], str_date, str_time);
        f = fopen(filename, "w");

        reading = 1;
        pthread_create(&top_accel_thread, NULL, read_top_accel, tathread_args);

        printf("Reading accelerometer, enter 's' to stop\n");
        STOPSCAN:
        scanf("%s", user_in);
        if(user_in[0] != 's')
            goto STOPSCAN;

        reading = 0;
        pthread_join(top_accel_thread, NULL);
        printf("Stopped\n");
        fclose(f);

    }

    return 0;
}

