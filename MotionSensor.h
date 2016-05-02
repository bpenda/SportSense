#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

extern float ypr[3]; //yaw, pitch, roll
extern float accel[3];
extern float gyro[3];
extern int16_t a_buf[192];
extern int a_buf_size;
extern float temp;
extern float compass[3];

extern int ms_open();
extern int ms_update();
extern int ms_close();

#endif
