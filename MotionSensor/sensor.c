#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "helper_3dmath.h"
#include "../MotionSensor.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "I2Cdev/I2Cdev.h"
#include "sensor.h"


/* The following functions must be defined for this platform:
 * i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data)
 * i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
 * delay_ms(uint32_t num_ms)
 * min(int a, int b)
 */
#define min(a,b) ((a)<(b)?(a):(b))
#define i2c_write   writeBytes
#define i2c_read(a,b,c,d)    (readBytes(a,b,c,d)!=-1?0:1)
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation
//(0 = success, !0 = error)
uint8_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t a[3];              // [x, y, z]            accel vector
int16_t a_buf[192];
int a_buf_size;
int16_t g[3];              // [x, y, z]            gyro vector
int32_t _q[4];
int32_t t;
int16_t c[3];

VectorFloat gravity;    // [x, y, z]            gravity vector

int r;
int initialized = 0;
int dmpReady = 0;
float lastval[3];
int16_t sensors;

float ypr[3];
Quaternion q; 
float temp;
float gyro[3];
float accel[3];
float compass[3];

uint16_t rate = 1000;

int ms_open() {
	dmpReady=1;
	initialized = 0;
	for (int i=0;i<DIM;i++){
		lastval[i]=10;
	}

	// initialize device
	printf("Initializing MPU...\n");
	if (mpu_init(NULL) != 0) {
		printf("MPU init failed!\n");
		return -1;
	}
	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to set sensors!\n");
		return -1;
	}
	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(2000)!=0) {
		printf("Failed to set gyro sensitivity!\n");
		return -1;
	}
	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(16)!=0) {
		printf("Failed to set accel sensitivity!\n");
		return -1;
	}

	// verify connection
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n",devStatus);

	//fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to initialize MPU fifo!\n");
		return -1;
	}


	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware()!=0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	printf("Setting sampling rate...\n");
    uint8_t dt = 0x07;
    i2c_write(0x68, 0x19, 1, &dt);
    delay_ms(100);
	if (mpu_set_sample_rate(1000)!=0) {
		printf("Failed to set sampling rate!\n");
		return -1;
	}
    delay_ms(100);

	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1)!=0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	//dmp_set_orientation()
	//if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) {
		printf("Failed to enable DMP features!\n");
		return -1;
	}


	printf("Setting DMP fifo rate...\n");
	if (dmp_set_fifo_rate(rate)!=0) {
		printf("Failed to set dmp fifo rate!\n");
		return -1;
	}
	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo()!=0) {
		printf("Failed to reset fifo!\n");
		return -1;
	}

	printf("Checking... ");
	do {
		delay_ms(1000/rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
		r=dmp_read_fifo(g,a,_q,&sensors,&fifoCount);
	} while (r!=0 || fifoCount<5); //packtets!!!
	printf("Done.\n");

	initialized = 1;

    uint16_t srate;
    srate = 0;
    dmp_get_fifo_rate(&srate);
    printf("FIFO rate: %d \n", (int)srate );
    srate = 0;
    mpu_get_sample_rate(&srate);
    printf("Sampling rate: %d \n", (int)srate );
	return 0;
}

int ms_update() {
	if (!dmpReady) {
		printf("Error: DMP not ready!!\n");
		return -1;
	}
    a_buf_size = 0;
    ssize_t read;
    do{
        read = dmp_read_fifo(g,a,_q,&sensors,&fifoCount);
        if(a[0] == 0 && a[1] == 0 && a[2] == 0){
            printf("fifocount: %d\n", fifoCount);
        }
        a_buf[a_buf_size] = a[0];
        a_buf[a_buf_size + 1] = a[1];
        a_buf[a_buf_size + 2] = a[2];
        a_buf_size += 3;
    }
	while (read !=0); //gyro and accel can be null because of being disabled in the efeatures

	q = _q;
	//0=gyroX, 1=gyroY, 2=gyroZ
	//swapped to match Yaw,Pitch,Roll
	//Scaled from deg/s to get tr/s
	for (int i=0;i<DIM;i++){
		gyro[i]   = (float)(g[DIM-i-1])/131.0/360.0;
		accel[i]   = (float)(a[DIM-i-1]);
		compass[i] = (float)(c[DIM-i-1]);
	}

	return 0;
}

int ms_close() {
	return 0;
}

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
	v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
	v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
	v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
	return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
	return 0;
}

