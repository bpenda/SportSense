#!/usr/bin/python
"""
Released under the MIT License
Copyright 2015 MrTijn/Tijndagamer
"""

# Import the MPU6050 class from the MPU6050.py file
from MPU6050 import MPU6050
from time import sleep

# Create a new instance of the MPU6050 class
sensor = MPU6050(0x68)

sensor.set_accel_range(sensor.ACCEL_RANGE_16G)

f = open('data.csv', 'w')
cur_time = time.time()

while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    cur_time = time.time()


    print(str(accel_data['x']))
    print(",")
    print(str(accel_data['y']))
    print(",")
    print(str(accel_data['z']))
    print("\n")

    f.write(str(int(round(cur_time*1000))))
    f.write(",")
    f.write(str(gyro_data['x'] / 131))
    f.write(",")
    f.write(str(gyro_data['y'] / 131))
    f.write(",")
    f.write(str(gyro_data['z'] / 131))
    f.write(",")
    f.write(str(accel_data['x']))
    f.write(",")
    f.write(str(accel_data['y']))
    f.write(",")
    f.write(str(accel_data['z']))
    f.write('\n')
