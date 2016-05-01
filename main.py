#!/usr/bin/python
"""
Released under the MIT License
Copyright 2015 MrTijn/Tijndagamer
"""

# Import the MPU6050 class from the MPU6050.py file
from MPU6050 import MPU6050
import time
import RPi.GPIO as GPIO

# Create a new instance of the MPU6050 class
sensor = MPU6050(0x68)

sensor.set_accel_range(sensor.ACCEL_RANGE_16G)

f = open('data.csv', 'w')
cur_time = time.time()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
print "LED on"
GPIO.output(18,GPIO.HIGH)
print "LED off"
GPIO.output(18,GPIO.LOW)

while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    cur_time = time.time()

    f.write(str(int(round(cur_time*1000))))
    f.write(",")
    f.write(str(gyro_data['x'] / 131))
    f.write(",")
    f.write(str(gyro_data['y'] / 131))
    f.write(",")
    f.write(str(gyro_data['z'] / 131))
    f.write(",")
    f.write(str(accel_data['x']/sensor.ACCEL_SCALE_MODIFIER_16G))
    f.write(",")
    f.write(str(accel_data['y']/sensor.ACCEL_SCALE_MODIFIER_16G))
    f.write(",")
    f.write(str(accel_data['z']/sensor.ACCEL_SCALE_MODIFIER_16G))
    f.write('\n')
