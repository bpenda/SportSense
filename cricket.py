#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2016 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################

from __future__ import division
from __future__ import with_statement
import signal
import socket
import time
import string
import sys
import getopt
import math
import thread
from array import *
import smbus
import select
import os
import logging
import csv
import RPi.GPIO as GPIO
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library


f = open('data.csv', 'w')
cur_time = time.time()


####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
class I2C:

    def __init__(self, address, bus=smbus.SMBus(1)):
        self.address = address
        self.bus = bus
        self.misses = 0

    def reverseByteOrder(self, data):
        "Reverses the byte order of an int (16-bit) or long (32-bit) value"
        # Courtesy Vishal Sapre
        dstr = hex(data)[2:].replace('L','')
        byteCount = len(dstr[::2])
        val = 0
        for i, n in enumerate(range(byteCount)):
            d = data & 0xFF
            val |= (d << (8 * (byteCount - i - 1)))
            data >>= 8
        return val

    def writeByte(self, value):
        while True:
            try:
                self.bus.write_byte(self.address, value)
                break
            except IOError, err:
                self.misses += 1

    def write8(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        while True:
            try:
                self.bus.write_byte_data(self.address, reg, value)
                break
            except IOError, err:
                self.misses += 1

    def writeList(self, reg, list):
        "Writes an array of bytes using I2C format"
        while True:
            try:
                self.bus.write_i2c_block_data(self.address, reg, list)
                break
            except IOError, err:
                self.misses += 1

    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                return result
            except IOError, err:
                self.misses += 1

    def readS8(self, reg):
        "Reads a signed byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                if (result > 127):
                    return result - 256
                else:
                    return result
            except IOError, err:
                self.misses += 1

    def readU16(self, reg):
        "Reads an unsigned 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readS16(self, reg):
        "Reads a signed 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                if (hibyte > 127):
                    hibyte -= 256
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readList(self, reg, length):
        "Reads a a byte array value from the I2C device"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result

    def getMisses(self):
        return self.misses


####################################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement.  Works with the Invensense IMUs:
#
#  - MPU-6050
#  - MPU-9150
#  - MPU-9250
#
#  The compass / magnetometer of the MPU-9250 is not used
#
####################################################################################################
class MPU6050 :
    i2c = None

    # Registers/etc.
    __MPU6050_RA_MAG_WIA = 0x00
    __MPU6050_RA_MAG_INFO = 0x01
    __MPU6050_RA_MAG_ST1 = 0x02          # 0x01 = Data ready for one shot polling
    __MPU6050_RA_MAG_HXL = 0x03          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HXH = 0x04          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HYL = 0x05          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HYH = 0x06          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HZL = 0x07          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_HZH = 0x08          # Note different byte order cf acc/gyro
    __MPU6050_RA_MAG_ST2 = 0x09          # Unlatch register for next sample - read 7 bytes each time
    __MPU6050_RA_MAG_CNTL1 = 0x0A        # 0x10 = 16-bit, 0x01 = one-shot polling, 0x02 = continuous
    __MPU6050_RA_MAG_CNTL2 = 0x0B        # Reset
    __MPU6050_RA_MAG_ASTC = 0x0C         # Self test
    __MPU6050_RA_MAG_TS1 = 0x0D          # Shipment test register
    __MPU6050_RA_MAG_TS2 = 0x0E          # Shipment test register
    __MPU6050_RA_MAG_I2CDIS = 0x0F       # I2C disable
    __MPU6050_RA_MAG_ASAX = 0x10         # Factory settings
    __MPU6050_RA_MAG_ASAY = 0x11         # Factory settings
    __MPU6050_RA_MAG_ASAZ = 0x12         # Factory settings
    __MPU6050_RA_XG_OFFS_USRH = 0x13
    __MPU6050_RA_XG_OFFS_USRL = 0x14
    __MPU6050_RA_YG_OFFS_USRH = 0x15
    __MPU6050_RA_YG_OFFS_USRL = 0x16
    __MPU6050_RA_ZG_OFFS_USRH = 0x17
    __MPU6050_RA_ZG_OFFS_USRL = 0x18
    __MPU6050_RA_SMPLRT_DIV = 0x19
    __MPU6050_RA_CONFIG = 0x1A
    __MPU6050_RA_GYRO_CONFIG = 0x1B
    __MPU6050_RA_ACCEL_CONFIG = 0x1C
    __MPU9250_RA_ACCEL_CFG_2 = 0x1D
    __MPU6050_RA_FF_THR = 0x1D
    __MPU6050_RA_FF_DUR = 0x1E
    __MPU6050_RA_MOT_THR = 0x1F
    __MPU6050_RA_MOT_DUR = 0x20
    __MPU6050_RA_ZRMOT_THR = 0x21
    __MPU6050_RA_ZRMOT_DUR = 0x22
    __MPU6050_RA_FIFO_EN = 0x23
    __MPU6050_RA_I2C_MST_CTRL = 0x24
    __MPU6050_RA_I2C_SLV0_ADDR = 0x25
    __MPU6050_RA_I2C_SLV0_REG = 0x26
    __MPU6050_RA_I2C_SLV0_CTRL = 0x27
    __MPU6050_RA_I2C_SLV1_ADDR = 0x28
    __MPU6050_RA_I2C_SLV1_REG = 0x29
    __MPU6050_RA_I2C_SLV1_CTRL = 0x2A
    __MPU6050_RA_I2C_SLV2_ADDR = 0x2B
    __MPU6050_RA_I2C_SLV2_REG = 0x2C
    __MPU6050_RA_I2C_SLV2_CTRL = 0x2D
    __MPU6050_RA_I2C_SLV3_ADDR = 0x2E
    __MPU6050_RA_I2C_SLV3_REG = 0x2F
    __MPU6050_RA_I2C_SLV3_CTRL = 0x30
    __MPU6050_RA_I2C_SLV4_ADDR = 0x31
    __MPU6050_RA_I2C_SLV4_REG = 0x32
    __MPU6050_RA_I2C_SLV4_DO = 0x33
    __MPU6050_RA_I2C_SLV4_CTRL = 0x34
    __MPU6050_RA_I2C_SLV4_DI = 0x35
    __MPU6050_RA_I2C_MST_STATUS = 0x36
    __MPU6050_RA_INT_PIN_CFG = 0x37
    __MPU6050_RA_INT_ENABLE = 0x38
    __MPU6050_RA_DMP_INT_STATUS = 0x39
    __MPU6050_RA_INT_STATUS = 0x3A
    __MPU6050_RA_ACCEL_XOUT_H = 0x3B
    __MPU6050_RA_ACCEL_XOUT_L = 0x3C
    __MPU6050_RA_ACCEL_YOUT_H = 0x3D
    __MPU6050_RA_ACCEL_YOUT_L = 0x3E
    __MPU6050_RA_ACCEL_ZOUT_H = 0x3F
    __MPU6050_RA_ACCEL_ZOUT_L = 0x40
    __MPU6050_RA_TEMP_OUT_H = 0x41
    __MPU6050_RA_TEMP_OUT_L = 0x42
    __MPU6050_RA_GYRO_XOUT_H = 0x43
    __MPU6050_RA_GYRO_XOUT_L = 0x44
    __MPU6050_RA_GYRO_YOUT_H = 0x45
    __MPU6050_RA_GYRO_YOUT_L = 0x46
    __MPU6050_RA_GYRO_ZOUT_H = 0x47
    __MPU6050_RA_GYRO_ZOUT_L = 0x48
    __MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    __MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    __MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    __MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    __MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    __MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    __MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    __MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    __MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    __MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    __MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    __MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    __MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    __MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    __MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    __MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    __MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    __MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    __MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    __MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    __MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    __MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    __MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    __MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    __MPU6050_RA_MOT_DETECT_STATUS = 0x61
    __MPU6050_RA_I2C_SLV0_DO = 0x63
    __MPU6050_RA_I2C_SLV1_DO = 0x64
    __MPU6050_RA_I2C_SLV2_DO = 0x65
    __MPU6050_RA_I2C_SLV3_DO = 0x66
    __MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67
    __MPU6050_RA_SIGNAL_PATH_RESET = 0x68
    __MPU6050_RA_MOT_DETECT_CTRL = 0x69
    __MPU6050_RA_USER_CTRL = 0x6A
    __MPU6050_RA_PWR_MGMT_1 = 0x6B
    __MPU6050_RA_PWR_MGMT_2 = 0x6C
    __MPU6050_RA_BANK_SEL = 0x6D
    __MPU6050_RA_MEM_START_ADDR = 0x6E
    __MPU6050_RA_MEM_R_W = 0x6F
    __MPU6050_RA_DMP_CFG_1 = 0x70
    __MPU6050_RA_DMP_CFG_2 = 0x71
    __MPU6050_RA_FIFO_COUNTH = 0x72
    __MPU6050_RA_FIFO_COUNTL = 0x73
    __MPU6050_RA_FIFO_R_W = 0x74
    __MPU6050_RA_WHO_AM_I = 0x75

    __SCALE_GYRO = 500.0 * math.pi / (65536 * 180)
    __SCALE_ACCEL = 8.0 / 65536                                                           #AB! +/-4g

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address
        self.ambient = 0

        self.num_i2c_errs = 0
        self.num_0g_hits = 0
        self.num_2g_hits = 0

        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0


        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets sample rate to 1kHz/(1+0) = 1kHz or 1ms (note 1kHz assumes dlpf is on - setting
        # dlpf to 0 or 7 changes 1kHz to 8kHz and therefore will require sample rate divider
        # to be changed to 7 to obtain the same 1kHz sample rate.
        #-------------------------------------------------------------------------------------------
        sample_rate_divisor = int(math.trunc(adc_frequency / sampling_rate))
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, sample_rate_divisor - 1)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets clock source to gyro reference w/ PLL
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Gyro DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 =  250Hz @ 8kHz sampling - DO NOT USE, THE ACCELEROMETER STILL SAMPLES AT 1kHz WHICH PRODUCES EXPECTED BUT NOT CODED FOR TIMING AND FIFO CONTENT PROBLEMS
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x0* FIFO overflow overwrites oldest FIFO contents
        # 0x4* FIFO overflow does not overwrite full FIFO contents
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x40 | glpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable gyro self tests, scale of +/- 250 degrees/s
        #
        # 0x00 =  +/- 250 degrees/s
        # 0x08 =  +/- 500 degrees/s
        # 0x10 = +/- 1000 degrees/s
        # 0x18 = +/- 2000 degrees/s
        # See SCALE_GYRO for converstion from raw data to units of radians per second
        #-------------------------------------------------------------------------------------------
        # int(math.log(degrees / 250, 2)) << 3
        self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Accel DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 = 460Hz
        # 0x01 = 184Hz
        # 0x02 =  92Hz
        # 0x03 =  41Hz
        # 0x04 =  20Hz
        # 0x05 =  10Hz
        # 0x06 =   5Hz
        # 0x07 = 460Hz
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU9250_RA_ACCEL_CFG_2, alpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable accel self tests, scale of +/-4g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        # int(math.log(g / 2, 2)) << 3
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x10)                             #AB! +/-4g
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push / pull, 50us pulse 0x10.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x10)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).                  #AB!
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enabled the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x40)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO later on - see flushFIFO()
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Set up the magnetometer: 0x10 = 16-bit, 0x01 = one-shot, 0x02 = continuous
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_MAG_CNTL1, 0x12)

        #-------------------------------------------------------------------------------------------
        # Read ambient temperature
        #-------------------------------------------------------------------------------------------
        temp = self.readTemperature()

    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def readCompass(self):
        compass_bytes = self.i2c.readList(self.__MPU6050_RA_MAG_HXL, 7)

        #-------------------------------------------------------------------------------------------
        # Convert the array of 6 bytes to 3 shorts - 7th byte kicks off another read
        #-------------------------------------------------------------------------------------------
        compass_data = []
        for ii in range(0, 6, 2):
            lobyte = compass_bytes[ii]
            hibyte = compass_bytes[ii + 1]
            if (hibyte > 127):
                hibyte -= 256

            compass_data.append((hibyte << 8) + lobyte)

        [mgx, mgy, mgz] = compass_data
        return mgx, mgy, mgz

    def enableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Set INT pin to push / pull, 50us pulse 0x10.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x10)

    def disableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Set INT pin to push / pull, 50us pulse 0x00.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)

    def readFIFO(self):
        #-------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data averaging, and return the averaged values and inferred time
        # based upon the sampling rate and the number of samples.
        #-------------------------------------------------------------------------------------------
        ax = 0.0
        ay = 0.0
        az = 0.0
        rx = 0.0
        ry = 0.0
        rz = 0.0

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
        fifo_batches = int(fifo_bytes / 12)  # This rounds down
        valid_batches = fifo_batches
        batch_size = 6   # signed shorts: ax, ay, az, gx, gy, gz

        batches = []
        for ii in range(fifo_batches):
            sensor_data = []
            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 12)
            for jj in range(0, 12, 2):
                hibyte = fifo_batch[jj]
                lobyte = fifo_batch[jj + 1]
                reading = (hibyte << 8) + lobyte
                if (hibyte > 127):
                    reading -= 65536

                sensor_data.append(reading)

            batches.append(sensor_data)
        #    cur_time = time.time()

        #    f.write(str(int(round(cur_time*1000))))
        for sensor_data in batches:
            f.write("")
            f.write(",")
            f.write(str(sensor_data[3]))
            f.write(",")
            f.write(str(sensor_data[4]))
            f.write(",")
            f.write(str(sensor_data[5]))
            f.write(",")
            f.write(str(sensor_data[0]))
            f.write(",")
            f.write(str(sensor_data[1]))
            f.write(",")
            f.write(str(sensor_data[2]))
            f.write('\n')

        return 

    def flushFIFO(self):
        #-------------------------------------------------------------------------------------------
        # First shut off the feed in the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Empty the FIFO by reading whatever is there
        #-------------------------------------------------------------------------------------------
        SMBUS_MAX_BUF_SIZE = 32

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(int(fifo_bytes / SMBUS_MAX_BUF_SIZE)):
            self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, SMBUS_MAX_BUF_SIZE)

        for ii in range(fifo_bytes % SMBUS_MAX_BUF_SIZE):
            self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        #-------------------------------------------------------------------------------------------
        # Finally start feeding the FIFO with sensor data again
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x78)

    def setGyroOffsets(self, gx, gy, gz):
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = ax * self.__SCALE_ACCEL
        qay = ay * self.__SCALE_ACCEL
        qaz = az * self.__SCALE_ACCEL

        qrx = gx * self.__SCALE_GYRO
        qry = gy * self.__SCALE_GYRO
        qrz = gz * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz



####################################################################################################
#
# GPIO pins initialization for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOInit(FIFOOverflowISR):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
    GPIO.add_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.RISING, FIFOOverflowISR)


####################################################################################################
#
# GPIO pins cleanup for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOTerm():
    GPIO.remove_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT)
    GPIO.cleanup()


####################################################################################################
#
# Class to split initialation, flight startup and flight control
#
####################################################################################################
class Quadcopter:

    MOTOR_LOCATION_FRONT = 0b00000001
    MOTOR_LOCATION_BACK =  0b00000010
    MOTOR_LOCATION_LEFT =  0b00000100
    MOTOR_LOCATION_RIGHT = 0b00001000

    MOTOR_ROTATION_CW = 1
    MOTOR_ROTATION_ACW = 2

    keep_looping = False
    shoot_video = False


    #===============================================================================================
    # One-off initialization
    #===============================================================================================
    def __init__(self):

        #===========================================================================================
        # Globals for the IMU setup
        # alpf               - the accelerometer low pass filter
        # glpf               - the gyro low pass filter
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        #===========================================================================================
        alpf = 1
        glpf = 1

        global adc_frequency
        global sampling_rate
        adc_frequency = 1000        #AB! defined by dlpf >= 1; DO NOT USE ZERO => 8000 adc_frequency
        sampling_rate = 1000        #AB! <= 500 to prevent FIFO overflow with diagnostics enabled

        global mpu6050
        mpu6050 = MPU6050(0x68, alpf, glpf)

        #-------------------------------------------------------------------------------------------
        # Set up the global constants - gravity in meters per second squared
        #-------------------------------------------------------------------------------------------
        GRAV_ACCEL = 9.80665

        mpu6050.flushFIFO()
        time.sleep(20 / sampling_rate)
        mpu6050.readFIFO()

        #-------------------------------------------------------------------------------------------
        # Set up the variaous timing constants and stats
        #-------------------------------------------------------------------------------------------
        esc_period = 0.01
        i_time = 0.005
        sampling_loops = 0
        motion_loops = 0
        start_flight = time.time()

        #-------------------------------------------------------------------------------------------
        # Flush the FIFO and enable the FIFO overflow interrupt
        #-------------------------------------------------------------------------------------------
        mpu6050.enableFIFOOverflowISR()
        mpu6050.flushFIFO()

        #===========================================================================================
        #
        # Motion and PID processing loop naming conventions
        #
        # qa? = quad frame acceleration
        # qg? = quad frame gravity
        # qr? = quad frame rotation
        # ea? = earth frame acceleration
        # eg? = earth frame gravity
        # ua? = euler angles between reference frames
        # ur? = euler rotation between frames
        #
        #===========================================================================================
        self.keep_looping = True
        while self.keep_looping:

            #---------------------------------------------------------------------------------------
            # Sleep for a while waiting for the FIFO to collect several batches of data.
            #---------------------------------------------------------------------------------------
            sleep_time = esc_period - i_time
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            #---------------------------------------------------------------------------------------
            # Now get the batch of averaged data from the FIFO.
            #---------------------------------------------------------------------------------------
            try:
                mpu6050.readFIFO()
            except IOError, err:
                keep_looping = False
                break

            #---------------------------------------------------------------------------------------
            # Track the number of motion loops and sampling loops; any discrepancy between these are the
            # missed samples or sampling errors.
            #---------------------------------------------------------------------------------------
            motion_loops += 1
            sampling_loops += i_time * sampling_rate

        mpu6050.disableFIFOOverflowISR()
        for esc in self.esc_list:
            esc.set(0)



    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        #-------------------------------------------------------------------------------------------
        # Stop the signal handler
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        #-------------------------------------------------------------------------------------------
        # Clean up the GPIO FIFO Overflow ISR
        #-------------------------------------------------------------------------------------------
        GPIOTerm()

        #-------------------------------------------------------------------------------------------
        # Reset the signal handler to default
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        sys.exit(0)

    ####################################################################################################
    #
    # Signal handler for Ctrl-C => abort cleanly
    #
    ####################################################################################################
    def shutdownSignalHandler(self, signal, frame):
        if not self.keep_looping:
            self.shutdown()
        self.keep_looping = False

    ####################################################################################################
    #
    # Signal handler for FIFO overflow 
    #
    ####################################################################################################
    def fifoOverflowISR(self, pin):
        print "FIFO OVERFLOW, ABORT"
        self.keep_looping = False

Quad = Quadcopter()
